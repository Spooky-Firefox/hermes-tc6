use embedded_dma::{ReadBuffer, WriteBuffer};

/// The SPI transport the driver talks to the MAC-PHY through.
///
/// # SPI bus requirements
///
/// The OPEN Alliance TC6 interface is **SPI mode 0** (CPOL = 0, CPHA = 0), **MSB first**, with
/// SCK up to the MAC-PHY's rated maximum (commonly 15-25 MHz; check the datasheet). This trait
/// does not configure the bus - the application must set it up before constructing the
/// implementor. A wrong mode or bit order typically shows up as `configure()` returning
/// [`ConfigError::BusUnresponsive`](crate::tc6_driver::ConfigError::BusUnresponsive) (IDVER reads
/// all-zero / all-ones) or [`ConfigError::HeaderRejected`](crate::tc6_driver::ConfigError::HeaderRejected)
/// on the very first control command (headers corrupted by a wrong sampling edge).
///
/// CSn is asserted low for a transaction and deasserted high at its end. Per spec 7.2 the host
/// must hold CSn deasserted for at least the MAC-PHY's `resync_timer` when switching between a
/// data transaction and a control transaction; ensuring that is currently the implementor's
/// responsibility (see `todo.md`).
///
/// # Safety
///
/// The implementor must ensure that when dropped, any ongoing DMA transfers are completed or
/// aborted first, so a DMA engine cannot write into freed memory.
pub trait Tc6Interface {
    type Error;
    // Todo add error type
    /// This function sends a block of data to the TC6 device and populates the result buffer with the response
    fn begin_send_block(
        &mut self,
        header: u32,
        data: impl ReadBuffer<Word = u8>,
        result: impl WriteBuffer<Word = u8>,
    ) -> Result<(), Self::Error>;

    // this function is called by the driver to check if the transfer is complete, it should return true if the transfer is complete and false if it is still in progress
    fn transfer_complete(&mut self) -> Result<bool, Self::Error>;

    /// Runs one control-mode SPI transaction (a register read or write), see TC6 spec Section 7.4.
    ///
    /// Unlike the data-chunk methods this is a plain full-duplex transfer: `tx` and `rx` must be
    /// the same length and the driver has already laid `tx` out as
    /// `[4-byte control command header][register data / don't-care ...][4-byte don't-care]`.
    /// `rx` receives the MISO stream, which is `tx` delayed by one 32-bit word:
    /// `[4 don't-care bytes][echoed control header][echoed write data / read data ...]`.
    ///
    /// Completion is reported the same way as data chunks, through
    /// [`transfer_complete`](Self::transfer_complete).
    fn begin_control_transaction(
        &mut self,
        tx: impl ReadBuffer<Word = u8>,
        rx: impl WriteBuffer<Word = u8>,
    ) -> Result<(), Self::Error>;

    /// This function is not required to be implemented as it falls back to the begin send block function
    /// but it is highly recommended to be implemented for performance reasons as we probably are IO bound
    fn begin_send_joined_block(
        &mut self,
        data: impl ReadBuffer<Word = u8>,
        mut result: impl WriteBuffer<Word = u8>,
    ) -> Result<(), Self::Error> {
        let data = unsafe { data.read_buffer() };
        let result_len = unsafe { result.write_buffer().1 };
        assert!(data.1 > 4, "data buffer must be at least 4 bytes long");
        assert_eq!(
            data.1, result_len,
            "data and result buffers must be the same length"
        );

        // The first 4 bytes hold the header as big-endian bytes (see the driver, which writes
        // `header.raw().to_be_bytes()`). Read it back byte-wise: `data.0` is only `u8`-aligned,
        // so `*data.0.cast::<u32>()` would be an unaligned read, and a plain cast would also pick
        // up the host's native endianness instead of the wire's.
        let header_bytes: &[u8; 4] = unsafe { core::slice::from_raw_parts(data.0, 4) }
            .try_into()
            .unwrap();
        let header = u32::from_be_bytes(*header_bytes);
        let data_block: &[u8; 64] = unsafe { core::slice::from_raw_parts(data.0.add(4), 64) }
            .try_into()
            .unwrap();

        self.begin_send_block(header, data_block, result)
    }
}
