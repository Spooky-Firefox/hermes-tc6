use embedded_dma::{ReadBuffer, WriteBuffer};

/// SAFETY: The implementor must ensure that when drop any ongoing DMA transfers are completed or aborted before the implementor is dropped. This is to ensure that the DMA transfer does not access invalid memory after the implementor is dropped.
pub trait Tc6Interface {
    type Error;
    // Todo add error type
    /// This function send a block of data to the TC6 device and populates the result buffer with the response
    fn begin_send_block(
        &mut self,
        header: u32,
        data: impl ReadBuffer<Word = u8>,
        result: impl WriteBuffer<Word = u8>,
    ) -> Result<(), Self::Error>;

    // this function is called by the driver to check if the transfer is complete, it should return true if the transfer is complete and false if it is still in progress
    fn transfer_complete(&mut self) -> Result<bool, Self::Error>;

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

        let header = unsafe { *data.0.cast::<u32>() };
        let data_block: &[u8; 64] = unsafe { core::slice::from_raw_parts(data.0.add(4), 64) }
            .try_into()
            .unwrap();

        self.begin_send_block(header, data_block, result)
    }
}
