use core::slice;

use embedded_hal::spi::SpiDevice;

/// [`Tc6Interface`](crate::tc6_interface::Tc6Interface) on top of a blocking `embedded-hal`
/// [`SpiDevice`]. [`transfer_complete`](crate::tc6_interface::Tc6Interface::transfer_complete)
/// always returns `true` (each call runs the transfer synchronously), so drive the driver with
/// `on_transmit_ready` / manual steps rather than expecting `on_interrupt` to yield mid-burst.
///
/// The `SpiDevice` must be configured for **SPI mode 0** (CPOL = 0, CPHA = 0), **MSB first**.
/// This wrapper does not enforce the spec-7.2 `resync_timer` CSn hold when switching between
/// data and control transactions; if you interleave `configure` / register reads with data
/// traffic, wrap a `SpiDevice` that inserts that delay (see `todo.md`).
pub struct EmbeddedHalImplementation<T: SpiDevice> {
    inner: T,
}

impl<T: SpiDevice> EmbeddedHalImplementation<T> {
    pub fn new(inner: T) -> Self {
        Self { inner }
    }

    pub fn inner(&self) -> &T {
        &self.inner
    }

    pub fn inner_mut(&mut self) -> &mut T {
        &mut self.inner
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EmbeddedHalImplementationError<T> {
    SpiError(T),
    BufferSizeMismatch,
    OtherError,
}

impl<T: SpiDevice> crate::tc6_interface::Tc6Interface for EmbeddedHalImplementation<T> {
    type Error = EmbeddedHalImplementationError<T::Error>;

    fn begin_send_block(
        &mut self,
        header: u32,
        data: impl embedded_dma::ReadBuffer<Word = u8>,
        mut result: impl embedded_dma::WriteBuffer<Word = u8>,
    ) -> Result<(), Self::Error> {
        let data = unsafe { slice::from_raw_parts(data.read_buffer().0, data.read_buffer().1) };
        let result =
            unsafe { slice::from_raw_parts_mut(result.write_buffer().0, result.write_buffer().1) };
        use embedded_hal::spi::Operation;
        let mut header_bytes = header.to_be_bytes();
        let (first, second) = result.split_at_mut(4);
        if second.len() != data.len() {
            return Err(EmbeddedHalImplementationError::BufferSizeMismatch);
        }
        self.inner
            .transaction(&mut [
                Operation::Transfer(first, header_bytes.as_mut_slice()),
                Operation::Transfer(second, data),
            ])
            .map_err(EmbeddedHalImplementationError::SpiError)
    }

    // embedded-hal is blocking so we can just return true here
    fn transfer_complete(&mut self) -> Result<bool, Self::Error> {
        Ok(true)
    }

    fn begin_control_transaction(
        &mut self,
        tx: impl embedded_dma::ReadBuffer<Word = u8>,
        mut rx: impl embedded_dma::WriteBuffer<Word = u8>,
    ) -> Result<(), Self::Error> {
        let tx = unsafe { slice::from_raw_parts(tx.read_buffer().0, tx.read_buffer().1) };
        let rx =
            unsafe { slice::from_raw_parts_mut(rx.write_buffer().0, rx.write_buffer().1) };
        if rx.len() != tx.len() {
            return Err(EmbeddedHalImplementationError::BufferSizeMismatch);
        }
        use embedded_hal::spi::Operation;
        self.inner
            .transaction(&mut [Operation::Transfer(rx, tx)])
            .map_err(EmbeddedHalImplementationError::SpiError)
    }
}
