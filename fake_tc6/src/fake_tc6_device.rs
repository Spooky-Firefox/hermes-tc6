use crate::{
    fake_tc6_device::Mode::{Command, ReadingHeader, ReadingWritingBlock},
    transmit_header::TransmitHeader,
};
use embedded_hal::spi::{self, Operation, SpiDevice};
use log::{info, trace, warn};
use std::vec;

const DEFAULT_BLOCK_SIZE: u16 = 64;

// exist because borrow checker
pub struct FakeTc6SpiDevice {
    device: FakeTc6Device,
}

impl FakeTc6SpiDevice {
    pub fn new() -> Self {
        FakeTc6SpiDevice {
            device: FakeTc6Device::new(),
        }
    }

    fn helper_read(&mut self, buf: &mut [u8]) -> Result<(), FakeTc6SpiDeviceError> {
        for byte in buf {
            if !matches!(self.device.mode, ReadingWritingBlock(_)) {
                warn!("mode is not block, please make sure you have written the header")
            }
            *byte = self.device.handle_byte(0)?
        }
        Ok(())
    }

    fn helper_write(&mut self, buf: &[u8]) -> Result<(), FakeTc6SpiDeviceError> {
        for (_i, byte) in buf.iter().enumerate() {
            self.device.handle_byte(*byte)?;
            //TODO add warning if write happens during footer output
            // if  >= self.device.block_size -4 {
            //     info!("not reading the MISO footer")
            // }
        }
        if !matches!(self.device.mode, ReadingHeader(0)) {
            warn!("Transaction did not end on starting point")
        }
        Ok(())
    }

    fn helper_transfer(
        &mut self,
        read: &mut [u8],
        write: &[u8],
    ) -> Result<(), FakeTc6SpiDeviceError> {
        for (i, write_byte) in write.iter().enumerate() {
            read[i] = self.device.handle_byte(*write_byte)?;
        }
        Ok(())
    }

    fn helper_transfer_in_place(&mut self, _buf: &mut [u8]) -> Result<(), FakeTc6SpiDeviceError> {
        todo!()
    }
}

impl Default for FakeTc6SpiDevice {
    fn default() -> Self {
        Self::new()
    }
}
pub struct FakeTc6Device {
    mode: Mode,
    block_size: u16,
    // the header the master, eg the mcu, have sent
    mosi_header_raw: [u8; 4],
    transmit_header: TransmitHeader,
    // the footer the mac-phy responds with
    miso_footer: [u8; 4],

    // offset in frame buff that the current block output shuld start att
    offset: usize,
    // buffer of eth frames to be sent on wire
    transmit_buff: vec::Vec<u8>,
    // buffer of received eth frames from wire
    receive_buff: vec::Vec<u8>,
}

impl FakeTc6Device {
    pub fn new() -> FakeTc6Device {
        FakeTc6Device {
            mode: ReadingHeader(0),
            block_size: DEFAULT_BLOCK_SIZE,
            mosi_header_raw: [0; 4],
            transmit_header: TransmitHeader::new(),
            miso_footer: [0; 4],
            offset: 0,
            transmit_buff: Vec::new(),
            receive_buff: Vec::new(),
        }
    }

    fn handle_byte(&mut self, byte: u8) -> Result<u8, FakeTc6DeviceError> {
        // todo add function so that state can return index into transaction
        let i2 = match self.mode {
            ReadingHeader(i) => i as usize,
            ReadingWritingBlock(i) => i as usize + 4,
            Command => todo!(),
        };
        let mut out = *self.transmit_buff.get(i2 + self.offset).unwrap_or(&0); // todo, add a seeded random here
        // handle if buff becomes empty and update end position in MISO header

        match self.mode {
            // TODO
            #[allow(clippy::ifs_same_cond)]
            ReadingHeader(i) => {
                trace!("read byte {} of header", i);
                self.mosi_header_raw[i as usize] = byte;
                self.mode = ReadingHeader(i + 1);
                if i == 3
                /*TODO && header is NOT command*/
                {
                    self.mode = ReadingWritingBlock(0);
                    trace!("header is {:?}", self.mosi_header_raw)
                } else if false
                /*TODO && i == 3 and header is command */
                {
                    self.mode = Command;
                }
            }
            ReadingWritingBlock(i) => {
                // if reading header is sending data AND i > (starting_offset << 2) and i < end_offset if exist
                // push byte

                if i > self.block_size.saturating_sub(4) {
                    // +1 because zero indexed
                    out = self.miso_footer[(i - (self.block_size - 4 + 1)) as usize]
                } else {
                    out = self
                        .receive_buff
                        .get(i as usize + self.offset)
                        .cloned()
                        .unwrap_or(0);
                }
                if i == self.block_size - 1 {
                    // header indicates host listend, inc offset so next block contains
                    if !self.transmit_header.norx {
                        self.offset += self.block_size as usize;
                    }
                    self.mode = ReadingHeader(0)
                } else {
                    self.mode = ReadingWritingBlock(i + 1);
                }
            }
            Command => todo!(),
        }
        Ok(out)
    }

    pub fn assert_cs(&mut self) -> Result<(), FakeTc6SpiDeviceError> {
        Ok(())
    }
    pub fn de_assert_cs(&mut self) -> Result<(), FakeTc6DeviceError> {
        //TODO handle command mode
        if !matches!(self.mode, ReadingHeader(0)) {
            Err(FakeTc6DeviceError::DeAssertCsNotEndOfBlock)
        } else {
            Ok(())
        }
    }
}

#[derive(Clone, Copy, Debug)]

pub enum FakeTc6DeviceError {
    DeAssertCsNotEndOfBlock,
}

impl Default for FakeTc6Device {
    fn default() -> Self {
        Self::new()
    }
}

pub enum Mode {
    // reading the the header, the u8 represents how many bytes have been read
    ReadingHeader(u8),
    ReadingWritingBlock(u16),
    Command,
}
impl SpiDevice for FakeTc6SpiDevice {
    fn transaction(
        &mut self,
        operations: &mut [embedded_hal::spi::Operation<'_, u8>],
    ) -> Result<(), Self::Error> {
        self.device.assert_cs()?;
        for op in operations.iter_mut() {
            match op {
                Operation::Read(items) => self.helper_read(items)?,
                Operation::Write(items) => self.helper_write(items)?,
                Operation::Transfer(items, items1) => self.helper_transfer(items, items1)?,
                Operation::TransferInPlace(items) => self.helper_transfer_in_place(items)?,
                Operation::DelayNs(_) => todo!("Time is currently not supported"),
            }
        }
        self.device.de_assert_cs()?;
        Ok(())
    }
}

#[derive(Clone, Copy, Debug)]
pub enum FakeTc6SpiDeviceError {
    DeviceError(FakeTc6DeviceError),
}

impl From<FakeTc6DeviceError> for FakeTc6SpiDeviceError {
    fn from(value: FakeTc6DeviceError) -> Self {
        FakeTc6SpiDeviceError::DeviceError(value)
    }
}

impl spi::ErrorType for FakeTc6SpiDevice {
    type Error = FakeTc6SpiDeviceError;
}

impl spi::Error for FakeTc6SpiDeviceError {
    fn kind(&self) -> spi::ErrorKind {
        todo!()
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use embedded_hal::spi::SpiDevice;

    fn init() {
        let _ = env_logger::builder().is_test(true).try_init();
    }

    #[test]
    fn test_transferred() {
        init();

        let mut tc6 = FakeTc6SpiDevice::new();
        let mut write_buff = [0u8; 64 + 4];
        let mut read_buff = [0u8; 64 + 4];
        let res = tc6.transfer(read_buff.as_mut_slice(), write_buff.as_mut_slice());
        println!("{:?}", res);
        assert!(res.is_ok());
    }
}
