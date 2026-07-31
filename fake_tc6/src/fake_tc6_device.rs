use crate::{
    fake_tc6_device::Mode::{Command, ReadingCommandHeader, ReadingHeader, ReadingWritingBlock},
    receive_footer::{self, ReceiveFooter},
    transmit_header::TransmitHeader,
};
use embedded_hal::spi::{self, Operation, SpiDevice};
use log::{info, trace, warn};
use std::vec;

const DEFAULT_BLOCK_SIZE: usize = 64;

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
            if self.device.is_reading_header() {
                warn!("mode is not block, please make sure you have written the header")
            }
            *byte = self.device.handle_byte(0)?
        }
        Ok(())
    }

    fn helper_write(&mut self, buf: &[u8]) -> Result<(), FakeTc6SpiDeviceError> {
        for byte in buf.iter() {
            self.device.handle_byte(*byte)?;
            if self.device.is_sending_footer() {
                info!("not reading the MISO footer")
            }
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
    block_size: usize,
    // the header the master, eg the mcu, have sent
    mosi_header_raw: [u8; 4],
    transmit_header: TransmitHeader,
    // the footer the mac-phy responds with
    miso_footer: [u8; 4],
    receive_footer: ReceiveFooter,

    // offset in frame buff that the current block output should start at IN WORDS NOT BYTES
    word_offset: isize,
    // buffer of eth frames to be sent on wire
    transmit_buff: vec::Vec<vec::Vec<u8>>,
    // buffer of received eth frames from wire
    receive_buff: vec::Vec<vec::Vec<u8>>,
}

impl FakeTc6Device {
    fn new() -> FakeTc6Device {
        FakeTc6Device {
            mode: ReadingHeader(0),
            block_size: DEFAULT_BLOCK_SIZE,
            mosi_header_raw: [0; 4],
            transmit_header: TransmitHeader::new(),
            miso_footer: [0; 4],
            word_offset: 0,
            transmit_buff: Vec::new(),
            receive_buff: Vec::new(),
            receive_footer: ReceiveFooter::new(),
        }
    }

    fn is_reading_header(&self) -> bool {
        matches!(self.mode, ReadingHeader(_) | ReadingCommandHeader(_))
    }

    fn is_sending_footer(&self) -> bool {
        matches!(self.mode, ReadingWritingBlock(i) if i >= self.block_size - 4)
    }

    fn handle_reading_header(&mut self, byte: u8, i: usize) -> Result<u8, FakeTc6DeviceError> {
        // when starting a new block, reset the receive footer to default values
        if i == 0 {
            self.receive_footer = ReceiveFooter::new();
        }

        let i2 = i;
        let out = *self
            .receive_buff
            .first_mut()
            .unwrap_or(&mut vec![])
            .get(i2 + self.word_offset as usize)
            .unwrap_or(&0);

        trace!("read byte {} of header", i);
        self.mosi_header_raw[i] = byte;
        self.mode = ReadingHeader(i + 1);
        if i == 3 {
            self.transmit_header = TransmitHeader::from_slice(&self.mosi_header_raw).unwrap();

            trace!(
                "header is {:?}\n{:?}",
                self.mosi_header_raw, self.transmit_header
            );
            if self.transmit_header.dnc {
                self.mode = ReadingWritingBlock(0);
            } else {
                self.mode = Command(0);
            }
        }
        Ok(out)
    }

    fn handle_reading_writing_block(
        &mut self,
        byte: u8,
        i: usize,
    ) -> Result<u8, FakeTc6DeviceError> {
        // if dv is set and the byte is in the data section, push it to the transmit buffer

        // if no sv and ev is set, the data section is from start of block to end of block
        // if sv is set and ev is not set, the data section is from swo*4 to end of block
        // if sv is not set and ev is set, the data section is from start of block to ebo
        // if ev and sv are set and swo*4 < ebo, the data section is from swo*4 to ebo
        // if ev and sv are set and swo*4 >= ebo, the data section is from start of block to EBO and SWO to end of block
        if self.transmit_header.dv {
            match (
                self.transmit_header.sv,
                self.transmit_header.ev,
                self.transmit_header.swo >= self.transmit_header.ebo,
            ) {
                // sv and ev are not set, data section is entire block
                (false, false, _) => {
                    // safe to push, we need to had a sv before this can occur, some one need to have pushed an empty vec
                    self.transmit_buff.last_mut().unwrap().push(byte);
                }
                // sv is set and ev is not set, data section is from swo*4 to end of block
                (true, false, _) => {
                    if i == self.transmit_header.swo as usize * 4 {
                        self.transmit_buff.push(vec![]);
                    }
                    if i >= self.transmit_header.swo as usize * 4 {
                        // safe to push, we need to had a sv before this can occur, some one need to have pushed an empty vec
                        self.transmit_buff.last_mut().unwrap().push(byte);
                    }
                }
                // sv is not set and ev is set, data section is from start of block to ebo
                (false, true, _) => {
                    if i <= self.transmit_header.ebo as usize {
                        // safe to push, we need to had a sv before this can occur, some one need to have pushed an empty vec
                        self.transmit_buff.last_mut().unwrap().push(byte);
                    }
                }
                // sv and ev are set, data section is from swo*4 to ebo if swo*4 < ebo, else from start of block to ebo and SWO to end of block
                (true, true, false) => {
                    if i == self.transmit_header.swo as usize * 4 {
                        self.transmit_buff.push(vec![]);
                    }
                    if i >= self.transmit_header.swo as usize * 4
                        && i <= self.transmit_header.ebo as usize
                    {
                        // safe to push, we need to had a sv before this can occur, some one need to have pushed an empty vec
                        self.transmit_buff.last_mut().unwrap().push(byte);
                    }
                }
                // sv and ev are set, data section is from swo*4 to ebo if swo*4 < ebo, else from start of block to ebo and SWO to end of block
                (true, true, true) => {
                    // NOTE ebo is never equal to SWO*4, as ebo points to the last byte of the frame, and SWO*4 points to the first byte of the frame,
                    // so if they are equal either the last byte from a frame is the same as the first byte of the next frame which is not possible,
                    // nor is a 1 byte frame possible.

                    if i == self.transmit_header.swo as usize * 4 {
                        self.transmit_buff.push(vec![]);
                    }

                    if i <= self.transmit_header.ebo as usize {
                        // safe to push, we need to had a sv before this can occur, some one need to have pushed an empty vec
                        self.transmit_buff.last_mut().unwrap().push(byte);
                    }
                    if i >= self.transmit_header.swo as usize * 4 {
                        // safe to push, we need to had a sv before this can occur, some one need to have pushed an empty vec
                        self.transmit_buff.last_mut().unwrap().push(byte);
                    }
                }
            }
        }
        let out = if i > self.block_size.saturating_sub(4) {
            // +1 because zero indexed
            self.miso_footer[i - (self.block_size - 4 + 1)]
        } else {
            self.receive_buff
                .first()
                .and_then(|v| v.get(i + self.word_offset as usize).cloned())
                .unwrap_or(0)
        };

        if i == self.block_size - 1 {
            // header indicates host listend, inc offset so next block contains
            self.mode = ReadingHeader(0)
        } else {
            self.mode = ReadingWritingBlock(i + 1);
        }
        Ok(out)
    }

    fn handle_command(&mut self, byte: u8, i: usize) -> Result<u8, FakeTc6DeviceError> {
        todo!()
    }

    fn handle_reading_command_header(
        &mut self,
        byte: u8,
        i: usize,
    ) -> Result<u8, FakeTc6DeviceError> {
        todo!()
    }

    fn handle_byte(&mut self, byte: u8) -> Result<u8, FakeTc6DeviceError> {
        let out = match self.mode.clone() {
            ReadingHeader(i) => self.handle_reading_header(byte, i)?,
            ReadingWritingBlock(i) => self.handle_reading_writing_block(byte, i)?,
            Command(i) => self.handle_command(byte, i)?,
            ReadingCommandHeader(i) => self.handle_reading_command_header(byte, i)?,
        };
        Ok(out)
    }

    pub fn assert_cs(&mut self) -> Result<(), FakeTc6SpiDeviceError> {
        Ok(())
    }
    pub fn de_assert_cs(&mut self) -> Result<(), FakeTc6DeviceError> {
        match self.mode {
            ReadingHeader(0) => Ok(()),
            // if in command docs says to revert to normal mode
            ReadingCommandHeader(0) => {
                self.mode = ReadingHeader(0);
                Ok(())
            }
            ReadingCommandHeader(_) | Command(_) => {
                Err(FakeTc6DeviceError::DeAssertCsNotEndOfCommand)
            }
            _ => Err(FakeTc6DeviceError::DeAssertCsNotEndOfBlock),
        }
    }

    /// this functions returns the byte that should be sent on the MISO line when the master is reading from the device.
    /// if there exist valid data to be written, it will set the dv in the footer.
    /// and it will also set the start word offset to the first byte of the frame that have data (condition, index + offset == 0)
    /// end byte will also be set to the last byte of the frame that have data (condition, index + offset == receive_buff.first().len() - 1)
    /// if a frame have been sent, it will be removed from the receive_buff and the offset will be updated
    /// if block is done it will uppdate offset so next block will start at the correct offset, if not norx
    fn get_out_byte_from_receive_buff(&mut self) -> u8 {
        let index = match self.mode {
            ReadingWritingBlock(i) => i + 4,
            ReadingHeader(i) => i,
            _ => {
                warn!("get_out_byte_from_receive_buff called in invalid mode");
                0
            }
        } as isize;

        // if we are the last byte of a frame set the end byte offset
        if index + self.word_offset * 4
            == self.receive_buff.first().map(|v| v.len()).unwrap_or(0) as isize - 1
        {
            self.receive_footer.ev = true;
            self.receive_footer.ebo = (index as u8) & 0b11_1111;
        }

        let out = if let Some(byte) = self
            .receive_buff
            .first()
            .and_then(|v| v.get((index + self.word_offset * 4) as usize))
        {
            self.receive_footer.dv = true;
            *byte
        } else {
            // if not norx and i word aligned then pop first receive buff and reset offset to -index/4
            // we also need to make sure that we are not expecting to transmit this to host later (negative offset))
            // if we have already had a start of a frame, we shuld not begin a new frame, as we can only send a signle start per block
            if !self.transmit_header.norx
                && index % 4 == 0
                && self.word_offset > 0
                && !self.transmit_header.sv
            {
                self.receive_buff.remove(0);
                // assume we are at index 7, the last byte of the second word, we have already sent sent al bytes of the frame,
                // the later first_mut will send zero.
                // we now go to the next byte, byte 8, the first byte of the third.
                // we need to set the offset to -2, so that the first_mut will send the first byte of the next frame.
                // 8 + word_offset * 4 = 0, word_offset = -2
                //word_offset *4= -8
                // word_offset = -index / 4;
                self.word_offset = -index / 4;
            }
            // cant just return zero here as we might have the next frame
            if let Some(byte) = self
                .receive_buff
                .first_mut()
                .and_then(|v| v.get((index + self.word_offset * 4) as usize))
            {
                self.receive_footer.dv = true;
                *byte
            } else {
                0
            }
        };

        // if we are at the start of a frame, set the start word offset to the first byte of the frame that have data
        if index + self.word_offset * 4 == 0 {
            self.receive_footer.sv = true;
            self.receive_footer.swo = (index as u8) >> 2;
        }

        // if last byte in block update offset so next block will start at the correct offset,
        // if norx is set, it meant that the host did not listen to the last block, so we need to keep the offset as is, so that the next block will be the same as this one
        // so the host will get the same data again, and can re-read the block ad infunitum until it listens to the block.
        if index == self.block_size as isize - 1 && !self.transmit_header.norx {
            self.word_offset -= (index + 1) / 4;
        }

        out
    }
}

#[derive(Clone, Copy, Debug)]

pub enum FakeTc6DeviceError {
    DeAssertCsNotEndOfBlock,
    DeAssertCsNotEndOfCommand,
}

impl Default for FakeTc6Device {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Clone, Debug)]
pub enum Mode {
    // reading the the header, the u represents how many bytes have been read
    ReadingHeader(usize),
    ReadingWritingBlock(usize),
    // echo command, the u32 represents how many bytes have been sent/read
    Command(usize),
    ReadingCommandHeader(usize),
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
    fn test_transferred_is_ok() {
        init();

        let mut tc6 = FakeTc6SpiDevice::new();
        let mut write_buff = [0u8; 64 + 4];
        write_buff[0] = 0b1000_0000; // dnc
        write_buff[3] = 0b1; // parity
        let mut read_buff = [0u8; 64 + 4];
        let res = tc6.transfer(read_buff.as_mut_slice(), write_buff.as_mut_slice());
        println!("{:?}", res);
        assert!(res.is_ok());
    }

    #[test]
    fn test_transferred_is_ok_with_header() {
        init();

        let header = TransmitHeader {
            dnc: false,
            seq: false,
            norx: false,
            dv: true,
            sv: true,
            swo: 0b1010,
            ev: true,
            ebo: 0b101010,
            tsc: 0b10,
            parity: false, // will be calculated
        };
    }

    // builds a TransmitHeader for a data chunk (dnc = true) with the given
    // dv/sv/swo/ev/ebo fields, parity is not checked by the device so it is left as false.
    fn make_header(dv: bool, sv: bool, swo: u8, ev: bool, ebo: u8) -> TransmitHeader {
        TransmitHeader {
            dnc: true,
            seq: false,
            norx: false,
            dv,
            sv,
            swo,
            ev,
            ebo,
            tsc: 0,
            parity: false,
        }
    }

    // builds a full block (DEFAULT_BLOCK_SIZE bytes) filled with `filler`, with `data` written
    // starting at byte offset `start`.
    fn make_block(start: usize, data: &[u8], filler: u8) -> [u8; DEFAULT_BLOCK_SIZE] {
        let mut block = [filler; DEFAULT_BLOCK_SIZE];
        block[start..start + data.len()].copy_from_slice(data);
        block
    }

    // sends one header + block chunk transaction (4 header bytes + DEFAULT_BLOCK_SIZE block
    // bytes) through the device, as a single SPI transfer.
    fn send_block(
        tc6: &mut FakeTc6SpiDevice,
        header: TransmitHeader,
        block: [u8; DEFAULT_BLOCK_SIZE],
    ) {
        let mut write_buff = [0u8; 4 + DEFAULT_BLOCK_SIZE];
        write_buff[..4].copy_from_slice(&header.to_bytes());
        write_buff[4..].copy_from_slice(&block);
        let mut read_buff = [0u8; 4 + DEFAULT_BLOCK_SIZE];
        tc6.transfer(read_buff.as_mut_slice(), write_buff.as_mut_slice())
            .expect("transfer should succeed");
    }

    // a frame that is fully contained within a single block (sv and ev set in the same chunk)
    #[test]
    fn test_frame_fully_inside_one_block() {
        init();
        let mut tc6 = FakeTc6SpiDevice::new();

        let frame = [1u8, 2, 3, 4, 5, 6, 7, 8];
        // swo = 1 word (byte offset 4), ebo = 11 -> bytes 4..=11 (8 bytes)
        let header = make_header(true, true, 1, true, 11);
        let block = make_block(4, &frame, 0xAA);

        send_block(&mut tc6, header, block);

        assert_eq!(tc6.device.transmit_buff, vec![frame.to_vec()]);
    }

    // a frame that starts at the end of one block (sv, no ev) and ends at the start of the
    // next block (ev, no sv)
    #[test]
    fn test_frame_starts_at_end_of_block_ends_at_start_of_next() {
        init();
        let mut tc6 = FakeTc6SpiDevice::new();

        // swo = 14 words -> byte offset 56, so bytes 56..=63 (8 bytes) belong to the frame
        let first_part = [1u8, 2, 3, 4, 5, 6, 7, 8];
        let header_a = make_header(true, true, 14, false, 0);
        let block_a = make_block(56, &first_part, 0xAA);
        send_block(&mut tc6, header_a, block_a);

        // continuation: no sv, ev with ebo = 3 -> bytes 0..=3 (4 bytes) belong to the frame
        let second_part = [9u8, 10, 11, 12];
        let header_b = make_header(true, false, 0, true, 3);
        let block_b = make_block(0, &second_part, 0xAA);
        send_block(&mut tc6, header_b, block_b);

        let expected: Vec<u8> = first_part
            .iter()
            .chain(second_part.iter())
            .copied()
            .collect();
        assert_eq!(tc6.device.transmit_buff, vec![expected]);
    }

    // a frame that starts in one block, fills a whole block in the middle (dv set, no sv/ev)
    // and ends in a third block
    #[test]
    fn test_frame_start_full_block_end() {
        init();
        let mut tc6 = FakeTc6SpiDevice::new();

        // start: swo = 8 words -> byte offset 32, so bytes 32..=63 (32 bytes) belong to the frame
        let start_part: Vec<u8> = (1u8..=32).collect();
        let header_a = make_header(true, true, 8, false, 0);
        let block_a = make_block(32, &start_part, 0xAA);
        send_block(&mut tc6, header_a, block_a);

        // middle: dv set, no sv/ev -> the whole block belongs to the frame
        let middle_part: Vec<u8> = (33u8..=96).collect();
        let header_b = make_header(true, false, 0, false, 0);
        let block_b: [u8; DEFAULT_BLOCK_SIZE] = middle_part.clone().try_into().unwrap();
        send_block(&mut tc6, header_b, block_b);

        // end: no sv, ev with ebo = 9 -> bytes 0..=9 (10 bytes) belong to the frame
        let end_part: Vec<u8> = (97u8..=106).collect();
        let header_c = make_header(true, false, 0, true, 9);
        let block_c = make_block(0, &end_part, 0xAA);
        send_block(&mut tc6, header_c, block_c);

        let expected: Vec<u8> = start_part
            .iter()
            .chain(middle_part.iter())
            .chain(end_part.iter())
            .copied()
            .collect();
        assert_eq!(tc6.device.transmit_buff, vec![expected]);
    }

    // two separate, complete frames, one fitting in block one and the other in block two
    #[test]
    fn test_two_frames_in_two_blocks() {
        init();
        let mut tc6 = FakeTc6SpiDevice::new();

        let frame_1 = [1u8, 2, 3, 4, 5, 6, 7, 8];
        let header_a = make_header(true, true, 0, true, 7);
        let block_a = make_block(0, &frame_1, 0xAA);
        send_block(&mut tc6, header_a, block_a);

        let frame_2 = [21u8, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32];
        let header_b = make_header(true, true, 0, true, 11);
        let block_b = make_block(0, &frame_2, 0xAA);
        send_block(&mut tc6, header_b, block_b);

        assert_eq!(
            tc6.device.transmit_buff,
            vec![frame_1.to_vec(), frame_2.to_vec()]
        );
    }

    // a frame split across two blocks with a dv = 0 block inserted between them, simulating
    // the host only receiving (no transmit data) for that chunk
    #[test]
    fn test_frame_with_dv_not_set_block_inserted() {
        init();
        let mut tc6 = FakeTc6SpiDevice::new();

        // start: swo = 8 words -> byte offset 32, so bytes 32..=63 (32 bytes) belong to the frame
        let start_part: Vec<u8> = (1u8..=32).collect();
        let header_a = make_header(true, true, 8, false, 0);
        let block_a = make_block(32, &start_part, 0xAA);
        send_block(&mut tc6, header_a, block_a);

        // host only receiving this chunk: dv = 0, sv/ev/swo/ebo must be 0 too
        let header_b = make_header(false, false, 0, false, 0);
        let block_b = [0xAAu8; DEFAULT_BLOCK_SIZE];
        send_block(&mut tc6, header_b, block_b);

        // end: no sv, ev with ebo = 9 -> bytes 0..=9 (10 bytes) belong to the frame
        let end_part: Vec<u8> = (33u8..=42).collect();
        let header_c = make_header(true, false, 0, true, 9);
        let block_c = make_block(0, &end_part, 0xAA);
        send_block(&mut tc6, header_c, block_c);

        let expected: Vec<u8> = start_part.iter().chain(end_part.iter()).copied().collect();
        assert_eq!(tc6.device.transmit_buff, vec![expected]);
    }
}
