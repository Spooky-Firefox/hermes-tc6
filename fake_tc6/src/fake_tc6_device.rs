use crate::{
    control_header::ControlHeader,
    fake_tc6_device::Mode::{Command, ReadingCommandHeader, ReadingHeader, ReadingWritingBlock},
    receive_footer::{self, ReceiveFooter},
    transmit_header::TransmitHeader,
};

/// Number of MMS 0 (standard control/status) register slots this fake models.
const MMS0_REGS: usize = 0x40;
use embedded_hal::spi::{self, Operation, SpiDevice};
use log::{info, trace, warn};
use std::vec;

const DEFAULT_BLOCK_SIZE: usize = 64;

// exist because borrow checker
pub struct FakeTc6SpiDevice {
    pub device: FakeTc6Device,
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
        for b in _buf.iter_mut() {
            *b = self.device.handle_byte(*b)?;
        }
        Ok(())
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

    // --- command mode (control register read/write, spec Section 7.4) ---
    // MMS 0 standard control/status register file (all 32-bit).
    regs_mms0: [u32; MMS0_REGS],
    // MOSI bytes seen so far in the current Command-mode transaction, so write data can be
    // echoed back (spec Section 7.4.4) and applied once each 32-bit word is complete.
    command_mosi: vec::Vec<u8>,
}

impl FakeTc6Device {
    fn new() -> FakeTc6Device {
        let mut dev = FakeTc6Device {
            mode: ReadingHeader(0),
            block_size: DEFAULT_BLOCK_SIZE,
            mosi_header_raw: [0; 4],
            transmit_header: TransmitHeader::new(),
            miso_footer: [0; 4],
            word_offset: 0,
            transmit_buff: Vec::new(),
            receive_buff: Vec::new(),
            receive_footer: ReceiveFooter::new(),
            regs_mms0: [0; MMS0_REGS],
            command_mosi: Vec::new(),
        };
        dev.reset_registers();
        dev
    }

    /// Restore the MMS 0 register file to power-on defaults: v1.1 IDVER, a STDCAP advertising
    /// cut-through / frame-timestamp / address-increment-disable and a 64-byte minimum chunk
    /// payload, CONFIG0 with CPS = 6 and SYNC = 0, STATUS0 with RESETC set, IMASK0 masking
    /// every maskable interrupt.
    fn reset_registers(&mut self) {
        self.regs_mms0 = [0; MMS0_REGS];
        self.regs_mms0[0x00] = 0x0000_0011; // IDVER  : MAJVER=1, MINVER=1
        self.regs_mms0[0x02] = 0x0000_00E6; // STDCAP : CTC | FTSC | AIDC | MINCPS=6
        self.regs_mms0[0x04] = 0x0000_0006; // CONFIG0: CPS=6
        self.regs_mms0[0x08] = 0x0000_0040; // STATUS0: RESETC
        self.regs_mms0[0x0C] = 0x0000_1FBF; // IMASK0 : reset value
    }

    /// Read a standard (MMS 0) register. Other memory maps and undefined addresses read as 0
    /// (spec Section 7.4.3).
    pub fn read_reg(&self, mms: u8, addr: u16) -> u32 {
        if mms != 0 {
            return 0;
        }
        match addr {
            // BUFSTS: TXC (bits 15..8) plenty of credit, RCA (bits 7..0) = queued rx frames
            0x000B => (0x10 << 8) | (self.receive_buff.len().min(0xFF) as u32),
            a if (a as usize) < MMS0_REGS => self.regs_mms0[a as usize],
            _ => 0,
        }
    }

    /// Write a standard (MMS 0) register, honouring read-only / write-1-to-clear / sticky
    /// semantics. Writes elsewhere are ignored (spec Section 7.4.2/7.4.3).
    fn write_reg(&mut self, mms: u8, addr: u16, val: u32) {
        if mms != 0 {
            return;
        }
        match addr {
            0x0000 | 0x0001 | 0x0002 | 0x000B => {} // read-only
            0x0003 => {
                // RESET.SWRESET: self-clearing, triggers a full reset
                if val & 1 != 0 {
                    self.reset_registers();
                }
            }
            0x0004 => {
                // CONFIG0: SYNC (bit 15) cannot be cleared once set (spec 7.6)
                let sticky_sync = self.regs_mms0[0x04] & (1 << 15);
                self.regs_mms0[0x04] = val | sticky_sync;
            }
            0x0008 => self.regs_mms0[0x08] &= !val, // STATUS0: write 1 to clear
            a if (a as usize) < MMS0_REGS => self.regs_mms0[a as usize] = val,
            _ => {}
        }
    }

    // this is meant to enqueue a frame to be sent to the host, it will be sent in the next block that is read from the device
    pub fn enqueue_receive_frame(&mut self, frame: Vec<u8>) {
        self.receive_buff.push(frame);
    }

    // this is meant to pop a frame that has been sent from the host,
    pub fn pop_transmit_frame(&mut self) -> Option<Vec<u8>> {
        self.transmit_buff.pop()
    }

    pub fn assert_cs(&mut self) -> Result<(), FakeTc6DeviceError> {
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

        let out = self.get_out_byte_from_receive_buff();

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
                    // safe to push: we must have had an sv before this can occur, so someone must have pushed an empty vec
                    self.transmit_buff.last_mut().unwrap().push(byte);
                }
                // sv is set and ev is not set, data section is from swo*4 to end of block
                (true, false, _) => {
                    if i == self.transmit_header.swo as usize * 4 {
                        self.transmit_buff.push(vec![]);
                    }
                    if i >= self.transmit_header.swo as usize * 4 {
                        // safe to push: we must have had an sv before this can occur, so someone must have pushed an empty vec
                        self.transmit_buff.last_mut().unwrap().push(byte);
                    }
                }
                // sv is not set and ev is set, data section is from start of block to ebo
                (false, true, _) => {
                    if i <= self.transmit_header.ebo as usize {
                        // safe to push: we must have had an sv before this can occur, so someone must have pushed an empty vec
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
                        // safe to push: we must have had an sv before this can occur, so someone must have pushed an empty vec
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
                        // safe to push: we must have had an sv before this can occur, so someone must have pushed an empty vec
                        self.transmit_buff.last_mut().unwrap().push(byte);
                    }
                    if i >= self.transmit_header.swo as usize * 4 {
                        // safe to push: we must have had an sv before this can occur, so someone must have pushed an empty vec
                        self.transmit_buff.last_mut().unwrap().push(byte);
                    }
                }
            }
        }
        let out = if i >= self.block_size.saturating_sub(4) {
            // snapshot the footer once, at the first footer byte of the block. norx is only known
            // once the header is fully parsed, so force dv/sv/ev back off here in case the header
            // phase bytes (read before norx was known) had already set them from real data.
            if i == self.block_size.saturating_sub(4) {
                if self.transmit_header.norx {
                    self.receive_footer = ReceiveFooter::new();
                }
                self.miso_footer = self.receive_footer.to_bytes();
            }
            self.miso_footer[i - self.block_size.saturating_sub(4)]
        } else {
            self.get_out_byte_from_receive_buff()
        };

        if i == self.block_size - 1 {
            // header indicates host listened, inc offset so next block contains
            self.mode = ReadingHeader(0)
        } else {
            self.mode = ReadingWritingBlock(i + 1);
        }
        Ok(out)
    }

    /// Command mode: everything after the 4-byte control command header. `i` counts bytes from
    /// the first post-header byte. MISO layout (spec Figures 13/14): 4 bytes echoed control
    /// header, then `LEN+1` 32-bit words of echoed write data (reads emit the register value).
    /// The 4 "RX chunk data (discard)" bytes are emitted earlier, during the header phase.
    fn handle_command(&mut self, byte: u8, i: usize) -> Result<u8, FakeTc6DeviceError> {
        let hdr = ControlHeader::from_slice(&self.mosi_header_raw).expect("4-byte header");
        let n_words = hdr.len as usize + 1;
        // full transaction is 4*(LEN+3) bytes; command mode is that minus the 4 header bytes
        let cmd_len = 4 * (n_words + 2) - 4;

        self.command_mosi.push(byte);

        let out = if i < 4 {
            // echoed control command header
            self.mosi_header_raw[i]
        } else {
            let data_i = i - 4;
            let word = data_i / 4;
            let byte_in_word = data_i % 4;
            let reg_addr = if hdr.aid {
                hdr.addr
            } else {
                hdr.addr.wrapping_add(word as u16)
            };
            if hdr.wnr {
                // echo the write-data byte that arrived one 32-bit word earlier, and apply the
                // write once the whole word has been received
                let echoed = self.command_mosi[data_i];
                if byte_in_word == 3 {
                    let ws = data_i - 3;
                    let value = u32::from_be_bytes([
                        self.command_mosi[ws],
                        self.command_mosi[ws + 1],
                        self.command_mosi[ws + 2],
                        self.command_mosi[ws + 3],
                    ]);
                    self.write_reg(hdr.mms, reg_addr, value);
                }
                echoed
            } else {
                self.read_reg(hdr.mms, reg_addr).to_be_bytes()[byte_in_word]
            }
        };

        if i + 1 >= cmd_len {
            self.command_mosi.clear();
            self.mode = ReadingHeader(0);
        } else {
            self.mode = Command(i + 1);
        }
        Ok(out)
    }

    fn handle_reading_command_header(
        &mut self,
        _byte: u8,
        _i: usize,
    ) -> Result<u8, FakeTc6DeviceError> {
        // Control headers are recognised and parsed in `handle_reading_header` (DNC = 0 ->
        // `Command`), so this mode is never entered.
        unreachable!("ReadingCommandHeader is unused; see handle_reading_header")
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

    /// this function returns the byte that should be sent on the MISO line when the master is reading from the device. (in data mode, not command mode)
    /// if there exists valid data to be written, it will set the dv in the footer.
    /// and it will also set the start word offset to the first byte of the frame that has data (condition, index + offset == 0)
    /// end byte will also be set to the last byte of the frame that has data (condition, index + offset == receive_buff.first().len() - 1)
    /// if a frame has been sent, it will be removed from the receive_buff and the offset will be updated
    /// if the block is done it will update the offset so the next block will start at the correct offset, if not norx
    fn get_out_byte_from_receive_buff(&mut self) -> u8 {
        // norx means the host will ignore this chunk's payload; footer must stay all-zero and
        // no state (word_offset, receive_buff) may change so the same data is resent next chunk.
        // this can only be honored once the header has actually been fully parsed (block phase);
        // during the header-phase bytes, this chunk's own norx bit isn't known yet since it's
        // still being clocked in on MOSI, so those bytes always carry real payload data.
        if self.transmit_header.norx && matches!(self.mode, ReadingWritingBlock(_)) {
            return 0;
        }

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

        let mut found_byte = false;
        let out = if let Some(byte) = self
            .receive_buff
            .first()
            .and_then(|v| v.get((index + self.word_offset * 4) as usize))
        {
            self.receive_footer.dv = true;
            found_byte = true;
            *byte
        } else {
            // pop the finished frame and realign word_offset onto the next queued frame, but only when
            // word_offset is not already negative: a negative word_offset means the start of the next frame
            // has deliberately been delayed to later in this block (e.g. to exercise host-side mid-block
            // frame-start reception in tests), and that pending frame must not be skipped by popping again
            // before its start byte has actually been reached.
            // if we have already had a start of a frame, we should not begin a new frame, as we can only send a single start per block
            if index % 4 == 0
                && self.word_offset > 0
                && !self.receive_footer.sv
                && !self.receive_buff.is_empty()
            {
                self.receive_buff.remove(0);
                // assume we are at index 7, the last byte of the second word, we have already sent all bytes of the frame,
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
                found_byte = true;
                *byte
            } else {
                0
            }
        };

        // if we are at the start of a frame, set the start word offset to the first byte of the frame that have data
        // (only when there actually is data here, e.g. not right after popping an exhausted frame with nothing queued behind it)
        if found_byte && index + self.word_offset * 4 == 0 {
            self.receive_footer.sv = true;
            self.receive_footer.swo = (index as u8) >> 2;
        }

        // if last byte in block update offset so next block will start at the correct offset
        if index == self.block_size as isize - 1 {
            self.word_offset += (index + 1) / 4;
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
    // reading the header, the value represents how many bytes have been read
    ReadingHeader(usize),
    ReadingWritingBlock(usize),
    // echo command, the value represents how many bytes have been sent/read
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
    // bytes) through the device, as a single SPI transfer, and returns the captured MISO bytes.
    fn transfer_block(
        tc6: &mut FakeTc6SpiDevice,
        header: TransmitHeader,
        block: [u8; DEFAULT_BLOCK_SIZE],
    ) -> [u8; 4 + DEFAULT_BLOCK_SIZE] {
        let mut write_buff = [0u8; 4 + DEFAULT_BLOCK_SIZE];
        write_buff[..4].copy_from_slice(&header.to_bytes());
        write_buff[4..].copy_from_slice(&block);
        let mut read_buff = [0u8; 4 + DEFAULT_BLOCK_SIZE];
        tc6.transfer(read_buff.as_mut_slice(), write_buff.as_mut_slice())
            .expect("transfer should succeed");
        read_buff
    }

    // sends one header + block chunk transaction (4 header bytes + DEFAULT_BLOCK_SIZE block
    // bytes) through the device, as a single SPI transfer.
    fn send_block(
        tc6: &mut FakeTc6SpiDevice,
        header: TransmitHeader,
        block: [u8; DEFAULT_BLOCK_SIZE],
    ) {
        transfer_block(tc6, header, block);
    }

    // splits the MISO bytes captured for one header+block transaction into the receive payload
    // and footer, decodes the footer, and returns the sub-slice of the payload it marks as valid
    // receive frame data (empty if dv is not set).
    fn receive_valid_data(read_buff: &[u8]) -> &[u8] {
        let (block, footer) = read_buff.split_at(DEFAULT_BLOCK_SIZE);
        let footer = ReceiveFooter::from_slice(footer).unwrap();
        if !footer.dv {
            return &block[..0];
        }
        let start = if footer.sv {
            footer.swo as usize * 4
        } else {
            0
        };
        let end = if footer.ev {
            footer.ebo as usize + 1
        } else {
            block.len()
        };
        &block[start..end]
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

    // a receive frame that fits entirely within a single block, starting at the very first byte
    #[test]
    fn test_receive_frame_fully_inside_one_block() {
        init();
        let mut tc6 = FakeTc6SpiDevice::new();

        let frame = [1u8, 2, 3, 4, 5, 6, 7, 8];
        tc6.device.receive_buff.push(frame.to_vec());

        let header = make_header(false, false, 0, false, 0);
        let read_buff = transfer_block(&mut tc6, header, [0u8; DEFAULT_BLOCK_SIZE]);

        assert_eq!(receive_valid_data(&read_buff), frame);
    }

    // a receive frame that fills an entire first block and finishes partway into the second,
    // which should also cause the (now fully sent) frame to be popped from the receive queue
    #[test]
    fn test_receive_frame_spanning_two_blocks() {
        init();
        let mut tc6 = FakeTc6SpiDevice::new();

        let frame: Vec<u8> = (1u8..=70).collect();
        tc6.device.receive_buff.push(frame.clone());

        let header = make_header(false, false, 0, false, 0);

        let read_a = transfer_block(&mut tc6, header, [0u8; DEFAULT_BLOCK_SIZE]);
        let read_b = transfer_block(&mut tc6, header, [0u8; DEFAULT_BLOCK_SIZE]);

        let mut received: Vec<u8> = vec![];
        received.extend_from_slice(receive_valid_data(&read_a));
        received.extend_from_slice(receive_valid_data(&read_b));
        assert_eq!(received, frame);

        assert!(tc6.device.receive_buff.is_empty());
    }

    // word_offset can be preset to a negative value to delay a frame's start to the middle of a
    // block, e.g. to test that the host correctly handles a receive frame starting mid-block
    #[test]
    fn test_receive_frame_delayed_start_mid_block() {
        init();
        let mut tc6 = FakeTc6SpiDevice::new();

        let frame = [11u8, 12, 13, 14, 15, 16];
        let start = 32;
        tc6.device.receive_buff.push(frame.to_vec());
        tc6.device.word_offset = -((start / 4) as isize);

        let header = make_header(false, false, 0, false, 0);
        let read_buff = transfer_block(&mut tc6, header, [0u8; DEFAULT_BLOCK_SIZE]);

        assert_eq!(&read_buff[..start], vec![0u8; start]);
        assert_eq!(receive_valid_data(&read_buff), frame);
    }

    // a receive frame that starts at the end of one block, fills an entire block in the middle,
    // and ends at the start of a third block
    #[test]
    fn test_receive_frame_start_full_block_end() {
        init();
        let mut tc6 = FakeTc6SpiDevice::new();

        let frame: Vec<u8> = (1u8..=106).collect();
        tc6.device.receive_buff.push(frame.clone());
        // delay the start of the frame to byte 32 of the first block
        tc6.device.word_offset = -8;

        let header = make_header(false, false, 0, false, 0);

        let read_a = transfer_block(&mut tc6, header, [0u8; DEFAULT_BLOCK_SIZE]);
        let read_b = transfer_block(&mut tc6, header, [0u8; DEFAULT_BLOCK_SIZE]);
        let read_c = transfer_block(&mut tc6, header, [0u8; DEFAULT_BLOCK_SIZE]);

        let mut received: Vec<u8> = vec![];
        received.extend_from_slice(receive_valid_data(&read_a));
        received.extend_from_slice(receive_valid_data(&read_b));
        received.extend_from_slice(receive_valid_data(&read_c));
        assert_eq!(received, frame);

        assert!(tc6.device.receive_buff.is_empty());
    }

    // a receive frame split across two blocks with a norx block inserted between them,
    // simulating the host asking the MAC-PHY to hold and resend the same data later
    #[test]
    fn test_receive_frame_with_norx_block_inserted() {
        init();
        let mut tc6 = FakeTc6SpiDevice::new();

        let frame: Vec<u8> = (1u8..=42).collect();
        tc6.device.receive_buff.push(frame.clone());
        // delay the start of the frame to byte 32 of the first block
        tc6.device.word_offset = -8;

        let header = make_header(false, false, 0, false, 0);
        let norx_header = TransmitHeader {
            norx: true,
            ..header
        };

        let read_a = transfer_block(&mut tc6, header, [0u8; DEFAULT_BLOCK_SIZE]);

        // host is not listening this chunk: footer must be all-zero and the frame retained
        let read_norx = transfer_block(&mut tc6, norx_header, [0xAAu8; DEFAULT_BLOCK_SIZE]);
        assert!(receive_valid_data(&read_norx).is_empty());
        assert_eq!(tc6.device.receive_buff, vec![frame.clone()]);

        let read_b = transfer_block(&mut tc6, header, [0u8; DEFAULT_BLOCK_SIZE]);

        let mut received: Vec<u8> = vec![];
        received.extend_from_slice(receive_valid_data(&read_a));
        received.extend_from_slice(receive_valid_data(&read_b));
        assert_eq!(received, frame);

        assert!(tc6.device.receive_buff.is_empty());
    }
}
