//! Tests for the split TC6 driver.
//!
//! Two kinds of test double are used:
//!
//! * [`SpyInterface`] - a hand-rolled [`Tc6Interface`] that records every chunk the
//!   driver pushes over "SPI" and lets the test play the MAC-PHY side: write a footer
//!   (+ optional receive payload) back into the MISO buffer of the last chunk and raise
//!   "transfer complete". This is the "custom tc6 interface to track what's sent/read".
//! * [`fake_tc6`]'s `FakeTc6SpiDevice` behind the real [`EmbeddedHalImplementation`]
//!   SPI wrapper, for an end-to-end round trip through the actual wire-format code.
//!
//! Both halves returned by [`split`] are deconstructed directly (their producer/consumer
//! and internal state-machine fields are reachable from this child module) so a test can
//! drive one side and observe the other.
//!
//! Pool plumbing uses `heapless`'s box pool: <https://docs.rs/heapless/latest/heapless/pool/boxed/index.html>

use super::*;
use heapless::pool::boxed::BoxBlock;
use smoltcp::phy::{Device, Medium, RxToken, TxToken};
use smoltcp::time::Instant;
use std::collections::BTreeMap;
use std::vec::Vec;

const M: usize = 1600; // FrameBuffer capacity (bytes); >= MIN_FRAME_BUFFER_CAPACITY
const NQ: usize = 4; // spsc queue const param; usable capacity is NQ - 1 == 3

// --------------------------------------------------------------------------------------
// test doubles
// --------------------------------------------------------------------------------------

struct NoopSignal;
impl TransmitSignal for NoopSignal {
    fn signal(&mut self) {}
}

/// Counts how many times the low-prio side asked to wake the high-prio side.
#[derive(Default)]
struct CountingSignal {
    count: usize,
}
impl TransmitSignal for CountingSignal {
    fn signal(&mut self) {
        self.count += 1;
    }
}

/// One chunk the driver handed to the interface.
struct Chunk {
    header: u32,
    /// snapshot of the 64 MOSI payload bytes
    data: [u8; 64],
    /// true if it came in via `begin_send_joined_block`
    joined: bool,
    /// where the driver wants the MISO bytes (64 data + 4 footer) written back
    result_ptr: *mut u8,
    result_len: usize,
}

/// One control transaction the driver handed to the interface.
struct ControlTxn {
    /// full MOSI snapshot (header + write data + trailing don't-care word)
    tx: Vec<u8>,
    /// where the driver wants the MISO stream written back
    result_ptr: *mut u8,
    result_len: usize,
}

struct SpyInterface {
    chunks: Vec<Chunk>,
    control_txns: Vec<ControlTxn>,
    complete: bool,
    /// If set, answers every control transaction as soon as it starts (MOSI in, MISO out)
    /// and raises completion, so the driver's blocking register functions can be driven.
    control_reply: Option<fn(&[u8], &mut [u8])>,
    /// footers handed back by `respond` carry SYNC = 1 (a configured chip) unless cleared
    footer_sync: bool,
}

impl Default for SpyInterface {
    fn default() -> Self {
        Self {
            chunks: Vec::new(),
            control_txns: Vec::new(),
            complete: false,
            control_reply: None,
            footer_sync: true,
        }
    }
}

impl SpyInterface {
    fn last(&self) -> &Chunk {
        self.chunks.last().expect("no chunk sent yet")
    }

    fn last_control(&self) -> &ControlTxn {
        self.control_txns.last().expect("no control txn sent yet")
    }

    /// Play the MAC-PHY side of a control *read*: echo the command header the driver sent
    /// into MISO bytes `[4..8)` and place `values` after it, then raise "transfer complete".
    fn respond_control_read(&mut self, values: &[u32]) {
        let t = self.control_txns.last().expect("no control txn sent yet");
        let (ptr, len) = (t.result_ptr, t.result_len);
        let mut echoed_header = [0u8; 4];
        echoed_header.copy_from_slice(&t.tx[0..4]);
        assert!(len >= 8 + 4 * values.len());
        // SAFETY: same rationale as `respond` - the driver is blocked on completion so the
        // buffer it handed us is still alive and un-aliased.
        let buf = unsafe { core::slice::from_raw_parts_mut(ptr, len) };
        buf.fill(0);
        buf[4..8].copy_from_slice(&echoed_header);
        for (i, v) in values.iter().enumerate() {
            buf[8 + 4 * i..12 + 4 * i].copy_from_slice(&v.to_be_bytes());
        }
        self.complete = true;
    }

    /// Play the MAC-PHY side of a control *write*: echo the command header and echo the
    /// write-data words straight back (spec Section 7.4.4), then raise "transfer complete".
    fn respond_control_write(&mut self) {
        let t = self.control_txns.last().expect("no control txn sent yet");
        let (ptr, len) = (t.result_ptr, t.result_len);
        // MOSI = [4B header][4*n write data][4B don't-care]; echo header + the n data words.
        let n = len / 4 - 2;
        let mut echo = Vec::new();
        echo.extend_from_slice(&t.tx[0..4 + 4 * n]);
        let buf = unsafe { core::slice::from_raw_parts_mut(ptr, len) };
        buf.fill(0);
        buf[4..4 + echo.len()].copy_from_slice(&echo);
        self.complete = true;
    }

    /// Play a header parity error: the MAC-PHY streams the `0xC0000001` error code (spec
    /// Section 7.5.1) for every word from the second onward, then raises completion.
    fn respond_control_bad_header(&mut self) {
        let t = self.control_txns.last().expect("no control txn sent yet");
        let (ptr, len) = (t.result_ptr, t.result_len);
        let buf = unsafe { core::slice::from_raw_parts_mut(ptr, len) };
        for word in buf.chunks_exact_mut(4).skip(1) {
            word.copy_from_slice(&0xC000_0001u32.to_be_bytes());
        }
        self.complete = true;
    }

    /// Pretend the MAC-PHY filled in the MISO buffer of the most recent chunk:
    /// `payload` at the front (received frame bytes), `footer` in the trailing 4 bytes,
    /// and mark the transfer complete.
    fn respond(&mut self, payload: &[u8], footer: crate::tc6_footer::ReceiveFooter) {
        assert!(payload.len() <= 64);
        let footer = if self.footer_sync {
            footer.with_sync(true).with_correct_parity()
        } else {
            footer
        };
        let c = self.chunks.last().expect("no chunk sent yet");
        assert_eq!(c.result_len, 68, "MISO buffer is 64B data + 4B footer");
        // SAFETY: `result_ptr` came from `WriteBuffer::write_buffer` of the buffer the
        // driver handed us for this chunk (the tail of the in-flight receive FrameBuffer,
        // or the scratch buffer). That buffer is still alive and, since the driver is
        // blocked waiting for completion, un-aliased - exactly the situation a real DMA
        // completion writes into.
        let buf = unsafe { core::slice::from_raw_parts_mut(c.result_ptr, c.result_len) };
        buf[..payload.len()].copy_from_slice(payload);
        buf[64..68].copy_from_slice(&footer.to_bytes());
        self.complete = true;
    }
}

impl Tc6Interface for SpyInterface {
    type Error = core::convert::Infallible;

    fn begin_send_block(
        &mut self,
        header: u32,
        data: impl ReadBuffer<Word = u8>,
        mut result: impl WriteBuffer<Word = u8>,
    ) -> Result<(), Self::Error> {
        let (d_ptr, d_len) = unsafe { data.read_buffer() };
        assert_eq!(d_len, 64, "a data chunk payload is one 64-byte block");
        let mut snap = [0u8; 64];
        snap.copy_from_slice(unsafe { core::slice::from_raw_parts(d_ptr, 64) });
        let (r_ptr, r_len) = unsafe { result.write_buffer() };
        self.chunks.push(Chunk {
            header,
            data: snap,
            joined: false,
            result_ptr: r_ptr,
            result_len: r_len,
        });
        Ok(())
    }

    fn transfer_complete(&mut self) -> Result<bool, Self::Error> {
        // edge triggered: reading the "done" flag clears it, like a real status bit.
        // This also means `on_interrupt`'s internal loop terminates.
        Ok(core::mem::take(&mut self.complete))
    }

    fn begin_send_joined_block(
        &mut self,
        data: impl ReadBuffer<Word = u8>,
        mut result: impl WriteBuffer<Word = u8>,
    ) -> Result<(), Self::Error> {
        let (d_ptr, d_len) = unsafe { data.read_buffer() };
        assert!(d_len >= 68);
        let hdr: [u8; 4] = unsafe { core::slice::from_raw_parts(d_ptr, 4) }
            .try_into()
            .unwrap();
        let mut snap = [0u8; 64];
        snap.copy_from_slice(unsafe { core::slice::from_raw_parts(d_ptr.add(4), 64) });
        let (r_ptr, r_len) = unsafe { result.write_buffer() };
        self.chunks.push(Chunk {
            header: u32::from_be_bytes(hdr),
            data: snap,
            joined: true,
            result_ptr: r_ptr,
            result_len: r_len,
        });
        Ok(())
    }

    fn begin_control_transaction(
        &mut self,
        tx: impl ReadBuffer<Word = u8>,
        mut rx: impl WriteBuffer<Word = u8>,
    ) -> Result<(), Self::Error> {
        let (d_ptr, d_len) = unsafe { tx.read_buffer() };
        let (r_ptr, r_len) = unsafe { rx.write_buffer() };
        assert_eq!(d_len, r_len, "control tx/rx buffers must match length");
        assert!(
            d_len >= 12 && d_len % 4 == 0,
            "at least header + 1 word + pad"
        );
        let tx = unsafe { core::slice::from_raw_parts(d_ptr, d_len) }.to_vec();
        if let Some(reply) = self.control_reply {
            // SAFETY: same rationale as `respond` - the driver's MISO buffer is alive and
            // un-aliased until it sees completion.
            reply(&tx, unsafe {
                core::slice::from_raw_parts_mut(r_ptr, r_len)
            });
            self.complete = true;
        }
        self.control_txns.push(ControlTxn {
            tx,
            result_ptr: r_ptr,
            result_len: r_len,
        });
        Ok(())
    }
}

/// Minimal `ReadBuffer`/`WriteBuffer` over a raw slice, for calling interface methods
/// directly without going through the driver's private `DMAPtr`.
struct RawBuf {
    ptr: *mut u8,
    len: usize,
}
impl RawBuf {
    fn new(s: &mut [u8]) -> Self {
        Self {
            ptr: s.as_mut_ptr(),
            len: s.len(),
        }
    }
}
unsafe impl ReadBuffer for RawBuf {
    type Word = u8;
    unsafe fn read_buffer(&self) -> (*const u8, usize) {
        (self.ptr, self.len)
    }
}
unsafe impl WriteBuffer for RawBuf {
    type Word = u8;
    unsafe fn write_buffer(&mut self) -> (*mut u8, usize) {
        (self.ptr, self.len)
    }
}

/// `fake_tc6` behind the real SPI wrapper, but with a completion flag that reads `false`
/// once before it reads `true` - i.e. a transfer looks "in flight" for exactly one poll.
/// That is what lets `on_interrupt`'s loop take its `transfer_complete() == false` exit
/// (the plain blocking `EmbeddedHalImplementation` never would). A real DMA interface
/// behaves like this for real reasons.
#[cfg(feature = "embedded-hal")]
struct PollingFake {
    inner: crate::embedded_io_implementation::EmbeddedHalImplementation<
        fake_tc6::fake_tc6_device::FakeTc6SpiDevice,
    >,
    polls_until_done: u32,
}
#[cfg(feature = "embedded-hal")]
impl PollingFake {
    fn new(spi: fake_tc6::fake_tc6_device::FakeTc6SpiDevice) -> Self {
        Self {
            inner: crate::embedded_io_implementation::EmbeddedHalImplementation::new(spi),
            polls_until_done: 0,
        }
    }
}
#[cfg(feature = "embedded-hal")]
impl Tc6Interface for PollingFake {
    type Error = crate::embedded_io_implementation::EmbeddedHalImplementationError<
        fake_tc6::fake_tc6_device::FakeTc6SpiDeviceError,
    >;

    fn begin_send_block(
        &mut self,
        header: u32,
        data: impl ReadBuffer<Word = u8>,
        result: impl WriteBuffer<Word = u8>,
    ) -> Result<(), Self::Error> {
        self.inner.begin_send_block(header, data, result)?;
        self.polls_until_done = 2;
        Ok(())
    }

    fn transfer_complete(&mut self) -> Result<bool, Self::Error> {
        self.polls_until_done = self.polls_until_done.saturating_sub(1);
        Ok(self.polls_until_done == 0)
    }

    fn begin_send_joined_block(
        &mut self,
        data: impl ReadBuffer<Word = u8>,
        result: impl WriteBuffer<Word = u8>,
    ) -> Result<(), Self::Error> {
        self.inner.begin_send_joined_block(data, result)?;
        self.polls_until_done = 2;
        Ok(())
    }

    fn begin_control_transaction(
        &mut self,
        tx: impl ReadBuffer<Word = u8>,
        rx: impl WriteBuffer<Word = u8>,
    ) -> Result<(), Self::Error> {
        self.inner.begin_control_transaction(tx, rx)?;
        self.polls_until_done = 2;
        Ok(())
    }
}

/// A tiny MMS-0 register file that answers control transactions inline (synchronously),
/// which is what the blocking `configure` path needs. Models just enough of the standard
/// registers for bring-up: RO IDVER/PHYID/STDCAP, sticky `CONFIG0.SYNC`, W1C `STATUS0`.
struct RegDouble {
    regs: BTreeMap<(u8, u16), u32>,
    writes: Vec<(u8, u16, u32)>,
    /// drop the `SYNC` bit on every `CONFIG0` write (to exercise `ConfigError::SyncFailed`)
    ignore_sync_write: bool,
    /// never raise completion (to exercise `ConfigError::Timeout`)
    never_complete: bool,
    complete: bool,
}

impl RegDouble {
    fn new() -> Self {
        let mut regs = BTreeMap::new();
        regs.insert((0, 0x0000), 0x0000_0011); // IDVER: v1.1
        regs.insert((0, 0x0002), 0x0000_00C6); // STDCAP: CTC | FTSC | MINCPS=6
        regs.insert((0, 0x0004), 0x0000_0006); // CONFIG0: CPS=6, SYNC=0
        regs.insert((0, 0x0008), 0x0000_0040); // STATUS0: RESETC set
        regs.insert((0, 0x000C), 0x0000_1FBF); // IMASK0: reset value
        Self {
            regs,
            writes: Vec::new(),
            ignore_sync_write: false,
            never_complete: false,
            complete: false,
        }
    }

    fn reg(&self, mms: u8, addr: u16) -> u32 {
        self.regs.get(&(mms, addr)).copied().unwrap_or(0)
    }

    fn apply_write(&mut self, mms: u8, addr: u16, val: u32) {
        self.writes.push((mms, addr, val));
        match (mms, addr) {
            // read-only
            (0, 0x0000) | (0, 0x0001) | (0, 0x0002) => {}
            // STATUS0: write 1 to clear
            (0, 0x0008) => {
                let cur = self.reg(0, 0x0008);
                self.regs.insert((0, 0x0008), cur & !val);
            }
            // CONFIG0: SYNC is sticky once set (spec 7.6)
            (0, 0x0004) => {
                let cur = self.reg(0, 0x0004);
                let mut new_value = val;
                if registers::Config0::from_raw(cur).sync() {
                    new_value |= 1 << 15;
                }
                if self.ignore_sync_write {
                    new_value &= !(1 << 15);
                }
                self.regs.insert((0, 0x0004), new_value);
            }
            _ => {
                self.regs.insert((mms, addr), val);
            }
        }
    }
}

impl Default for RegDouble {
    fn default() -> Self {
        Self::new()
    }
}

impl Tc6Interface for RegDouble {
    type Error = core::convert::Infallible;

    fn begin_send_block(
        &mut self,
        _h: u32,
        _d: impl ReadBuffer<Word = u8>,
        _r: impl WriteBuffer<Word = u8>,
    ) -> Result<(), Self::Error> {
        unreachable!("RegDouble is only for control/bring-up tests")
    }

    fn transfer_complete(&mut self) -> Result<bool, Self::Error> {
        Ok(core::mem::take(&mut self.complete))
    }

    fn begin_control_transaction(
        &mut self,
        tx: impl ReadBuffer<Word = u8>,
        mut rx: impl WriteBuffer<Word = u8>,
    ) -> Result<(), Self::Error> {
        let (d_ptr, d_len) = unsafe { tx.read_buffer() };
        let (r_ptr, r_len) = unsafe { rx.write_buffer() };
        assert_eq!(d_len, r_len);
        let tx_buf = unsafe { core::slice::from_raw_parts(d_ptr, d_len) };
        let rx_buf = unsafe { core::slice::from_raw_parts_mut(r_ptr, r_len) };

        let hdr = tc6_header::ControlHeader::from_bytes(tx_buf[0..4].try_into().unwrap());
        let n = hdr.len() as usize + 1;

        rx_buf.fill(0);
        rx_buf[4..8].copy_from_slice(&tx_buf[0..4]); // echo the command header
        for i in 0..n {
            let addr = hdr.addr() + i as u16;
            let out = if hdr.wnr() {
                let val = u32::from_be_bytes(tx_buf[4 + 4 * i..8 + 4 * i].try_into().unwrap());
                self.apply_write(hdr.mms(), addr, val);
                val // echo write data (spec 7.4.4)
            } else {
                self.reg(hdr.mms(), addr)
            };
            rx_buf[8 + 4 * i..12 + 4 * i].copy_from_slice(&out.to_be_bytes());
        }

        if !self.never_complete {
            self.complete = true;
        }
        Ok(())
    }
}

// --------------------------------------------------------------------------------------
// environment setup
// --------------------------------------------------------------------------------------

/// Builds a fresh, isolated driver pair. Every invocation declares its own `box_pool!`
/// singletons (they are function-local types) and leaks its queues, so tests running in
/// parallel never share pool state.
macro_rules! setup {
    ($hi:ident, $lo:ident, $i_face:expr, $signal:expr) => {
        heapless::box_pool!(RxPool: FrameBuffer<M>);
        heapless::box_pool!(TxPool: FrameBuffer<M>);
        for _ in 0..8usize {
            RxPool.manage(::std::boxed::Box::leak(::std::boxed::Box::new(BoxBlock::new())));
            TxPool.manage(::std::boxed::Box::leak(::std::boxed::Box::new(BoxBlock::new())));
        }
        let rx_q: &'static mut Queue<Box<RxPool>, NQ> =
            ::std::boxed::Box::leak(::std::boxed::Box::new(Queue::new()));
        let tx_q: &'static mut Queue<Box<TxPool>, NQ> =
            ::std::boxed::Box::leak(::std::boxed::Box::new(Queue::new()));
        #[allow(unused_mut)]
        let (mut $hi, mut $lo) = split(($i_face), rx_q, tx_q, ($signal));
    };
}

/// Like [`setup!`] but goes through [`split_configured`], unwrapping the bring-up result.
macro_rules! setup_configured {
    ($hi:ident, $lo:ident, $i_face:expr, $signal:expr, $cfg:expr) => {
        heapless::box_pool!(RxPool: FrameBuffer<M>);
        heapless::box_pool!(TxPool: FrameBuffer<M>);
        for _ in 0..8usize {
            RxPool.manage(::std::boxed::Box::leak(::std::boxed::Box::new(BoxBlock::new())));
            TxPool.manage(::std::boxed::Box::leak(::std::boxed::Box::new(BoxBlock::new())));
        }
        let rx_q: &'static mut Queue<Box<RxPool>, NQ> =
            ::std::boxed::Box::leak(::std::boxed::Box::new(Queue::new()));
        let tx_q: &'static mut Queue<Box<TxPool>, NQ> =
            ::std::boxed::Box::leak(::std::boxed::Box::new(Queue::new()));
        #[allow(unused_mut)]
        let (mut $hi, mut $lo) = split_configured(($i_face), rx_q, tx_q, ($signal), $cfg)
            .expect("bring-up should succeed");
    };
}

fn ethernet_like(len: usize) -> Vec<u8> {
    (0..len)
        .map(|i| (i as u8).wrapping_mul(31).wrapping_add(7))
        .collect()
}

mod bring_up;
mod control;
mod data_path;
mod end_to_end;
mod low_prio;
