use crate::tc6_footer::ReceiveFooter;
use crate::tc6_interface::Tc6Interface;
use crate::{frame_buffer::FrameBuffer, registers, tc6_footer, tc6_header};
use embedded_dma::{ReadBuffer, WriteBuffer};
use heapless::pool::boxed::{Box, BoxPool};
use heapless::spsc::{Consumer, Producer, Queue};
use smoltcp::phy::DeviceCapabilities;

// NOTE: `box_pool!` singletons must name a concrete type, so `RxP`/`TxP` (and the `M` they were
// built with) have to be declared by the application, not by this generic driver. Pass the
// resulting pool types + `'static` queue storage into `split()` below.

// TODO, is this a good idea? this was a hack to avoid the lifetime issues with passing &mut [u8] to the tc6 interface
struct DMAPtr {
    ptr: *mut u8,
    len: usize,
}

impl DMAPtr {
    fn from_slice(slice: &mut [u8]) -> Self {
        Self {
            ptr: slice.as_mut_ptr(),
            len: slice.len(),
        }
    }
}

unsafe impl WriteBuffer for DMAPtr {
    type Word = u8;

    unsafe fn write_buffer(&mut self) -> (*mut Self::Word, usize) {
        (self.ptr, self.len)
    }
}

unsafe impl ReadBuffer for DMAPtr {
    type Word = u8;

    unsafe fn read_buffer(&self) -> (*const Self::Word, usize) {
        (self.ptr, self.len)
    }
}

enum TC6DriverState {
    Idle,
    WaitingForTransmitComplete,
    /// A control-mode register read/write (TC6 spec Section 7.4) is in flight over SPI.
    WaitingForControlComplete,
}

/// Largest number of consecutive registers one control transaction issued by this driver may
/// touch. The TC6 `LEN` field allows up to 128; this is just the size of the driver's static
/// scratch buffers (`4 * (CONTROL_MAX_REGS + 2)` bytes each for MOSI and MISO).
pub const CONTROL_MAX_REGS: usize = 8;

/// Byte length of a control transaction touching `CONTROL_MAX_REGS` registers:
/// 4-byte header + `n` data words + 4-byte trailing don't-care word (TC6 spec Figures 13/14).
const CONTROL_BUF_LEN: usize = 4 * (CONTROL_MAX_REGS + 2);

/// Error from the register read/write entry points on [`TC6HighPrio`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControlError<E> {
    /// A control transaction is already pending or in flight; retry once it has completed
    /// (poll [`TC6HighPrio::take_control_result`]).
    Busy,
    /// The requested register count is zero or exceeds [`CONTROL_MAX_REGS`].
    UnsupportedCount,
    /// The underlying interface failed to start the transfer.
    Interface(E),
}

/// A register read/write request accepted but not yet put on the wire.
struct PendingControl {
    /// `true` = write (WNR = 1), `false` = read.
    write: bool,
    /// Memory Map Selector (TC6 spec Table 6).
    mms: u8,
    /// Address of the first register within the selected memory map.
    addr: u16,
    /// Number of consecutive registers, `1..=CONTROL_MAX_REGS`.
    count: u8,
    /// Write data, `values[..count]` used for writes; ignored for reads.
    values: [u32; CONTROL_MAX_REGS],
}

/// Bookkeeping for the control transaction currently on the wire.
struct ControlInFlight {
    write: bool,
    count: u8,
}

/// Result of a completed control transaction, taken via [`TC6HighPrio::take_control_result`].
pub struct ControlResult {
    /// The control command header echoed back by the MAC-PHY (MISO bytes `[4..8)`).
    pub echoed_header: tc6_header::ControlHeader,
    /// `true` if the MAC-PHY flagged the command header as bad (HDRB set, or the echoed header
    /// failed its parity check - e.g. the `0xC0000001` error code from spec Section 7.5.1). No
    /// register access was performed; the host should retry the transaction.
    pub header_bad: bool,
    /// `true` if this was a write.
    pub write: bool,
    values: [u32; CONTROL_MAX_REGS],
    count: u8,
}

impl ControlResult {
    /// The register values.
    ///
    /// For a read these are the values read back. For a write these are the *echoed* write data
    /// words the MAC-PHY returned, which the host can compare against what it sent to spot bus
    /// errors (TC6 spec Section 7.4.4). Empty when [`header_bad`](Self::header_bad) is set.
    pub fn values(&self) -> &[u32] {
        if self.header_bad {
            &[]
        } else {
            &self.values[..self.count as usize]
        }
    }
}

/// `CONFIG0.TXCTHRESH` - minimum free transmit credits before the chip asserts IRQn.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum TxCreditThreshold {
    /// >= 1 credit (reset default).
    #[default]
    One,
    /// >= 4 credits.
    Four,
    /// >= 8 credits.
    Eight,
    /// >= 16 credits.
    Sixteen,
}

impl TxCreditThreshold {
    const fn bits(self) -> u8 {
        match self {
            TxCreditThreshold::One => 0b00,
            TxCreditThreshold::Four => 0b01,
            TxCreditThreshold::Eight => 0b10,
            TxCreditThreshold::Sixteen => 0b11,
        }
    }
}

/// Frame timestamp width, selected by `CONFIG0.FTSS` when timestamps are enabled.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimestampSize {
    Bits32,
    Bits64,
}

/// PLCA (Physical Layer Collision Avoidance) node setup, written to the PHY's MMS 4 registers
/// during [`TC6HighPrio::configure`].
///
/// The MAC-PHY runs the PLCA state machine, but every node must be told its own ID - there is no
/// auto-assignment. Build one with [`PlcaConfig::follower`] or [`PlcaConfig::coordinator`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PlcaConfig {
    /// This node's PLCA ID (`0..=254`). Node `0` is the coordinator and sends the BEACON.
    pub node_id: u8,
    /// Total node count. Only used by the coordinator (`node_id == 0`); ignored otherwise.
    pub node_count: u8,
}

impl PlcaConfig {
    /// A follower node with the given ID. Node count is only needed on the coordinator.
    pub const fn follower(node_id: u8) -> Self {
        Self {
            node_id,
            node_count: 0,
        }
    }

    /// The coordinator (node 0), which sends the BEACON, for a network of `node_count` nodes.
    pub const fn coordinator(node_count: u8) -> Self {
        Self {
            node_id: 0,
            node_count,
        }
    }
}

/// Bring-up configuration applied by [`TC6HighPrio::configure`] / [`split_configured`].
///
/// [`Config::default`] is the minimal safe setup: unprotected control, >= 1 transmit credit
/// before IRQn, no cut-through, no timestamps, PLCA left untouched, every maskable interrupt
/// masked. The chunk payload size is always forced to 64 bytes - the data path assumes it.
//
// TODO: `CONFIG0.PROTE` (protected control transfers - each 32-bit word sent with its ones'
// complement, spec 7.4.4) is not offered here yet: the control path in `start_control_transaction`
// / `handle_control_complete` only does unprotected single-word transfers.
#[derive(Debug, Clone, Copy)]
pub struct Config {
    /// `CONFIG0.TXCTHRESH`.
    pub tx_credit_threshold: TxCreditThreshold,
    /// `CONFIG0.TXCTE` - cut-through from the SPI host to the network. Requires `STDCAP.CTC`.
    pub cut_through_tx: bool,
    /// `CONFIG0.RXCTE` - cut-through from the network to the SPI host. Requires `STDCAP.CTC`.
    pub cut_through_rx: bool,
    /// `Some` enables `CONFIG0.FTSE` and picks the width via `FTSS`. Requires `STDCAP.FTSC`.
    pub timestamp: Option<TimestampSize>,
    /// `Some` sets this node's PLCA ID / node count and enables PLCA; `None` leaves the PHY's
    /// PLCA registers untouched (strap defaults / plain CSMA/CD).
    pub plca: Option<PlcaConfig>,
    /// Value written to `IMASK0`. Defaults to [`registers::IMask0::RESET_VALUE`].
    pub interrupt_mask: registers::IMask0,
    /// Max busy-wait iterations for a single control transfer to report complete before
    /// [`configure`](TC6HighPrio::configure) gives up with [`ConfigError::Timeout`].
    pub transfer_poll_limit: u32,
    /// Max `STATUS0` re-reads waiting for `RESETC` before [`ConfigError::ResetNotComplete`].
    pub reset_poll_limit: u32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            tx_credit_threshold: TxCreditThreshold::One,
            cut_through_tx: false,
            cut_through_rx: false,
            timestamp: None,
            plca: None,
            interrupt_mask: registers::IMask0::RESET_VALUE,
            transfer_poll_limit: 100_000,
            reset_poll_limit: 64,
        }
    }
}

/// Error from [`TC6HighPrio::configure`] / [`split_configured`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConfigError<E> {
    /// A control transaction was already pending/in flight when `configure` was called.
    Busy,
    /// A control transfer never reported complete within `Config::transfer_poll_limit`.
    Timeout,
    /// `STATUS0.RESETC` never became set within `Config::reset_poll_limit` reads.
    ResetNotComplete,
    /// `IDVER` read back implausible - the bus is not working or this is not a TC6 device.
    BadVersion { major: u8, minor: u8 },
    /// A requested feature is not advertised in `STDCAP` (the `&str` names it).
    Unsupported(&'static str),
    /// `CONFIG0` read-back did not show `SYNC` set with a 64-byte chunk payload.
    SyncFailed,
    /// The MAC-PHY rejected a control command header (parity error / HDRB).
    HeaderRejected,
    /// The underlying interface failed to run a transfer.
    Interface(E),
}

/// Implemented by the application to wake the high-priority side when the low-priority side has
/// queued a frame to transmit (e.g. pend a software interrupt, or spawn an RTIC task that ends up
/// calling `TC6HighPrio::on_transmit_ready`). Without this, a pure-transmit workload would never
/// get serviced, since `on_interrupt` only fires on chip-initiated events.
pub trait TransmitSignal {
    fn signal(&mut self);
}

/// Owned by the high-priority context (e.g. the TC6 IRQ task): services the chip over SPI.
pub struct TC6HighPrio<T, RxP, TxP, const M: usize, const NQ: usize>
where
    T: Tc6Interface,
    RxP: BoxPool<Data = FrameBuffer<M>> + 'static,
    TxP: BoxPool<Data = FrameBuffer<M>> + 'static,
{
    interface: T,
    current_transmit_buffer: Option<Box<TxP>>, // the frame currently being transmitted, if any
    current_receive_buffer: Option<Box<RxP>>,  // the frame currently being received, if any
    // todo, do we need Option, can we use empty vec to indicate no data?
    extra_start_buffer: Option<heapless::Vec<u8, 64>>, // used to store the start of a receive frame if the start was written to the end of the previous framebuffer
    driver_state: TC6DriverState,
    current_transmit_buffer_cursor: usize, // keeps track of how much have been written to the tc6 interface, so we know what to transfer in the next block
    scratch_header: [u8; 4], // used for temporary storage of the header in the case where a block needs to be moved
    scratch_buffer: [u8; 68], // used for temporary storage of a block in the case where a block needs to be moved
    rx_ready_producer: Producer<'static, Box<RxP>>, // frames pulled off the wire, handed to the low-prio side
    tx_ready_consumer: Consumer<'static, Box<TxP>>, // frames built by the low-prio side, waiting to go out over SPI
    // --- command mode (register read/write, TC6 spec Section 7.4) ---
    pending_control: Option<PendingControl>, // accepted request, not yet on the wire
    control_in_flight: Option<ControlInFlight>, // request currently being transferred over SPI
    last_control_result: Option<ControlResult>, // completed transaction, waiting to be taken
    control_tx_buffer: [u8; CONTROL_BUF_LEN],   // MOSI: header + write data + trailing don't-care
    control_rx_buffer: [u8; CONTROL_BUF_LEN],   // MISO: don't-care + echoed header + data
    // Set by `configure`; cleared if a receive footer ever reports SYNC = 0 (the chip reset
    // itself and lost its configuration, spec Section 7.7 - the host must re-run `configure`).
    sync_established: bool,
}

/// Owned by the low-priority context (e.g. `idle`): implements `smoltcp::phy::Device`.
pub struct TC6LowPrio<RxP, TxP, S, const M: usize, const NQ: usize>
where
    RxP: BoxPool<Data = FrameBuffer<M>> + 'static,
    TxP: BoxPool<Data = FrameBuffer<M>> + 'static,
    S: TransmitSignal,
{
    rx_ready_consumer: Consumer<'static, Box<RxP>>,
    tx_ready_producer: Producer<'static, Box<TxP>>,
    transmit_signal: S, // poked whenever a frame is pushed onto `tx_ready_producer`
}

/// Splits the driver into its high-priority (IRQ) and low-priority (smoltcp) halves.
///
/// `rx_ready`/`tx_ready` need `'static` storage from the caller (e.g. `static_cell` or a
/// `static mut` in `#[init]`). `RxP`/`TxP` must already have their memory blocks `manage`d.
///
/// The returned driver is **unconfigured**: [`TC6HighPrio::configure`] must succeed before any
/// frame data will move (the data path is inert while `SYNC = 0`). Prefer [`split_configured`]
/// unless you specifically need to defer bring-up (e.g. the SPI transport is not ready yet).
#[must_use = "the driver does nothing until TC6HighPrio::configure succeeds"]
pub fn split<T, RxP, TxP, S, const M: usize, const NQ: usize>(
    interface: T,
    rx_ready: &'static mut Queue<Box<RxP>, NQ>,
    tx_ready: &'static mut Queue<Box<TxP>, NQ>,
    transmit_signal: S,
) -> (
    TC6HighPrio<T, RxP, TxP, M, NQ>,
    TC6LowPrio<RxP, TxP, S, M, NQ>,
)
where
    T: Tc6Interface,
    RxP: BoxPool<Data = FrameBuffer<M>> + 'static,
    TxP: BoxPool<Data = FrameBuffer<M>> + 'static,
    S: TransmitSignal,
{
    let (rx_ready_producer, rx_ready_consumer) = rx_ready.split();
    let (tx_ready_producer, tx_ready_consumer) = tx_ready.split();

    (
        TC6HighPrio {
            interface,
            driver_state: TC6DriverState::Idle,
            current_transmit_buffer_cursor: 0,
            scratch_header: [0; 4],
            scratch_buffer: [0; 68],
            current_transmit_buffer: None,
            current_receive_buffer: None,
            rx_ready_producer,
            tx_ready_consumer,
            extra_start_buffer: None,
            pending_control: None,
            control_in_flight: None,
            last_control_result: None,
            control_tx_buffer: [0; CONTROL_BUF_LEN],
            control_rx_buffer: [0; CONTROL_BUF_LEN],
            sync_established: false,
        },
        TC6LowPrio {
            rx_ready_consumer,
            tx_ready_producer,
            transmit_signal,
        },
    )
}

/// Like [`split`], then run [`TC6HighPrio::configure`] on the high-priority half before handing
/// both back, so the caller can never obtain an unconfigured driver. On error the driver halves
/// are dropped; there is nothing useful to do with them.
#[allow(clippy::type_complexity)]
pub fn split_configured<T, RxP, TxP, S, const M: usize, const NQ: usize>(
    interface: T,
    rx_ready: &'static mut Queue<Box<RxP>, NQ>,
    tx_ready: &'static mut Queue<Box<TxP>, NQ>,
    transmit_signal: S,
    config: &Config,
) -> Result<
    (
        TC6HighPrio<T, RxP, TxP, M, NQ>,
        TC6LowPrio<RxP, TxP, S, M, NQ>,
    ),
    ConfigError<T::Error>,
>
where
    T: Tc6Interface,
    RxP: BoxPool<Data = FrameBuffer<M>> + 'static,
    TxP: BoxPool<Data = FrameBuffer<M>> + 'static,
    S: TransmitSignal,
{
    let (mut hi, lo) = split(interface, rx_ready, tx_ready, transmit_signal);
    hi.configure(config)?;
    Ok((hi, lo))
}

impl<T, RxP, TxP, const M: usize, const NQ: usize> TC6HighPrio<T, RxP, TxP, M, NQ>
where
    T: Tc6Interface,
    RxP: BoxPool<Data = FrameBuffer<M>> + 'static,
    TxP: BoxPool<Data = FrameBuffer<M>> + 'static,
{
    // Call this from every interrupt source the chip/interface can raise: the TC6 chip's own IRQ
    // pin *and* whatever signals SPI/DMA transfer completion (often a separate vector). This is
    // re-entrant/idempotent — it just re-checks `driver_mode` / `interface.transfer_complete()`
    // rather than assuming which source triggered it, so spurious or overlapping calls are fine.
    pub fn on_interrupt(&mut self) -> Result<(), T::Error> {
        // we might have a frame to receive
        loop {
            match self.driver_state {
                TC6DriverState::Idle => {
                    self.handle_idle()?;
                    if matches!(self.driver_state, TC6DriverState::Idle) {
                        // `handle_idle` started nothing (not configured yet, or no receive
                        // buffer / transmit frame available) - stop spinning.
                        break;
                    }
                }
                TC6DriverState::WaitingForTransmitComplete => {
                    if self.interface.transfer_complete()? {
                        self.handle_transmit_complete()?;
                    } else {
                        break;
                    }
                }
                TC6DriverState::WaitingForControlComplete => {
                    if self.interface.transfer_complete()? {
                        self.handle_control_complete()?;
                    } else {
                        break;
                    }
                }
            }
        }
        Ok(())
    }

    // This should be called whenever the low-prio side may have queued a frame to transmit:
    // either from whatever `TransmitSignal::signal` triggers, or from periodic polling if the
    // caller would rather not wire up a signal at all.
    pub fn on_transmit_ready(&mut self) -> Result<(), T::Error> {
        if !matches!(self.driver_state, TC6DriverState::Idle) {
            // still waiting for the previous transfer to complete, so we can't start a new one yet
            return Ok(());
        }
        self.handle_idle()?;
        Ok(())
    }

    fn handle_transmit_complete(&mut self) -> Result<(), T::Error> {
        let footer;
        {
            let bytes = if let (Some(_transmit_frame), Some(receive_frame)) = (
                self.current_transmit_buffer.as_mut(),
                self.current_receive_buffer.as_mut(),
            ) {
                // read the footer
                // Safety: before this function can be called, we have started a block send and the transfer has completed,
                // so the tc6 chip should have populated the 68 bytes at the end of the receive frame
                // that we sent in handle idle, so we can read the footer from the receive frame
                unsafe { receive_frame.get_mut_at_end(68) }
            } else if self.current_transmit_buffer.as_mut().is_some() {
                // read the footer
                &mut self.scratch_buffer
            } else if let Some(receive_frame) = self.current_receive_buffer.as_mut() {
                // read the footer
                // Safety: before this function can be called, we have started a block send and the transfer has completed,
                // so the tc6 chip should have populated the 68 bytes at the end of the receive frame
                // that we sent in handle idle, so we can read the footer from the receive frame
                unsafe { receive_frame.get_mut_at_end(68) }
            } else {
                unreachable!(
                    "how did we get here? We can't have a transfer complete without either the goal of reading (Some(ReceiveFrame)), or writing (Some(TransmitFrame)) or both, so this should not happen"
                )
            };

            if self.current_transmit_buffer.is_some() {
                let cursor = self.current_transmit_buffer_cursor;
                // A continuation block (cursor != 0) was sent as a joined header+data chunk,
                // which meant overwriting the 4 real frame bytes at [cursor-4, cursor) with
                // the chunk header. Those bytes were already transmitted in the previous
                // block, but restore them from `scratch_header` so the frame stays intact
                // (needed if it ever has to be retransmitted).
                if cursor != 0 {
                    let frame = self.current_transmit_buffer.as_mut().unwrap();
                    frame.as_mut_slice()[cursor - 4..cursor].copy_from_slice(&self.scratch_header);
                }

                self.current_transmit_buffer_cursor += 64;
                // if we have reached the end of the current transmit buffer drop/free the transmit box
                if self.current_transmit_buffer_cursor
                    >= self
                        .current_transmit_buffer
                        .as_ref()
                        .unwrap()
                        .as_slice()
                        .len()
                {
                    self.current_transmit_buffer = None;
                    self.current_transmit_buffer_cursor = 0;
                }
            }

            footer = tc6_footer::ReceiveFooter::from_bytes(bytes[64..68].try_into().unwrap());
        }
        // Every receive footer mirrors CONFIG0.SYNC. Losing it after bring-up means the chip
        // reset itself; flag that the host needs to re-run `configure` (spec Section 7.7).
        if self.sync_established && !footer.sync() {
            self.sync_established = false;
        }
        if footer.dv() {
            self.handle_data_valid(footer)?;
        }
        self.driver_state = TC6DriverState::Idle;
        Ok(())
    }

    fn handle_data_valid(&mut self, footer: ReceiveFooter) -> Result<(), T::Error> {
        // if we did not have a receive frame, then we have sent the norx flag with the header,
        // and therefore the tc6 should not have set dv.
        // but if it did either we messed up and did not set the norx flag, or the tc6 chip is misbehaving, either way we should panic.
        assert!(self.current_receive_buffer.is_some());
        let receive_frame = self.current_receive_buffer.as_mut().unwrap();
        // if receive frame is empty,
        let frame_done: bool;
        if receive_frame.as_slice().is_empty() {
            if footer.sv() {
                unsafe { receive_frame.set_start((footer.swo() << 2) as usize) };
                if footer.ev() {
                    unsafe {
                        receive_frame.add_len(footer.ebo() as usize + 1 - footer.swo() as usize * 4)
                    }
                    frame_done = true;
                } else {
                    unsafe { receive_frame.add_len(64 - footer.swo() as usize * 4) };
                    frame_done = false;
                }
            } else {
                panic!(
                    "we are expecting a start valid bit, as data valid is set and we are beginning a new frame (indicated by empty receive frame), but the start valid bit is not set, this should not happen"
                );
            }
        } else {
            if footer.ev() && footer.sv() {
                // if for some reason a new frame is started before the previous one is finished, we will panic, as this should not happen
                assert!(footer.ebo() < footer.swo() * 4);
            }
            // note this is only valid if ev is set
            if footer.sv() {
                assert!(footer.ev());

                // TODO possible optimization is to not write to temp buffer, and get box immediately.
                self.extra_start_buffer = Some(
                    heapless::Vec::from_slice(
                        // would want to use bytes here but due to lifetimes i had to move it out of scope, so we have an extra unsafe block here
                        &unsafe { receive_frame.get_mut_at_end(64) }[footer.swo() as usize * 4..],
                    )
                    .unwrap(),
                );
            }

            if footer.ev() {
                unsafe { receive_frame.add_len(footer.ebo() as usize + 1) }
                frame_done = true;
            } else {
                // dv is set, but ev and sv not set so whole block is valid and it's ok to add the whole block to the receive frame
                unsafe { receive_frame.add_len(64) }
                frame_done = false;
            }
        }

        if frame_done {
            self.rx_ready_producer
                .enqueue(self.current_receive_buffer.take().unwrap())
                .unwrap();
        }
        Ok(())
    }

    fn handle_idle(&mut self) -> Result<(), T::Error> {
        // A queued register read/write takes priority over data chunks: command mode is how the
        // host configures the chip in the first place (set CONFIG0.SYNC etc.), and while a
        // control transaction is pending we must not start a data transfer.
        if self.pending_control.is_some() {
            return self.start_control_transaction();
        }

        // Until `configure` has driven CONFIG0.SYNC = 1, the MAC-PHY ignores transmit data
        // chunks and returns DV = 0, so running the data path would just churn the pool and the
        // SPI bus for nothing. Stay idle; frames the low-prio side enqueues wait in the bounded
        // queue (and `transmit()` starts returning None once it fills).
        if !self.sync_established {
            return Ok(());
        }

        // do we have a receive buffer ready to fill? if not allocate one from the pool and queue it up for the tc6 chip to fill
        if self.current_receive_buffer.is_none() {
            // TODO make sure the compiler is smart and avoids a hugely expensive copy/move of the FrameBuffer struct here, since it is a large array. It should be optimized away by the compiler, but we should check.
            self.current_receive_buffer = RxP::alloc(FrameBuffer::new()).ok();
            if let Some(buff) = self.extra_start_buffer.as_ref() {
                self.current_receive_buffer
                    .as_mut()
                    .unwrap()
                    .push_slice(buff.as_slice());
            }
            // TODO, maybe a option::take() would be better, but doing this way as im not sure if take would clone the inner vec.
            self.extra_start_buffer = None;
        }

        if self.current_transmit_buffer.is_none() {
            // do we have a transmit buffer ready to send? if not, try to get one from the low-prio side
            self.current_transmit_buffer = self.tx_ready_consumer.dequeue();
            self.current_transmit_buffer_cursor = 0;
        }

        // if we have a transmit buffer ready send that
        if let Some(frame) = self.current_transmit_buffer.as_mut() {
            let len = frame.as_slice()[self.current_transmit_buffer_cursor..].len();
            let mut header = tc6_header::TransmitHeader::new();
            // TC6 spec 7.3.6: DNC shall be 1 for every transmit data chunk (0 = control command).
            header = header.with_dnc(true);
            header = header.with_dv(true);

            // end valid, if current block fits the last bit of the frame,
            let ev = len <= 64;
            header = header.with_ev(ev);
            // end byte offset, if current block fits the last bit of the frame,
            // then which byte in the frame is the last byte
            // -1, because zero indexed, and the end byte offset is inclusive, so if the frame is 1 byte long, the end byte offset is 0
            let ebo: u8 = if ev { len as u8 - 1 } else { 0 };
            header = header.with_ebo(ebo);

            if self.current_transmit_buffer_cursor == 0 {
                // we have no space we can use to write a header, so we can not use the more efficient write joined
                // also this is the start of data transmission, sv = 1, swo = 0.
                header = header.with_sv(true);
                header = header.with_swo(0);
                //safety: we are reading from a valid index, we might read more than what is initialized, but the swo and ebo will tell the tc6 chip how much of the data is valid, so it will ignore the rest
                let data = unsafe {
                    DMAPtr::from_slice(
                        frame.get_raw_mut_from_index(self.current_transmit_buffer_cursor, 64),
                    )
                };

                // we can't receive any data due to some reason,(like full receive box pool)
                if self.current_receive_buffer.is_none() {
                    header = header.with_norx(true)
                }

                let result = if let Some(f) = self.current_receive_buffer.as_mut() {
                    // SAFETY, we are writing to a point which is outside what is considered initialized, and we are not marking this part as initialized,
                    // that is done when we read the footer returned from the tc6 chip, and then we can determine what is valid and update the length.
                    unsafe { DMAPtr::from_slice(f.get_mut_at_end(68)) }
                } else {
                    DMAPtr::from_slice(&mut self.scratch_buffer)
                };
                header = header.with_correct_parity();
                let header_raw: u32 = header.raw();
                self.interface.begin_send_block(header_raw, data, result)?;
                self.driver_state = TC6DriverState::WaitingForTransmitComplete;
            } else {
                // Continuation block: the 4 frame bytes just before the cursor were already
                // sent in the previous block, so we can borrow them to hold this chunk's
                // header and send header + 64 data bytes as one joined transfer. `header`
                // (dnc/dv/ev/ebo) is already set correctly above; a continuation block is
                // never a frame start, so sv/swo stay 0.
                let cursor = self.current_transmit_buffer_cursor;
                let header = header.with_correct_parity();

                // stash the real bytes we're about to overwrite so the frame can be restored
                // once this transfer completes (see handle_transmit_complete)
                self.scratch_header
                    .copy_from_slice(&frame.as_slice()[cursor - 4..cursor]);
                frame.as_mut_slice()[cursor - 4..cursor]
                    .copy_from_slice(&header.raw().to_be_bytes());

                // SAFETY: the window is [cursor-4, cursor+64); on the final (partial) block
                // its tail runs past the initialized frame data into spare capacity - the
                // chip ignores those bytes because ebo marks the last valid one.
                let data = DMAPtr::from_slice(unsafe { frame.get_mut_at_index(cursor - 4, 68) });
                let result = if let Some(f) = self.current_receive_buffer.as_mut() {
                    // SAFETY, we are writing to a point which is outside what is considered initialized, and we are not marking this part as initialized,
                    // that is done when we read the footer returned from the tc6 chip, and then we can determine what is valid and update the length.
                    unsafe { DMAPtr::from_slice(f.get_mut_at_end(68)) }
                } else {
                    DMAPtr::from_slice(&mut self.scratch_buffer)
                };
                self.interface.begin_send_joined_block(data, result)?;
                self.driver_state = TC6DriverState::WaitingForTransmitComplete;
            }
        } else if let Some(f) = self.current_receive_buffer.as_mut() {
            // irq fired, we have data to be read
            let mut header = tc6_header::TransmitHeader::new();
            // still a data chunk (DNC = 1), we just have nothing to send, so dv = 0
            header = header.with_dnc(true);
            header = header.with_dv(false);
            header = header.with_correct_parity();
            self.scratch_buffer[0..4].copy_from_slice(&header.raw().to_be_bytes());
            let data = DMAPtr::from_slice(&mut self.scratch_buffer);
            // SAFETY, we are writing to a point which is outside what is considered initialized, and we are not marking this part as initialized,
            // that is done when we read the footer returned from the tc6 chip, and then we can determine what is valid and update the length.
            let result = unsafe { DMAPtr::from_slice(f.get_mut_at_end(68)) };
            self.interface.begin_send_joined_block(data, result)?;
            self.driver_state = TC6DriverState::WaitingForTransmitComplete;
        } else {
            // irq fired, but we have no data to send nor space to receive data
            todo!("what should we do here?")
        }
        Ok(())
    }

    // --- command mode: register read / write (TC6 spec Section 7.4) ---

    /// `true` if a control transaction is pending or in flight, i.e. the register read/write
    /// entry points would return [`ControlError::Busy`].
    pub fn control_busy(&self) -> bool {
        self.pending_control.is_some()
            || self.control_in_flight.is_some()
            || matches!(self.driver_state, TC6DriverState::WaitingForControlComplete)
    }

    /// Take the result of the last completed control transaction, if one is waiting.
    pub fn take_control_result(&mut self) -> Option<ControlResult> {
        self.last_control_result.take()
    }

    /// Queue a read of `count` consecutive registers starting at `addr` within memory map `mms`
    /// (see `crate::registers` for the standard map and its addresses). The address
    /// post-increments per register (TC6 `AID` = 0). Pump [`on_interrupt`](Self::on_interrupt)
    /// until [`take_control_result`](Self::take_control_result) returns the values.
    pub fn begin_register_read(
        &mut self,
        mms: u8,
        addr: u16,
        count: u8,
    ) -> Result<(), ControlError<T::Error>> {
        self.queue_control(PendingControl {
            write: false,
            mms,
            addr,
            count,
            values: [0; CONTROL_MAX_REGS],
        })
    }

    /// Queue a write of `values` to consecutive registers starting at `addr` within memory map
    /// `mms`. The address post-increments per register (TC6 `AID` = 0). At most
    /// [`CONTROL_MAX_REGS`] values.
    pub fn begin_register_write(
        &mut self,
        mms: u8,
        addr: u16,
        values: &[u32],
    ) -> Result<(), ControlError<T::Error>> {
        if values.is_empty() || values.len() > CONTROL_MAX_REGS {
            return Err(ControlError::UnsupportedCount);
        }
        let mut buf = [0u32; CONTROL_MAX_REGS];
        buf[..values.len()].copy_from_slice(values);
        self.queue_control(PendingControl {
            write: true,
            mms,
            addr,
            count: values.len() as u8,
            values: buf,
        })
    }

    fn queue_control(&mut self, req: PendingControl) -> Result<(), ControlError<T::Error>> {
        if self.control_busy() {
            return Err(ControlError::Busy);
        }
        if req.count == 0 || req.count as usize > CONTROL_MAX_REGS {
            return Err(ControlError::UnsupportedCount);
        }
        self.pending_control = Some(req);
        // If the state machine is otherwise idle, kick it now so callers that don't wire up an
        // interrupt source (e.g. a blocking init loop) still make progress. Otherwise it is
        // picked up by `handle_idle` once the in-flight transfer finishes.
        if matches!(self.driver_state, TC6DriverState::Idle) {
            self.handle_idle().map_err(ControlError::Interface)?;
        }
        Ok(())
    }

    /// Lay the pending request out in `control_tx_buffer` and hand it to the interface.
    fn start_control_transaction(&mut self) -> Result<(), T::Error> {
        let req = self
            .pending_control
            .take()
            .expect("start_control_transaction called with no pending request");
        let n = req.count as usize;
        // header (4B) + n data words + trailing don't-care word (4B), see spec Figures 13/14.
        let total = 4 * (n + 2);

        let header = tc6_header::ControlHeader::new()
            .with_wnr(req.write)
            .with_mms(req.mms)
            .with_addr(req.addr)
            .with_len(req.count - 1) // LEN is register count minus one
            .with_correct_parity();

        self.control_tx_buffer[..total].fill(0);
        self.control_tx_buffer[0..4].copy_from_slice(&header.to_bytes());
        if req.write {
            for (i, word) in req.values[..n].iter().enumerate() {
                self.control_tx_buffer[4 + 4 * i..8 + 4 * i]
                    .copy_from_slice(&word.to_be_bytes());
            }
        }
        self.control_rx_buffer[..total].fill(0);

        let tx = DMAPtr::from_slice(&mut self.control_tx_buffer[..total]);
        let rx = DMAPtr::from_slice(&mut self.control_rx_buffer[..total]);
        self.interface.begin_control_transaction(tx, rx)?;

        self.control_in_flight = Some(ControlInFlight {
            write: req.write,
            count: req.count,
        });
        self.driver_state = TC6DriverState::WaitingForControlComplete;
        Ok(())
    }

    /// Parse the MISO stream of a finished control transaction into a [`ControlResult`].
    fn handle_control_complete(&mut self) -> Result<(), T::Error> {
        let in_flight = self
            .control_in_flight
            .take()
            .expect("handle_control_complete called with no transaction in flight");
        let n = in_flight.count as usize;

        // MISO layout: [0..4) don't-care, [4..8) echoed control header, [8..) echoed/read words.
        let echoed_header = tc6_header::ControlHeader::from_bytes(
            self.control_rx_buffer[4..8].try_into().unwrap(),
        );
        // On a header parity error the MAC-PHY streams the 0xC0000001 error code where the
        // echoed header would be (spec Section 7.5.1); it decodes to HDRB = 1, so this catches
        // both that and an ordinary bad-header echo.
        let header_bad = echoed_header.hdrb() || !echoed_header.parity_ok();

        let mut values = [0u32; CONTROL_MAX_REGS];
        if !header_bad {
            for (i, slot) in values[..n].iter_mut().enumerate() {
                let off = 8 + 4 * i;
                *slot = u32::from_be_bytes(
                    self.control_rx_buffer[off..off + 4].try_into().unwrap(),
                );
            }
        }

        self.last_control_result = Some(ControlResult {
            echoed_header,
            header_bad,
            write: in_flight.write,
            values,
            count: in_flight.count,
        });
        self.driver_state = TC6DriverState::Idle;
        Ok(())
    }

    // --- blocking bring-up (register configuration, spec Sections 7.6 / 9.2) ---

    /// `true` once [`configure`](Self::configure) has driven `CONFIG0.SYNC = 1` and no receive
    /// footer has since reported `SYNC = 0`. While this is `false` the MAC-PHY ignores transmit
    /// data chunks and delivers no receive frames.
    pub fn is_configured(&self) -> bool {
        self.sync_established
    }

    /// Run the mandatory bring-up sequence, busy-polling each control transfer to completion:
    ///
    /// 1. wait for `STATUS0.RESETC` (reset complete),
    /// 2. sanity-check `IDVER`,
    /// 3. gate requested features on `STDCAP`,
    /// 4. write `CONFIG0` with a 64-byte chunk payload and `SYNC = 1`,
    /// 5. read `CONFIG0` back and confirm `SYNC` stuck,
    /// 6. program `IMASK0`,
    /// 7. clear latched `STATUS0` bits (including `RESETC`),
    /// 8. if `cfg.plca` is set, write this node's PLCA ID / node count and enable PLCA.
    ///
    /// Call this from init (or after a footer reports `SYNC = 0`) *before* servicing data
    /// traffic. It only touches control registers - the frame queues are untouched - and it
    /// polls [`Tc6Interface::transfer_complete`] directly, so it does not need an interrupt
    /// source wired up yet. On [`ConfigError::Timeout`] the state machine is returned to idle,
    /// but a genuinely stuck transfer may still be running on the interface.
    pub fn configure(&mut self, cfg: &Config) -> Result<(), ConfigError<T::Error>> {
        use registers::{addr, mms, Config0, IdVer, Status0, StdCap};

        if self.control_busy() {
            return Err(ConfigError::Busy);
        }

        // 1. reset complete?
        let mut tries = 0u32;
        loop {
            let s0 = Status0::from_raw(self.read_reg_blocking(
                mms::STANDARD,
                addr::STATUS0,
                cfg.transfer_poll_limit,
            )?);
            if s0.resetc() {
                break;
            }
            tries += 1;
            if tries >= cfg.reset_poll_limit {
                return Err(ConfigError::ResetNotComplete);
            }
        }

        // 2. is anyone home?
        let id = IdVer::from_raw(self.read_reg_blocking(
            mms::STANDARD,
            addr::IDVER,
            cfg.transfer_poll_limit,
        )?);
        if id.majver() == 0 {
            return Err(ConfigError::BadVersion {
                major: id.majver(),
                minor: id.minver(),
            });
        }

        // 3. capability gating
        let cap = StdCap::from_raw(self.read_reg_blocking(
            mms::STANDARD,
            addr::STDCAP,
            cfg.transfer_poll_limit,
        )?);
        if (cfg.cut_through_tx || cfg.cut_through_rx) && !cap.ctc() {
            return Err(ConfigError::Unsupported("cut-through"));
        }
        if cfg.timestamp.is_some() && !cap.ftsc() {
            return Err(ConfigError::Unsupported("frame timestamp"));
        }

        // 4. write CONFIG0 with the driver's required 64-byte chunk payload and SYNC = 1
        let mut c0 = Config0::from_raw(0)
            .with_cps(6) // 2^6 = 64-byte data chunk payload
            .with_txcte(cfg.cut_through_tx)
            .with_rxcte(cfg.cut_through_rx)
            .with_txcthresh(cfg.tx_credit_threshold.bits());
        if let Some(size) = cfg.timestamp {
            c0 = c0
                .with_ftse(true)
                .with_ftss(matches!(size, TimestampSize::Bits64));
        }
        c0 = c0.with_sync(true);
        self.write_reg_blocking(
            mms::STANDARD,
            addr::CONFIG0,
            c0.raw(),
            cfg.transfer_poll_limit,
        )?;

        // 5. confirm SYNC stuck (spec 7.6: once set it cannot be written back to 0)
        let readback = Config0::from_raw(self.read_reg_blocking(
            mms::STANDARD,
            addr::CONFIG0,
            cfg.transfer_poll_limit,
        )?);
        if !readback.sync() || readback.cps() != 6 {
            return Err(ConfigError::SyncFailed);
        }

        // 6. interrupt mask
        self.write_reg_blocking(
            mms::STANDARD,
            addr::IMASK0,
            cfg.interrupt_mask.raw(),
            cfg.transfer_poll_limit,
        )?;

        // 7. clear RESETC and any latched errors
        self.write_reg_blocking(
            mms::STANDARD,
            addr::STATUS0,
            Status0::clear_all().raw(),
            cfg.transfer_poll_limit,
        )?;

        // 8. optional PLCA node setup (PHY MMS 4): write ID + node count, then enable
        if let Some(p) = cfg.plca {
            use registers::{plca, PlcaCtrl0, PlcaCtrl1};
            self.write_reg_blocking(
                mms::PHY_VS_PLCA,
                plca::CTRL1,
                PlcaCtrl1::from_raw(0)
                    .with_id(p.node_id)
                    .with_node_count(p.node_count)
                    .raw(),
                cfg.transfer_poll_limit,
            )?;
            self.write_reg_blocking(
                mms::PHY_VS_PLCA,
                plca::CTRL0,
                PlcaCtrl0::from_raw(0).with_enable(true).raw(),
                cfg.transfer_poll_limit,
            )?;
        }

        self.sync_established = true;
        Ok(())
    }

    fn read_reg_blocking(
        &mut self,
        mms: u8,
        addr: u16,
        poll_limit: u32,
    ) -> Result<u32, ConfigError<T::Error>> {
        let res = self.run_control_blocking(
            PendingControl {
                write: false,
                mms,
                addr,
                count: 1,
                values: [0; CONTROL_MAX_REGS],
            },
            poll_limit,
        )?;
        if res.header_bad {
            return Err(ConfigError::HeaderRejected);
        }
        Ok(res.values()[0])
    }

    fn write_reg_blocking(
        &mut self,
        mms: u8,
        addr: u16,
        value: u32,
        poll_limit: u32,
    ) -> Result<(), ConfigError<T::Error>> {
        let mut values = [0; CONTROL_MAX_REGS];
        values[0] = value;
        let res = self.run_control_blocking(
            PendingControl {
                write: true,
                mms,
                addr,
                count: 1,
                values,
            },
            poll_limit,
        )?;
        if res.header_bad {
            return Err(ConfigError::HeaderRejected);
        }
        Ok(())
    }

    /// Issue one control transaction and spin on [`Tc6Interface::transfer_complete`] until it
    /// finishes or `poll_limit` iterations elapse. Never enters the data path.
    fn run_control_blocking(
        &mut self,
        req: PendingControl,
        poll_limit: u32,
    ) -> Result<ControlResult, ConfigError<T::Error>> {
        debug_assert!(!self.control_busy(), "run_control_blocking while busy");
        self.pending_control = Some(req);
        self.start_control_transaction()
            .map_err(ConfigError::Interface)?;

        let mut spins = 0u32;
        loop {
            if self
                .interface
                .transfer_complete()
                .map_err(ConfigError::Interface)?
            {
                self.handle_control_complete()
                    .map_err(ConfigError::Interface)?;
                return Ok(self
                    .last_control_result
                    .take()
                    .expect("handle_control_complete stores a result"));
            }
            spins += 1;
            if spins >= poll_limit {
                // don't leave the state machine wedged in WaitingForControlComplete
                self.control_in_flight = None;
                self.driver_state = TC6DriverState::Idle;
                return Err(ConfigError::Timeout);
            }
        }
    }
}

pub struct TC6DriverRxToken<RxP: BoxPool> {
    frame: Box<RxP>,
}

impl<RxP, const M: usize> smoltcp::phy::RxToken for TC6DriverRxToken<RxP>
where
    RxP: BoxPool<Data = FrameBuffer<M>> + 'static,
{
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&[u8]) -> R,
    {
        f(self.frame.as_slice())
    }

    fn meta(&self) -> smoltcp::phy::PacketMeta {
        smoltcp::phy::PacketMeta::default()
    }
}

pub struct TC6DriverTxToken<'a, TxP: BoxPool + 'static, S> {
    frame: Box<TxP>,
    queue: &'a mut Producer<'static, Box<TxP>>,
    // poked once the frame has been written and queued, so the high-prio side wakes up and
    // pushes it over SPI (see `TransmitSignal`)
    signal: &'a mut S,
}

impl<'a, TxP, S, const M: usize> smoltcp::phy::TxToken for TC6DriverTxToken<'a, TxP, S>
where
    TxP: BoxPool<Data = FrameBuffer<M>> + 'static,
    S: TransmitSignal,
{
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let TC6DriverTxToken {
            mut frame,
            queue,
            signal,
        } = self;
        // safety: we say that this is initialized, as we assume that FnOnce will write to the buffer
        let r = unsafe {
            frame.add_len(len);
            f(frame.as_mut_slice())
        };

        queue.enqueue(frame).unwrap();
        // frame is queued: wake the high-prio side to service it
        signal.signal();
        r
    }
}

impl<RxP, TxP, S, const M: usize, const NQ: usize> smoltcp::phy::Device
    for TC6LowPrio<RxP, TxP, S, M, NQ>
where
    RxP: BoxPool<Data = FrameBuffer<M>> + 'static,
    TxP: BoxPool<Data = FrameBuffer<M>> + 'static,
    S: TransmitSignal,
{
    type RxToken<'a>
        = TC6DriverRxToken<RxP>
    where
        Self: 'a;
    type TxToken<'a>
        = TC6DriverTxToken<'a, TxP, S>
    where
        Self: 'a;

    fn capabilities(&self) -> smoltcp::phy::DeviceCapabilities {
        let mut cap = DeviceCapabilities::default();
        cap.medium = smoltcp::phy::Medium::Ethernet;
        cap.max_burst_size = Some(NQ);
        // set default checksum capabilities, which is calculated and verified by software (TODO find out if this is the case, and that i have not mixed it up)
        cap.checksum = smoltcp::phy::ChecksumCapabilities::default();
        cap.max_transmission_unit = M; //TODO actually determine what the correct MTU should be for a ethernet frame
        cap
    }

    fn receive(
        &mut self,
        _timestamp: smoltcp::time::Instant,
    ) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        // only report a received frame if a tx buffer will also be available (see paired-token
        // note: smoltcp always wants both, so this doubles as the anti-deadlock guard)
        if self.rx_ready_consumer.is_empty() {
            return None;
        }
        // TODO check if box can be created as well
        if !self.tx_ready_producer.ready() {
            return None;
        }

        // safety: we have already checked that the rx_ready_consumer is not empty, so dequeue is safe
        let rx_frame = self.rx_ready_consumer.dequeue().unwrap();
        let rx_token = TC6DriverRxToken { frame: rx_frame };

        // TODO make sure the compiler is smart and avoids a hugely expensive copy/move of the FrameBuffer struct here, since it is a large array. It should be optimized away by the compiler, but we should check.
        let tx_frame = TxP::alloc(FrameBuffer::new()).unwrap();
        let tx_token = TC6DriverTxToken {
            frame: tx_frame,
            queue: &mut self.tx_ready_producer,
            signal: &mut self.transmit_signal,
        };
        Some((rx_token, tx_token))
    }

    fn transmit(&mut self, _timestamp: smoltcp::time::Instant) -> Option<Self::TxToken<'_>> {
        if !self.tx_ready_producer.ready() {
            return None;
        }
        let tx_frame = TxP::alloc(FrameBuffer::new()).unwrap();
        Some(TC6DriverTxToken {
            frame: tx_frame,
            queue: &mut self.tx_ready_producer,
            signal: &mut self.transmit_signal,
        })
    }
}

#[cfg(test)]
mod driver_tests {
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

    const M: usize = 1536; // FrameBuffer capacity (bytes)
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
        /// footers handed back by `respond` carry SYNC = 1 (a configured chip) unless cleared
        footer_sync: bool,
    }

    impl Default for SpyInterface {
        fn default() -> Self {
            Self {
                chunks: Vec::new(),
                control_txns: Vec::new(),
                complete: false,
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
            let (dptr, dlen) = unsafe { data.read_buffer() };
            assert_eq!(dlen, 64, "a data chunk payload is one 64-byte block");
            let mut snap = [0u8; 64];
            snap.copy_from_slice(unsafe { core::slice::from_raw_parts(dptr, 64) });
            let (rptr, rlen) = unsafe { result.write_buffer() };
            self.chunks.push(Chunk {
                header,
                data: snap,
                joined: false,
                result_ptr: rptr,
                result_len: rlen,
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
            let (dptr, dlen) = unsafe { data.read_buffer() };
            assert!(dlen >= 68);
            let hdr: [u8; 4] = unsafe { core::slice::from_raw_parts(dptr, 4) }
                .try_into()
                .unwrap();
            let mut snap = [0u8; 64];
            snap.copy_from_slice(unsafe { core::slice::from_raw_parts(dptr.add(4), 64) });
            let (rptr, rlen) = unsafe { result.write_buffer() };
            self.chunks.push(Chunk {
                header: u32::from_be_bytes(hdr),
                data: snap,
                joined: true,
                result_ptr: rptr,
                result_len: rlen,
            });
            Ok(())
        }

        fn begin_control_transaction(
            &mut self,
            tx: impl ReadBuffer<Word = u8>,
            mut rx: impl WriteBuffer<Word = u8>,
        ) -> Result<(), Self::Error> {
            let (dptr, dlen) = unsafe { tx.read_buffer() };
            let (rptr, rlen) = unsafe { rx.write_buffer() };
            assert_eq!(dlen, rlen, "control tx/rx buffers must match length");
            assert!(dlen >= 12 && dlen % 4 == 0, "at least header + 1 word + pad");
            let tx = unsafe { core::slice::from_raw_parts(dptr, dlen) }.to_vec();
            self.control_txns.push(ControlTxn {
                tx,
                result_ptr: rptr,
                result_len: rlen,
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
    struct PollingFake {
        inner: crate::embeded_io_implementation::EmbeddedHalImplementation<
            fake_tc6::fake_tc6_device::FakeTc6SpiDevice,
        >,
        polls_until_done: u32,
    }
    impl PollingFake {
        fn new(spi: fake_tc6::fake_tc6_device::FakeTc6SpiDevice) -> Self {
            Self {
                inner: crate::embeded_io_implementation::EmbeddedHalImplementation::new(spi),
                polls_until_done: 0,
            }
        }
    }
    impl Tc6Interface for PollingFake {
        type Error = crate::embeded_io_implementation::EmbeddedHalImplementationError<
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
                    let mut newv = val;
                    if registers::Config0::from_raw(cur).sync() {
                        newv |= 1 << 15;
                    }
                    if self.ignore_sync_write {
                        newv &= !(1 << 15);
                    }
                    self.regs.insert((0, 0x0004), newv);
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
            let (dptr, dlen) = unsafe { tx.read_buffer() };
            let (rptr, rlen) = unsafe { rx.write_buffer() };
            assert_eq!(dlen, rlen);
            let txbuf = unsafe { core::slice::from_raw_parts(dptr, dlen) };
            let rxbuf = unsafe { core::slice::from_raw_parts_mut(rptr, rlen) };

            let hdr = tc6_header::ControlHeader::from_bytes(txbuf[0..4].try_into().unwrap());
            let n = hdr.len() as usize + 1;

            rxbuf.fill(0);
            rxbuf[4..8].copy_from_slice(&txbuf[0..4]); // echo the command header
            for i in 0..n {
                let addr = hdr.addr() + i as u16;
                let out = if hdr.wnr() {
                    let val = u32::from_be_bytes(txbuf[4 + 4 * i..8 + 4 * i].try_into().unwrap());
                    self.apply_write(hdr.mms(), addr, val);
                    val // echo write data (spec 7.4.4)
                } else {
                    self.reg(hdr.mms(), addr)
                };
                rxbuf[8 + 4 * i..12 + 4 * i].copy_from_slice(&out.to_be_bytes());
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
        ($hi:ident, $lo:ident, $iface:expr, $signal:expr) => {
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
            let (mut $hi, mut $lo) = split(($iface), rx_q, tx_q, ($signal));
        };
    }

    /// Like [`setup!`] but goes through [`split_configured`], unwrapping the bring-up result.
    macro_rules! setup_configured {
        ($hi:ident, $lo:ident, $iface:expr, $signal:expr, $cfg:expr) => {
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
            let (mut $hi, mut $lo) = split_configured(($iface), rx_q, tx_q, ($signal), $cfg)
                .expect("bring-up should succeed");
        };
    }

    fn ethernet_like(len: usize) -> Vec<u8> {
        (0..len)
            .map(|i| (i as u8).wrapping_mul(31).wrapping_add(7))
            .collect()
    }

    // --------------------------------------------------------------------------------------
    // low-priority side (smoltcp::phy::Device)
    // --------------------------------------------------------------------------------------

    #[test]
    fn low_prio_reports_expected_capabilities() {
        setup!(_hi, lo, SpyInterface::default(), NoopSignal);
        let caps = lo.capabilities();
        assert_eq!(caps.medium, Medium::Ethernet);
        assert_eq!(caps.max_transmission_unit, M);
        assert_eq!(caps.max_burst_size, Some(NQ));
    }

    #[test]
    fn transmit_returns_none_once_tx_queue_is_full() {
        setup!(_hi, lo, SpyInterface::default(), NoopSignal);
        // usable capacity is NQ - 1
        for _ in 0..(NQ - 1) {
            let tok = lo
                .transmit(Instant::ZERO)
                .expect("token while queue has room");
            tok.consume(64, |_buf| {});
        }
        assert!(
            lo.transmit(Instant::ZERO).is_none(),
            "no token once the tx-ready queue is full"
        );
    }

    #[test]
    fn receive_returns_none_when_no_frame_pending() {
        setup!(_hi, lo, SpyInterface::default(), NoopSignal);
        assert!(lo.receive(Instant::ZERO).is_none());
    }

    #[test]
    fn receive_returns_none_when_tx_side_not_ready() {
        setup!(hi, lo, SpyInterface::default(), NoopSignal);

        // a frame is waiting to be received...
        hi.rx_ready_producer
            .enqueue(RxPool.alloc(FrameBuffer::new()).unwrap())
            .unwrap();
        // ...but smoltcp always wants a paired tx token, and the tx queue is full
        for _ in 0..(NQ - 1) {
            lo.transmit(Instant::ZERO).unwrap().consume(64, |_| {});
        }

        assert!(
            lo.receive(Instant::ZERO).is_none(),
            "receive must not hand out an rx token it can't pair with a tx token"
        );
    }

    #[test]
    fn tx_token_signals_high_prio_once_the_frame_is_written() {
        setup!(_hi, lo, SpyInterface::default(), CountingSignal::default());
        assert_eq!(lo.transmit_signal.count, 0);

        // a token handed out but dropped unconsumed must not signal
        lo.transmit(Instant::ZERO).expect("tx token");
        assert_eq!(lo.transmit_signal.count, 0);

        {
            let tok = lo.transmit(Instant::ZERO).expect("tx token");
            tok.consume(64, |b| b.copy_from_slice(&ethernet_like(64)));
        }
        assert_eq!(lo.transmit_signal.count, 1, "woken once, after enqueue");

        {
            let tok = lo.transmit(Instant::ZERO).expect("tx token");
            tok.consume(64, |b| b.copy_from_slice(&ethernet_like(64)));
        }
        assert_eq!(lo.transmit_signal.count, 2, "once per queued frame");
    }

    #[test]
    fn receive_paired_tx_token_also_signals() {
        setup!(hi, lo, SpyInterface::default(), CountingSignal::default());
        hi.rx_ready_producer
            .enqueue(RxPool.alloc(FrameBuffer::new()).unwrap())
            .unwrap();

        {
            let (rx, tx) = lo.receive(Instant::ZERO).expect("rx + tx tokens");
            rx.consume(|_| {});
            tx.consume(64, |b| b.copy_from_slice(&ethernet_like(64)));
        }
        assert_eq!(lo.transmit_signal.count, 1);
    }

    // --------------------------------------------------------------------------------------
    // high-priority side: transmit path
    // --------------------------------------------------------------------------------------

    #[test]
    fn transmit_token_builds_one_full_data_chunk() {
        setup!(hi, lo, SpyInterface::default(), NoopSignal);
        hi.sync_established = true; // data-path test: pretend configure() ran

        let payload = ethernet_like(64);
        {
            let tok = lo.transmit(Instant::ZERO).expect("tx token");
            tok.consume(payload.len(), |buf| {
                assert_eq!(buf.len(), 64);
                buf.copy_from_slice(&payload);
            });
        }

        hi.on_transmit_ready().unwrap();

        assert_eq!(hi.interface.chunks.len(), 1, "exactly one chunk sent");
        let c = hi.interface.last();
        assert!(!c.joined);
        assert_eq!(&c.data[..], payload.as_slice());

        let h = tc6_header::TransmitHeader::from_raw(c.header);
        assert!(h.dnc(), "TC6 7.3.6: data chunks must set DNC");
        assert!(h.dv());
        assert!(h.sv());
        assert_eq!(h.swo(), 0);
        assert!(h.ev(), "a 64-byte frame ends within this block");
        assert_eq!(h.ebo(), 63);
        assert!(h.parity_ok());

        assert!(matches!(
            hi.driver_state,
            TC6DriverState::WaitingForTransmitComplete
        ));
    }

    #[test]
    fn transmit_completion_without_receive_data_returns_to_idle() {
        setup!(hi, lo, SpyInterface::default(), NoopSignal);
        hi.sync_established = true; // data-path test: pretend configure() ran

        {
            let tok = lo.transmit(Instant::ZERO).unwrap();
            tok.consume(64, |buf| buf.copy_from_slice(&ethernet_like(64)));
        }
        hi.on_transmit_ready().unwrap();

        // MAC-PHY says: transfer done, nothing to receive (DV = 0 footer).
        hi.interface.respond(
            &[],
            crate::tc6_footer::ReceiveFooter::new().with_correct_parity(),
        );
        hi.handle_transmit_complete().unwrap();

        assert!(matches!(hi.driver_state, TC6DriverState::Idle));
        assert_eq!(
            hi.interface.chunks.len(),
            1,
            "completion of a chunk must not itself emit another chunk"
        );
    }

    // --------------------------------------------------------------------------------------
    // high-priority side: receive path
    // --------------------------------------------------------------------------------------

    #[test]
    fn receives_single_block_frame_and_hands_it_to_low_prio() {
        setup!(hi, lo, SpyInterface::default(), NoopSignal);
        hi.sync_established = true; // data-path test: pretend configure() ran

        // no frame queued for tx -> handle_idle issues a receive-only ("joined") chunk
        hi.handle_idle().unwrap();
        {
            let c = hi.interface.last();
            assert!(c.joined);
            assert!(
                !tc6_header::TransmitHeader::from_raw(c.header).dv(),
                "receive-only chunk carries no tx data"
            );
        }

        let frame = ethernet_like(20);
        let footer = crate::tc6_footer::ReceiveFooter::new()
            .with_dv(true)
            .with_sv(true)
            .with_swo(0)
            .with_ev(true)
            .with_ebo(frame.len() as u8 - 1)
            .with_correct_parity();
        hi.interface.respond(&frame, footer);

        hi.handle_transmit_complete().unwrap();
        assert!(matches!(hi.driver_state, TC6DriverState::Idle));

        // the low-priority side should now see exactly that frame
        let (rx, _tx) = lo.receive(Instant::ZERO).expect("rx + tx tokens");
        rx.consume(|buf| assert_eq!(buf, frame.as_slice()));
        assert!(
            lo.receive(Instant::ZERO).is_none(),
            "rx-ready queue drained after one frame"
        );
    }

    #[test]
    fn receive_only_chunk_targets_the_receive_buffer_tail() {
        setup!(hi, _lo, SpyInterface::default(), NoopSignal);
        hi.sync_established = true; // data-path test: pretend configure() ran

        hi.handle_idle().unwrap();

        // the MISO target must be inside the freshly allocated receive FrameBuffer, and 68
        // bytes wide (64 payload + 4 footer)
        let c = hi.interface.last();
        assert_eq!(c.result_len, 68);
        assert!(hi.current_receive_buffer.is_some());
    }

    // --------------------------------------------------------------------------------------
    // interface: begin_send_joined_block default implementation
    // --------------------------------------------------------------------------------------

    #[test]
    fn joined_block_default_impl_forwards_header_and_payload() {
        #[derive(Default)]
        struct Recording {
            last: Option<(u32, [u8; 64], usize)>,
        }
        impl Tc6Interface for Recording {
            type Error = core::convert::Infallible;
            fn begin_send_block(
                &mut self,
                header: u32,
                data: impl ReadBuffer<Word = u8>,
                mut result: impl WriteBuffer<Word = u8>,
            ) -> Result<(), Self::Error> {
                let (dptr, dlen) = unsafe { data.read_buffer() };
                assert_eq!(dlen, 64);
                let mut snap = [0u8; 64];
                snap.copy_from_slice(unsafe { core::slice::from_raw_parts(dptr, 64) });
                let (_rptr, rlen) = unsafe { result.write_buffer() };
                self.last = Some((header, snap, rlen));
                Ok(())
            }
            fn transfer_complete(&mut self) -> Result<bool, Self::Error> {
                Ok(true)
            }
            fn begin_control_transaction(
                &mut self,
                _tx: impl ReadBuffer<Word = u8>,
                _rx: impl WriteBuffer<Word = u8>,
            ) -> Result<(), Self::Error> {
                unreachable!("this test does not exercise control transactions")
            }
        }

        let header = tc6_header::TransmitHeader::new()
            .with_dnc(true)
            .with_dv(false)
            .with_correct_parity();

        let mut data = [0u8; 68];
        data[0..4].copy_from_slice(&header.to_bytes());
        for (i, b) in data[4..68].iter_mut().enumerate() {
            *b = i as u8;
        }
        let mut result = [0u8; 68];

        let mut iface = Recording::default();
        iface
            .begin_send_joined_block(RawBuf::new(&mut data[..]), RawBuf::new(&mut result[..]))
            .unwrap();

        let (h, payload, rlen) = iface.last.expect("forwarded to begin_send_block");
        assert_eq!(h, header.raw());
        assert_eq!(rlen, 68);
        assert_eq!(payload, core::array::from_fn::<u8, 64, _>(|i| i as u8));
    }

    // --------------------------------------------------------------------------------------
    // end-to-end through the real SPI wrapper + fake_tc6
    // --------------------------------------------------------------------------------------

    #[test]
    fn round_trip_transmit_through_fake_tc6() {
        use crate::embeded_io_implementation::EmbeddedHalImplementation;
        use fake_tc6::fake_tc6_device::FakeTc6SpiDevice;

        let iface = EmbeddedHalImplementation::new(FakeTc6SpiDevice::new());
        setup!(hi, lo, iface, NoopSignal);
        hi.sync_established = true; // data-path test: pretend configure() ran

        let payload = ethernet_like(64);
        {
            let tok = lo.transmit(Instant::ZERO).expect("tx token");
            tok.consume(payload.len(), |buf| buf.copy_from_slice(&payload));
        }

        assert!(hi.on_transmit_ready().is_ok());

        // get interface to the underlying fake device
        let got = hi
            .interface
            .inner_mut()
            .device
            .pop_transmit_frame()
            .expect("fake device received a frame");
        assert_eq!(got, payload);
        assert!(
            hi.interface
                .inner_mut()
                .device
                .pop_transmit_frame()
                .is_none(),
            "exactly one frame put on the wire"
        );
    }

    // --------------------------------------------------------------------------------------
    // regression pins for known-broken paths (see module-level notes / PR discussion)
    // --------------------------------------------------------------------------------------

    /// A receive frame larger than one 64-byte block: the first block carries the start
    /// (SV, no EV) and fills the whole chunk, the second carries the end (EV, no SV). The
    /// driver must stitch them into one contiguous frame and hand it to the low-prio side.
    ///
    /// (Previously pinned as `#[should_panic]`: `handle_data_valid` used to null
    /// `current_receive_buffer` right before `if frame_done { ...take().unwrap() }`. That
    /// line has since been removed, so this is now a positive test.)
    #[test]
    fn receives_frame_spanning_two_blocks() {
        setup!(hi, lo, SpyInterface::default(), NoopSignal);
        hi.sync_established = true; // data-path test: pretend configure() ran

        let head = ethernet_like(64);
        let tail: Vec<u8> = (0..20u8).map(|i| 200 + i).collect();

        // block 0: start of frame, fills the whole chunk (SV, no EV)
        hi.handle_idle().unwrap();
        hi.interface.respond(
            &head,
            crate::tc6_footer::ReceiveFooter::new()
                .with_dv(true)
                .with_sv(true)
                .with_swo(0)
                .with_correct_parity(),
        );
        hi.handle_transmit_complete().unwrap();
        assert!(
            hi.current_receive_buffer.is_some(),
            "frame unfinished, buffer retained for the next block"
        );

        // block 1: end of frame (EV, no SV)
        hi.handle_idle().unwrap();
        hi.interface.respond(
            &tail,
            crate::tc6_footer::ReceiveFooter::new()
                .with_dv(true)
                .with_ev(true)
                .with_ebo(tail.len() as u8 - 1)
                .with_correct_parity(),
        );
        hi.handle_transmit_complete().unwrap();

        let mut expected = head.clone();
        expected.extend_from_slice(&tail);

        let (rx, _tx) = lo.receive(Instant::ZERO).expect("rx + tx tokens");
        rx.consume(|buf| assert_eq!(buf, expected.as_slice()));
        assert!(
            lo.receive(Instant::ZERO).is_none(),
            "rx-ready queue drained"
        );
    }

    /// Once a single-block frame's chunk completes, `handle_transmit_complete` advances the
    /// cursor past the frame length and drops `current_transmit_buffer`, so it is not
    /// re-sent. The follow-up `handle_idle` inside `on_interrupt` then issues a *receive*
    /// poll (joined chunk, DV = 0), not another copy of the frame.
    #[test]
    fn transmit_frame_is_sent_once_then_driver_polls_for_receive() {
        setup!(hi, lo, SpyInterface::default(), NoopSignal);
        hi.sync_established = true; // data-path test: pretend configure() ran

        let payload = ethernet_like(64);
        {
            let tok = lo.transmit(Instant::ZERO).unwrap();
            tok.consume(64, |buf| buf.copy_from_slice(&payload));
        }
        hi.on_transmit_ready().unwrap();
        assert_eq!(hi.interface.chunks.len(), 1);
        assert!(!hi.interface.chunks[0].joined);

        hi.interface.respond(
            &[],
            crate::tc6_footer::ReceiveFooter::new().with_correct_parity(),
        );
        hi.on_interrupt().unwrap();

        assert!(
            hi.current_transmit_buffer.is_none(),
            "tx buffer freed once the whole frame is on the wire"
        );
        assert_eq!(hi.current_transmit_buffer_cursor, 0);
        // the only extra chunk is the receive poll, and it does not carry the frame again
        assert_eq!(hi.interface.chunks.len(), 2);
        assert!(hi.interface.chunks[1].joined);
        assert!(!tc6_header::TransmitHeader::from_raw(hi.interface.chunks[1].header).dv());
    }

    // --------------------------------------------------------------------------------------
    // high-priority side: multi-block transmit
    // --------------------------------------------------------------------------------------

    /// A frame longer than 64 bytes goes out as: block 0 via `begin_send_block` (SV, no EV),
    /// then one joined header+data chunk per following 64-byte block, the last with EV set.
    /// Stepped by hand (`handle_idle` / `handle_transmit_complete`) so the test controls
    /// exactly when each block is issued and completed.
    #[test]
    fn transmits_frame_spanning_two_blocks_as_block_then_joined_chunk() {
        setup!(hi, lo, SpyInterface::default(), NoopSignal);
        hi.sync_established = true; // data-path test: pretend configure() ran

        // 128 bytes == exactly two blocks
        let payload = ethernet_like(128);
        {
            let tok = lo.transmit(Instant::ZERO).expect("tx token");
            tok.consume(payload.len(), |buf| buf.copy_from_slice(&payload));
        }

        // block 0
        hi.on_transmit_ready().unwrap();
        {
            let c0 = &hi.interface.chunks[0];
            assert!(!c0.joined);
            let h = tc6_header::TransmitHeader::from_raw(c0.header);
            assert!(h.dnc() && h.dv() && h.sv());
            assert_eq!(h.swo(), 0);
            assert!(!h.ev(), "more blocks follow");
            assert!(h.parity_ok());
            assert_eq!(&c0.data[..], &payload[..64]);
        }
        hi.interface.respond(
            &[],
            crate::tc6_footer::ReceiveFooter::new().with_correct_parity(),
        );
        hi.handle_transmit_complete().unwrap();
        assert_eq!(hi.current_transmit_buffer_cursor, 64);
        assert!(hi.current_transmit_buffer.is_some());
        // the 4 bytes about to be borrowed for the next header are saved first
        let saved = &payload[60..64];

        // block 1 (continuation, joined chunk)
        hi.handle_idle().unwrap();
        assert_eq!(
            &hi.scratch_header[..],
            saved,
            "overwritten frame bytes stashed"
        );
        {
            let c1 = hi.interface.last();
            assert!(c1.joined);
            let h = tc6_header::TransmitHeader::from_raw(c1.header);
            assert!(h.dnc() && h.dv());
            assert!(!h.sv(), "a continuation block is not a frame start");
            assert!(h.ev());
            assert_eq!(h.ebo(), 63);
            assert!(h.parity_ok());
            assert_eq!(&c1.data[..], &payload[64..128]);
        }
        hi.interface.respond(
            &[],
            crate::tc6_footer::ReceiveFooter::new().with_correct_parity(),
        );
        hi.handle_transmit_complete().unwrap();

        assert!(
            hi.current_transmit_buffer.is_none(),
            "frame fully sent, buffer freed"
        );
        assert_eq!(hi.current_transmit_buffer_cursor, 0);

        // the two chunk payloads reassemble into the original frame
        let mut sent = Vec::new();
        sent.extend_from_slice(&hi.interface.chunks[0].data);
        sent.extend_from_slice(&hi.interface.chunks[1].data);
        assert_eq!(sent, payload);
    }

    /// Final block is partial (frame length not a multiple of 64). Per TC6 the chunk is
    /// still a full fixed-size block; EBO marks the last valid byte and the chip ignores
    /// the rest. The driver hands the interface `[cursor-4, cursor+64)`, whose tail sits in
    /// not-yet-written `FrameBuffer` capacity here - fine for real hardware/DMA, and the
    /// fake device drops everything past EBO.
    #[test]
    fn transmits_frame_with_partial_final_block_sets_ebo() {
        setup!(hi, lo, SpyInterface::default(), NoopSignal);
        hi.sync_established = true; // data-path test: pretend configure() ran

        let payload = ethernet_like(100); // 64 + 36
        {
            let tok = lo.transmit(Instant::ZERO).expect("tx token");
            tok.consume(payload.len(), |buf| buf.copy_from_slice(&payload));
        }

        // block 0
        hi.on_transmit_ready().unwrap();
        hi.interface.respond(
            &[],
            crate::tc6_footer::ReceiveFooter::new().with_correct_parity(),
        );
        hi.handle_transmit_complete().unwrap();

        // block 1 (partial: 36 bytes)
        hi.handle_idle().unwrap();
        let c1 = hi.interface.last();
        assert!(c1.joined);
        let h = tc6_header::TransmitHeader::from_raw(c1.header);
        assert!(h.dnc() && h.dv() && !h.sv() && h.ev());
        assert_eq!(h.ebo(), 35, "last valid byte is at payload offset 35");
        assert!(h.parity_ok());
        assert_eq!(&c1.data[..36], &payload[64..100]);

        hi.interface.respond(
            &[],
            crate::tc6_footer::ReceiveFooter::new().with_correct_parity(),
        );
        hi.handle_transmit_complete().unwrap();
        assert!(hi.current_transmit_buffer.is_none());
    }

    #[test]
    fn round_trip_multi_block_transmit_through_fake_tc6() {
        use crate::embeded_io_implementation::EmbeddedHalImplementation;
        use fake_tc6::fake_tc6_device::FakeTc6SpiDevice;

        let iface = EmbeddedHalImplementation::new(FakeTc6SpiDevice::new());
        setup!(hi, lo, iface, NoopSignal);
        hi.configure(&Config::default()).expect("bring-up");

        let payload = ethernet_like(192); // three blocks
        {
            let tok = lo.transmit(Instant::ZERO).expect("tx token");
            tok.consume(payload.len(), |buf| buf.copy_from_slice(&payload));
        }

        // Step three blocks by hand. `on_interrupt` can't be used here: this interface is
        // blocking so `transfer_complete()` is always true, and the loop only exits on the
        // `transfer_complete() == false` branch a real DMA interface would hit while a
        // transfer is in flight.
        for _ in 0..3 {
            hi.handle_idle().unwrap();
            hi.handle_transmit_complete().unwrap();
        }

        assert!(hi.current_transmit_buffer.is_none());
        let got = hi
            .interface
            .inner_mut()
            .device
            .pop_transmit_frame()
            .expect("fake device reassembled the frame");
        assert_eq!(got, payload);
    }

    /// Black-box round trip: no driver internals are touched. A frame is seeded on the
    /// MAC-PHY (before it is handed to the driver), the IRQ handler is pumped via the public
    /// `on_interrupt`, and the frame is collected through the public `smoltcp::phy::Device`
    /// `receive`. A frame is also pushed the other way with `transmit` so both directions run
    /// through `on_interrupt` (the outgoing side isn't asserted here - that would need a peek
    /// at the interface).
    #[test]
    fn black_box_round_trip_via_on_interrupt_and_device_receive() {
        use fake_tc6::fake_tc6_device::FakeTc6SpiDevice;

        let rx_frame: Vec<u8> = (0..128u32).map(|i| (i as u8) ^ 0x5A).collect();

        setup!(hi, lo, PollingFake::new(FakeTc6SpiDevice::new()), NoopSignal);
        hi.configure(&Config::default()).expect("bring-up");
        // seed the MAC-PHY with a frame to receive (after bring-up so the control traffic
        // doesn't run over it)
        hi.interface
            .inner
            .inner_mut()
            .device
            .enqueue_receive_frame(rx_frame.clone());

        // queue a frame to go out as well
        {
            let tok = lo.transmit(Instant::ZERO).expect("tx token");
            tok.consume(64, |buf| buf.copy_from_slice(&ethernet_like(64)));
        }

        // pump the IRQ handler; each call issues/finishes at most one chunk then returns
        for _ in 0..16 {
            hi.on_interrupt().unwrap();
        }

        // collect the received frame through the public Device API
        let (rx, _tx) = lo
            .receive(Instant::ZERO)
            .expect("driver surfaced the received frame");
        rx.consume(|buf| assert_eq!(buf, rx_frame.as_slice()));
        assert!(
            lo.receive(Instant::ZERO).is_none(),
            "only the one frame was received"
        );
    }

    // --------------------------------------------------------------------------------------
    // high-priority side: command mode (register read / write, TC6 spec Section 7.4)
    // --------------------------------------------------------------------------------------

    /// A single-register read builds a control command header (DNC = 0, WNR = 0, LEN = 0) and
    /// hands the MAC-PHY-echoed value back through `take_control_result`.
    #[test]
    fn register_read_single_builds_header_and_returns_value() {
        setup!(hi, _lo, SpyInterface::default(), NoopSignal);

        hi.begin_register_read(0, 0x0002, 1).expect("accepted");
        assert!(hi.control_busy());
        assert!(matches!(
            hi.driver_state,
            TC6DriverState::WaitingForControlComplete
        ));

        // header on the wire
        let sent = tc6_header::ControlHeader::from_bytes(
            hi.interface.last_control().tx[0..4].try_into().unwrap(),
        );
        assert!(!sent.dnc(), "DNC = 0 marks a control command");
        assert!(!sent.wnr(), "a read");
        assert_eq!(sent.mms(), 0);
        assert_eq!(sent.addr(), 0x0002);
        assert_eq!(sent.len(), 0, "LEN is (register count - 1)");
        assert!(sent.parity_ok());
        assert_eq!(
            hi.interface.last_control().tx.len(),
            4 * (1 + 2),
            "header + 1 word + trailing don't-care"
        );

        hi.interface.respond_control_read(&[0x1234_5678]);
        hi.handle_control_complete().unwrap();
        assert!(matches!(hi.driver_state, TC6DriverState::Idle));
        assert!(!hi.control_busy());

        let res = hi.take_control_result().expect("a result is waiting");
        assert!(!res.write);
        assert!(!res.header_bad);
        assert_eq!(res.values(), &[0x1234_5678]);
        assert_eq!(res.echoed_header.addr(), 0x0002);
        assert!(hi.take_control_result().is_none(), "result taken once");
    }

    /// A write sets WNR = 1, puts the data words after the header, and the echoed write data
    /// comes back for the host to verify (spec Section 7.4.4).
    #[test]
    fn register_write_puts_data_after_header_and_echoes_it() {
        setup!(hi, _lo, SpyInterface::default(), NoopSignal);

        hi.begin_register_write(0, 0x0004, &[0xDEAD_BEEF]).expect("accepted");

        let txn = hi.interface.last_control();
        let sent = tc6_header::ControlHeader::from_bytes(txn.tx[0..4].try_into().unwrap());
        assert!(!sent.dnc());
        assert!(sent.wnr(), "a write");
        assert_eq!(sent.addr(), 0x0004);
        assert_eq!(sent.len(), 0);
        assert!(sent.parity_ok());
        assert_eq!(&txn.tx[4..8], &0xDEAD_BEEFu32.to_be_bytes(), "data word follows header");

        hi.interface.respond_control_write();
        hi.handle_control_complete().unwrap();

        let res = hi.take_control_result().expect("result");
        assert!(res.write);
        assert!(!res.header_bad);
        assert_eq!(res.values(), &[0xDEAD_BEEF], "echoed write data");
        assert!(res.echoed_header.wnr());
    }

    /// Reading several consecutive registers in one transaction: LEN = count - 1, and every
    /// value word from the MISO stream is returned in order.
    #[test]
    fn register_read_multiple_consecutive() {
        setup!(hi, _lo, SpyInterface::default(), NoopSignal);

        hi.begin_register_read(0, 0x0000, 3).expect("accepted");
        let sent = tc6_header::ControlHeader::from_bytes(
            hi.interface.last_control().tx[0..4].try_into().unwrap(),
        );
        assert_eq!(sent.len(), 2, "3 registers -> LEN = 2");
        assert_eq!(hi.interface.last_control().tx.len(), 4 * (3 + 2));

        hi.interface.respond_control_read(&[0x0000_0011, 0x0000_0011, 0xAABB_CCDD]);
        hi.handle_control_complete().unwrap();

        let res = hi.take_control_result().unwrap();
        assert_eq!(res.values(), &[0x0000_0011, 0x0000_0011, 0xAABB_CCDD]);
    }

    /// If the MAC-PHY flags the command header as bad (it streams `0xC0000001` where the echoed
    /// header would be), the result carries `header_bad` and no register values.
    #[test]
    fn control_header_bad_is_surfaced_and_yields_no_values() {
        setup!(hi, _lo, SpyInterface::default(), NoopSignal);

        hi.begin_register_read(0, 0x0002, 1).expect("accepted");
        hi.interface.respond_control_bad_header();
        hi.handle_control_complete().unwrap();

        let res = hi.take_control_result().expect("result");
        assert!(res.header_bad);
        assert!(res.values().is_empty());
        assert!(matches!(hi.driver_state, TC6DriverState::Idle));
    }

    /// While a transaction is pending/in flight, another request is rejected with `Busy`
    /// rather than clobbering the first.
    #[test]
    fn second_control_request_while_busy_is_rejected() {
        setup!(hi, _lo, SpyInterface::default(), NoopSignal);

        hi.begin_register_read(0, 0x0002, 1).expect("first accepted");
        assert!(matches!(
            hi.begin_register_read(0, 0x0003, 1),
            Err(ControlError::Busy)
        ));
        assert!(matches!(
            hi.begin_register_write(0, 0x0004, &[0]),
            Err(ControlError::Busy)
        ));

        // once it completes, the interface is free again
        hi.interface.respond_control_read(&[0]);
        hi.handle_control_complete().unwrap();
        let _ = hi.take_control_result();
        assert!(hi.begin_register_read(0, 0x0003, 1).is_ok());
    }

    #[test]
    fn control_rejects_bad_register_counts() {
        setup!(hi, _lo, SpyInterface::default(), NoopSignal);

        assert!(matches!(
            hi.begin_register_read(0, 0, 0),
            Err(ControlError::UnsupportedCount)
        ));
        assert!(matches!(
            hi.begin_register_read(0, 0, CONTROL_MAX_REGS as u8 + 1),
            Err(ControlError::UnsupportedCount)
        ));
        assert!(matches!(
            hi.begin_register_write(0, 0, &[]),
            Err(ControlError::UnsupportedCount)
        ));
        assert!(matches!(
            hi.begin_register_write(0, 0, &[0u32; CONTROL_MAX_REGS + 1]),
            Err(ControlError::UnsupportedCount)
        ));
    }

    /// Driven purely through the public `on_interrupt` pump: the transaction looks in-flight
    /// for one poll (SpyInterface's completion flag is edge-triggered), then completes.
    #[test]
    fn register_read_completes_through_on_interrupt() {
        setup!(hi, _lo, SpyInterface::default(), NoopSignal);

        hi.begin_register_read(0, 0x0002, 1).expect("accepted");

        // transfer not done yet -> loop takes its `transfer_complete() == false` exit
        hi.on_interrupt().unwrap();
        assert!(hi.take_control_result().is_none());
        assert!(hi.control_busy());

        hi.interface.respond_control_read(&[0x0000_00A5]);
        hi.on_interrupt().unwrap();

        let res = hi.take_control_result().expect("result after completion");
        assert_eq!(res.values(), &[0x0000_00A5]);
    }

    /// A queued register read is serviced ahead of a waiting transmit frame.
    #[test]
    fn pending_control_is_serviced_before_data() {
        setup!(hi, lo, SpyInterface::default(), NoopSignal);
        hi.sync_established = true; // data-path test: pretend configure() ran

        // a frame is waiting to go out...
        {
            let tok = lo.transmit(Instant::ZERO).expect("tx token");
            tok.consume(64, |b| b.copy_from_slice(&ethernet_like(64)));
        }
        // ...but a control read is queued first
        hi.begin_register_read(0, 0x0004, 1).expect("accepted");
        assert!(matches!(
            hi.driver_state,
            TC6DriverState::WaitingForControlComplete
        ));
        assert_eq!(hi.interface.chunks.len(), 0, "no data chunk yet");
        assert_eq!(hi.interface.control_txns.len(), 1);

        hi.interface.respond_control_read(&[0]);
        hi.handle_control_complete().unwrap();
        let _ = hi.take_control_result();

        // now the transmit frame goes out
        hi.on_transmit_ready().unwrap();
        assert_eq!(hi.interface.chunks.len(), 1);
        assert!(tc6_header::TransmitHeader::from_raw(hi.interface.chunks[0].header).dv());
    }

    // --------------------------------------------------------------------------------------
    // blocking bring-up (configure / split_configured)
    // --------------------------------------------------------------------------------------

    /// The happy path: `configure` waits out RESETC, writes CONFIG0 with SYNC = 1 and a
    /// 64-byte payload, programs IMASK0, and clears STATUS0.
    #[test]
    fn configure_runs_the_full_bring_up_sequence() {
        setup!(hi, _lo, RegDouble::new(), NoopSignal);
        assert!(!hi.is_configured());

        hi.configure(&Config::default()).expect("bring-up ok");

        assert!(hi.is_configured());
        let c0 = registers::Config0::from_raw(hi.interface.reg(0, 0x0004));
        assert!(c0.sync(), "SYNC written");
        assert_eq!(c0.cps(), 6, "64-byte chunk payload forced");
        assert_eq!(hi.interface.reg(0, 0x0008) & 0x40, 0, "RESETC cleared");
        assert_eq!(
            hi.interface.reg(0, 0x000C),
            registers::IMask0::RESET_VALUE.raw(),
            "IMASK0 programmed"
        );
        assert!(matches!(hi.driver_state, TC6DriverState::Idle));
    }

    #[test]
    fn configure_applies_optional_features() {
        setup!(hi, _lo, RegDouble::new(), NoopSignal);
        let cfg = Config {
            tx_credit_threshold: TxCreditThreshold::Eight,
            cut_through_rx: true,
            timestamp: Some(TimestampSize::Bits64),
            ..Config::default()
        };
        hi.configure(&cfg).expect("bring-up ok");

        let c0 = registers::Config0::from_raw(hi.interface.reg(0, 0x0004));
        assert!(c0.rxcte());
        assert!(!c0.txcte());
        assert_eq!(c0.txcthresh(), 0b10);
        assert!(c0.ftse() && c0.ftss());
        assert!(c0.sync());
    }

    #[test]
    fn configure_rejects_cut_through_when_stdcap_lacks_it() {
        setup!(hi, _lo, RegDouble::new(), NoopSignal);
        hi.interface.regs.insert((0, 0x0002), 0x0000_0006); // STDCAP: MINCPS only, no CTC

        let cfg = Config {
            cut_through_rx: true,
            ..Config::default()
        };
        assert!(matches!(
            hi.configure(&cfg),
            Err(ConfigError::Unsupported("cut-through"))
        ));
        assert!(!hi.is_configured());
    }

    #[test]
    fn configure_times_out_waiting_for_reset_complete() {
        setup!(hi, _lo, RegDouble::new(), NoopSignal);
        hi.interface.regs.insert((0, 0x0008), 0x0000_0000); // RESETC never set

        let cfg = Config {
            reset_poll_limit: 3,
            ..Config::default()
        };
        assert!(matches!(
            hi.configure(&cfg),
            Err(ConfigError::ResetNotComplete)
        ));
    }

    #[test]
    fn configure_rejects_implausible_version() {
        setup!(hi, _lo, RegDouble::new(), NoopSignal);
        hi.interface.regs.insert((0, 0x0000), 0);

        assert!(matches!(
            hi.configure(&Config::default()),
            Err(ConfigError::BadVersion { major: 0, .. })
        ));
    }

    #[test]
    fn configure_detects_sync_not_sticking() {
        setup!(hi, _lo, RegDouble::new(), NoopSignal);
        hi.interface.ignore_sync_write = true;

        assert!(matches!(
            hi.configure(&Config::default()),
            Err(ConfigError::SyncFailed)
        ));
        assert!(!hi.is_configured());
    }

    #[test]
    fn configure_times_out_on_a_stuck_transfer_and_unwedges() {
        setup!(hi, _lo, RegDouble::new(), NoopSignal);
        hi.interface.never_complete = true;

        let cfg = Config {
            transfer_poll_limit: 5,
            ..Config::default()
        };
        assert!(matches!(hi.configure(&cfg), Err(ConfigError::Timeout)));
        assert!(
            matches!(hi.driver_state, TC6DriverState::Idle),
            "state machine returned to idle after a timeout"
        );
        assert!(!hi.control_busy());
    }

    #[test]
    fn split_configured_yields_a_ready_driver() {
        setup_configured!(hi, _lo, RegDouble::new(), NoopSignal, &Config::default());
        assert!(hi.is_configured());
        assert!(registers::Config0::from_raw(hi.interface.reg(0, 0x0004)).sync());
    }

    #[test]
    fn configure_sets_up_plca_when_requested() {
        setup!(hi, _lo, RegDouble::new(), NoopSignal);
        let cfg = Config {
            plca: Some(PlcaConfig::coordinator(8)),
            ..Config::default()
        };
        hi.configure(&cfg).expect("bring-up ok");

        let c1 = registers::PlcaCtrl1::from_raw(hi.interface.reg(4, 0xCA02));
        assert_eq!(c1.id(), 0, "coordinator is node 0");
        assert_eq!(c1.node_count(), 8);
        assert!(registers::PlcaCtrl0::from_raw(hi.interface.reg(4, 0xCA01)).enable());
    }

    #[test]
    fn configure_without_plca_touches_no_mms4_registers() {
        setup!(hi, _lo, RegDouble::new(), NoopSignal);
        hi.configure(&Config::default()).expect("bring-up ok");
        assert!(
            !hi.interface.writes.iter().any(|&(mms, _, _)| mms == 4),
            "PLCA registers left untouched when cfg.plca is None"
        );
    }

    /// End to end through the real wire-format code and `fake_tc6`'s control-register model:
    /// `configure` drives actual control transactions and the chip's registers end up set.
    #[test]
    fn configure_through_fake_tc6() {
        use crate::embeded_io_implementation::EmbeddedHalImplementation;
        use fake_tc6::fake_tc6_device::FakeTc6SpiDevice;

        let iface = EmbeddedHalImplementation::new(FakeTc6SpiDevice::new());
        setup!(hi, _lo, iface, NoopSignal);

        hi.configure(&Config::default())
            .expect("bring-up through the real wire format");
        assert!(hi.is_configured());

        let dev = &hi.interface.inner_mut().device;
        let c0 = registers::Config0::from_raw(dev.read_reg(0, 0x0004));
        assert!(c0.sync(), "CONFIG0.SYNC set on the chip");
        assert_eq!(c0.cps(), 6);
        assert_eq!(dev.read_reg(0, 0x0008) & 0x40, 0, "RESETC cleared");
        assert_eq!(
            dev.read_reg(0, 0x000C),
            registers::IMask0::RESET_VALUE.raw(),
            "IMASK0 programmed"
        );
    }

    /// End to end: PLCA node setup reaches the PHY's MMS 4 registers through the real wire path.
    #[test]
    fn configure_with_plca_through_fake_tc6() {
        use crate::embeded_io_implementation::EmbeddedHalImplementation;
        use fake_tc6::fake_tc6_device::FakeTc6SpiDevice;

        let iface = EmbeddedHalImplementation::new(FakeTc6SpiDevice::new());
        setup!(hi, _lo, iface, NoopSignal);

        let cfg = Config {
            plca: Some(PlcaConfig::follower(3)),
            ..Config::default()
        };
        hi.configure(&cfg).expect("bring-up with PLCA");

        let dev = &hi.interface.inner_mut().device;
        assert_eq!(
            registers::PlcaCtrl1::from_raw(dev.read_reg(4, 0xCA02)).id(),
            3
        );
        assert!(registers::PlcaCtrl0::from_raw(dev.read_reg(4, 0xCA01)).enable());
    }

    /// End to end: when `fake_tc6` streams the `0xC0000001` header-error code (spec 7.5.1),
    /// the driver surfaces `header_bad` with no register values.
    #[test]
    fn control_header_parity_error_surfaced_through_fake_tc6() {
        use crate::embeded_io_implementation::EmbeddedHalImplementation;
        use fake_tc6::fake_tc6_device::FakeTc6SpiDevice;

        let mut spi = FakeTc6SpiDevice::new();
        spi.device.force_bad_control_header = true;
        setup!(hi, _lo, EmbeddedHalImplementation::new(spi), NoopSignal);

        hi.begin_register_read(0, 0x0002, 1).expect("queued");
        // EmbeddedHalImplementation runs the transfer synchronously; finish it by hand
        hi.handle_control_complete().unwrap();

        let res = hi.take_control_result().expect("result");
        assert!(res.header_bad);
        assert!(res.values().is_empty());
        // the chip latched STATUS0.HDRE (bit 5)
        assert_ne!(
            hi.interface.inner_mut().device.read_reg(0, 0x0008) & (1 << 5),
            0
        );
    }

    /// A receive footer with SYNC = 0 after bring-up means the chip reset itself; the driver
    /// drops its "configured" flag so the host knows to re-run `configure`.
    #[test]
    fn sync_loss_in_a_footer_clears_configured() {
        setup!(hi, _lo, SpyInterface::default(), NoopSignal);
        hi.sync_established = true;

        // a receive-only poll that comes back configured -> flag stays set
        hi.handle_idle().unwrap();
        hi.interface.respond(
            &[],
            crate::tc6_footer::ReceiveFooter::new()
                .with_sync(true)
                .with_correct_parity(),
        );
        hi.handle_transmit_complete().unwrap();
        assert!(hi.is_configured());

        // ...then one that reports SYNC = 0 -> flag cleared
        hi.interface.footer_sync = false;
        hi.handle_idle().unwrap();
        hi.interface.respond(
            &[],
            crate::tc6_footer::ReceiveFooter::new().with_correct_parity(),
        );
        hi.handle_transmit_complete().unwrap();
        assert!(!hi.is_configured());
    }
}
