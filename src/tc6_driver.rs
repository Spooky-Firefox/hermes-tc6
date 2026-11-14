//! The split TC6 MAC-PHY driver: [`TC6HighPrio`] services the chip over SPI, [`TC6LowPrio`]
//! implements `smoltcp::phy::Device`, and [`split`] / [`split_configured`] build the pair.
//!
//! The code is split by concern into child modules, which all extend [`TC6HighPrio`]:
//! `data_path` (data chunks, `on_interrupt`), `control` (register access), `bring_up`
//! (`Config`, `configure`, extended status) and `device` (the smoltcp `Device` impl).

use crate::tc6_footer::ReceiveFooter;
use crate::tc6_interface::Tc6Interface;
use crate::{frame_buffer::FrameBuffer, registers, tc6_footer, tc6_header};
use embedded_dma::{ReadBuffer, WriteBuffer};
use heapless::pool::boxed::{Box, BoxPool};
use heapless::spsc::{Consumer, Producer, Queue};
use smoltcp::phy::DeviceCapabilities;

mod bring_up;
mod control;
mod data_path;
mod device;

pub use bring_up::{Config, ConfigError, PlcaConfig, TimestampSize, TxCreditThreshold};
pub use control::{ControlError, ControlResult, RegisterError};
use control::{ControlInFlight, DEFAULT_TRANSFER_POLL_LIMIT, PendingControl};
pub use device::{TC6DriverRxToken, TC6DriverTxToken};

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

/// Ethernet L2 MTU reported to smoltcp: 14-byte header + 1500-byte payload, no FCS (the MAC-PHY
/// appends/strips it, spec 7.3).
const ETHERNET_MTU: usize = 1514;

/// Minimum `FrameBuffer` capacity `M` the driver is safe with. A receive frame accumulates up to
/// a full 1518-byte Ethernet frame (1514 + FCS, in case the MAC is configured to pass it), and
/// on every chunk the driver hands the MAC a 64-byte payload + 4-byte footer landing zone
/// *past* the data written so far (`FrameBuffer::get_mut_at_end(68)`). So `M` must cover
/// `1518 + 64 + 4`, rounded up for headroom.
const MIN_FRAME_BUFFER_CAPACITY: usize = 1600;

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
    control_tx_buffer: [u8; CONTROL_BUF_LEN], // MOSI: header + write data + trailing don't-care
    control_rx_buffer: [u8; CONTROL_BUF_LEN], // MISO: don't-care + echoed header + data
    // Set by `configure`; cleared if a receive footer ever reports SYNC = 0 (the chip reset
    // itself and lost its configuration, spec Section 7.7 - the host must re-run `configure`).
    sync_established: bool,
    // Most recent `RCA` (receive chunks available) copied out of a data-chunk footer: how many
    // more receive chunks the MAC-PHY has buffered right now (TC6 spec Section 7.7). Drives
    // `on_interrupt`'s drain loop - keep issuing chunks while this is > 0, stop at 0 and wait for
    // the next interrupt. Refreshed from every footer; stale-low at worst (new frames since the
    // last footer just get picked up on the next interrupt).
    rx_chunks_available: u8,
    // Most recent `TXC` (transmit credits) copied out of a data-chunk footer: how many transmit
    // chunks the MAC-PHY can still accept before `STATUS0.TXBOE`. `on_interrupt` will not keep
    // draining for transmit while this is 0. Between footers it can only be stale-low (the chip's
    // transmit buffer only drains while the host is not sending), so gating on it never causes an
    // overflow, only a slightly delayed send.
    tx_credits: u8,
    // Latched when a data-chunk footer reports `EXST = 1` (a STATUS0/STATUS1 bit is set and
    // unmasked, spec Section 7.3.7 / 7.7). Cleared by `service_extended_status`, which reads
    // STATUS0 and write-1-to-clears it, and by `configure`.
    extended_status_pending: bool,
    // Poll budget for the blocking register functions: `Config::transfer_poll_limit` from the
    // last `configure`, or the default before that.
    transfer_poll_limit: u32,
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
    // Post-monomorphization checks: too-small buffers / queues would only surface as a runtime
    // panic (or worse) deep in the data path.
    const {
        assert!(
            M >= MIN_FRAME_BUFFER_CAPACITY,
            "FrameBuffer capacity M is too small - see MIN_FRAME_BUFFER_CAPACITY"
        );
        assert!(
            NQ >= 2,
            "the frame queues need NQ >= 2 (usable capacity is NQ - 1)"
        );
    }

    debug!(
        "tc6: split() - M={}, NQ={}, driver starts unconfigured",
        M, NQ
    );

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
            rx_chunks_available: 0,
            tx_credits: 0,
            extended_status_pending: false,
            transfer_poll_limit: DEFAULT_TRANSFER_POLL_LIMIT,
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

#[cfg(test)]
mod tests;
