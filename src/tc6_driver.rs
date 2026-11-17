//! The split TC6 MAC-PHY driver: [`TC6HighPrio`] services the chip over SPI, [`TC6LowPrio`]
//! implements `smoltcp::phy::Device`, and [`split`] / [`split_configured`] build the pair.
//!
//! The code is split by concern into child modules, which all extend [`TC6HighPrio`]:
//! `data_path` (data chunks, `on_interrupt`), `control` (register access), `bring_up`
//! (`Config`, `configure`, extended status) and `device` (the smoltcp `Device` impl).

use crate::frame_buffer::{FrameBuffer, META_LEN};
use crate::tc6_footer::ReceiveFooter;
use crate::tc6_interface::Tc6Interface;
use crate::{registers, tc6_footer, tc6_header};
use bbqueue::BBQueue;
use bbqueue::prod_cons::framed::{FramedConsumer, FramedGrantR, FramedGrantW, FramedProducer};
use bbqueue::traits::coordination::cas::AtomicCoord;
use bbqueue::traits::notifier::polling::Polling;
use bbqueue::traits::storage::Inline;
use embedded_dma::{ReadBuffer, WriteBuffer};
use smoltcp::phy::DeviceCapabilities;

mod bring_up;
mod control;
mod data_path;
mod device;

pub use bring_up::{Config, ConfigError, PlcaConfig, TimestampSize, TxCreditThreshold};
pub use control::{ControlError, ControlResult, RegisterError};
use control::{ControlInFlight, DEFAULT_TRANSFER_POLL_LIMIT, PendingControl};
pub use device::{TC6DriverRxToken, TC6DriverTxToken};

/// One direction's frame ring: a `bbqueue` bip buffer of `N` bytes holding variable-length
/// frame records (see [`crate::frame_buffer`] for the record layout).
type Ring<const N: usize> = BBQueue<Inline<N>, AtomicCoord, Polling>;
type RingProducer<'s, const N: usize> = FramedProducer<&'s Ring<N>>;
type RingConsumer<'s, const N: usize> = FramedConsumer<&'s Ring<N>>;
type RingGrantW<'s, const N: usize> = FramedGrantW<&'s Ring<N>>;
type RingGrantR<'s, const N: usize> = FramedGrantR<&'s Ring<N>>;

/// bbqueue's per-record length prefix (`u16`).
const RING_HEADER_LEN: usize = 2;

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

/// Largest receive frame the buffers are sized for: 1514 + 4-byte FCS (received frames include
/// it) + 4-byte 802.1Q VLAN tag. A longer frame from the wire is dropped, not a panic (see
/// `handle_idle`).
const MAX_RX_FRAME: usize = 1522;

/// Largest start offset of a frame inside the first chunk's payload: `SWO` is 4 bits, counted in
/// 32-bit words (spec 7.3.7), so `15 * 4`.
const MAX_RX_START_OFFSET: usize = 60;

/// Bytes the driver hands the MAC-PHY per chunk *past* the frame data received so far
/// (`FrameBuffer::get_mut_at_end`): 64 payload + 4 footer.
const RX_CHUNK_WINDOW: usize = 68;

/// Minimum `FrameBuffer` capacity `M` that receives every frame up to [`MAX_RX_FRAME`] bytes.
///
/// Worst case: the frame starts at [`MAX_RX_START_OFFSET`] into the buffer, all but its last
/// byte has arrived, and the last chunk still needs a full [`RX_CHUNK_WINDOW`]:
/// `60 + (1522 - 1) + 68 = 1649`. (Chunks land 64-byte aligned, so the true worst case is a bit
/// lower, but this bound needs no alignment argument.) Smaller `M` would drop legal frames.
const MIN_FRAME_BUFFER_CAPACITY: usize = MAX_RX_START_OFFSET + (MAX_RX_FRAME - 1) + RX_CHUNK_WINDOW;

/// Body size of every receive grant: meta header + the worst-case frame region. The grant is
/// committed down to `META_LEN + start + len` once the frame is complete.
const RX_GRANT: usize = META_LEN + MIN_FRAME_BUFFER_CAPACITY;

/// Largest transmit frame region: the MTU rounded up to whole 64-byte chunks, so the transmit
/// SPI windows (which always span a full chunk) stay inside the record.
const TX_REGION_MAX: usize = ETHERNET_MTU.next_multiple_of(64);

/// Body size of every transmit grant. Committed down to `META_LEN + round_up(len, 64)`.
const TX_GRANT: usize = META_LEN + TX_REGION_MAX;

// Grant sizes go through bbqueue's `u16` frame header.
const _: () = assert!(RX_GRANT <= u16::MAX as usize);
const _: () = assert!(TX_GRANT <= u16::MAX as usize);

/// Ring bytes one worst-case record takes (bbqueue length prefix + body).
const RX_RECORD_MAX: usize = RING_HEADER_LEN + RX_GRANT;
const TX_RECORD_MAX: usize = RING_HEADER_LEN + TX_GRANT;

/// Smallest receive ring [`HermesStorage`] accepts: `2 * RX_RECORD_MAX + 1` bytes.
///
/// A bip buffer only hands out *contiguous* space, and bbqueue does not rewind its pointers
/// when the ring empties. With a ring smaller than two worst-case records, an empty ring whose
/// pointers sit near the middle has no contiguous gap big enough on either side, and the grant
/// would fail forever. Two records plus one byte always leaves room on one side. Bigger rings
/// hold more frames: small frames only take their actual size.
pub const MIN_RX_RING: usize = 2 * RX_RECORD_MAX + 1;

/// Smallest transmit ring [`HermesStorage`] accepts. Same reasoning as [`MIN_RX_RING`].
pub const MIN_TX_RING: usize = 2 * TX_RECORD_MAX + 1;

/// Frame storage for one driver instance: a receive ring of `RX_N` bytes and a transmit ring of
/// `TX_N` bytes.
///
/// Pass it to [`split`] / [`split_configured`]; the driver halves borrow it for as long as they
/// live. It is large, so put it in a `static` (e.g. RTIC `#[init(local = [...])]`, a
/// `StaticCell`, or a `static mut` guarded by a taken flag), not on the stack.
///
/// Frames take only as much ring space as they need (plus 6 bytes of headers and, on transmit,
/// padding to a 64-byte multiple), so a 60-byte ARP frame no longer occupies a full-MTU buffer.
/// Each side keeps one worst-case grant open while it works, though: the receive side always
/// holds `RX_GRANT` bytes for the frame it is receiving.
pub struct HermesStorage<const RX_N: usize, const TX_N: usize> {
    rx: Ring<RX_N>,
    tx: Ring<TX_N>,
}

impl<const RX_N: usize, const TX_N: usize> HermesStorage<RX_N, TX_N> {
    pub const fn new() -> Self {
        Self {
            rx: Ring::new(),
            tx: Ring::new(),
        }
    }
}

impl<const RX_N: usize, const TX_N: usize> Default for HermesStorage<RX_N, TX_N> {
    fn default() -> Self {
        Self::new()
    }
}

/// Implemented by the application to wake the high-priority side when the low-priority side has
/// queued a frame to transmit (e.g. pend a software interrupt, or spawn an RTIC task that ends up
/// calling `TC6HighPrio::on_transmit_ready`). Without this, a pure-transmit workload would never
/// get serviced, since `on_interrupt` only fires on chip-initiated events.
pub trait TransmitSignal {
    fn signal(&mut self);
}

/// Owned by the high-priority context (e.g. the TC6 IRQ task): services the chip over SPI.
pub struct TC6HighPrio<'s, T, const RX_N: usize, const TX_N: usize>
where
    T: Tc6Interface,
{
    interface: T,
    // the frame currently being transmitted, if any (a read grant on the transmit ring)
    current_transmit_buffer: Option<FrameBuffer<RingGrantR<'s, TX_N>>>,
    // the frame currently being received, if any (a worst-case write grant on the receive ring)
    current_receive_buffer: Option<FrameBuffer<RingGrantW<'s, RX_N>>>,
    // todo, do we need Option, can we use empty vec to indicate no data?
    extra_start_buffer: Option<heapless::Vec<u8, 64>>, // used to store the start of a receive frame if the start was written to the end of the previous framebuffer
    driver_state: TC6DriverState,
    current_transmit_buffer_cursor: usize, // keeps track of how much have been written to the tc6 interface, so we know what to transfer in the next block
    scratch_header: [u8; 4], // used for temporary storage of the header in the case where a block needs to be moved
    scratch_buffer: [u8; 68], // used for temporary storage of a block in the case where a block needs to be moved
    rx_producer: RingProducer<'s, RX_N>, // frames pulled off the wire, committed for the low-prio side
    tx_consumer: RingConsumer<'s, TX_N>, // frames built by the low-prio side, waiting to go out over SPI
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
pub struct TC6LowPrio<'s, S, const RX_N: usize, const TX_N: usize>
where
    S: TransmitSignal,
{
    rx_consumer: RingConsumer<'s, RX_N>,
    tx_producer: RingProducer<'s, TX_N>,
    transmit_signal: S, // poked whenever a frame is committed to `tx_producer`
}

/// Splits the driver into its high-priority (IRQ) and low-priority (smoltcp) halves.
///
/// Both halves borrow `storage` (see [`HermesStorage`]) for `'s`. Taking it by `&mut` means a
/// storage can back only one driver at a time.
///
/// The returned driver is **unconfigured**: [`TC6HighPrio::configure`] must succeed before any
/// frame data will move (the data path is inert while `SYNC = 0`). Prefer [`split_configured`]
/// unless you specifically need to defer bring-up (e.g. the SPI transport is not ready yet).
#[must_use = "the driver does nothing until TC6HighPrio::configure succeeds"]
pub fn split<'s, T, S, const RX_N: usize, const TX_N: usize>(
    interface: T,
    storage: &'s mut HermesStorage<RX_N, TX_N>,
    transmit_signal: S,
) -> (
    TC6HighPrio<'s, T, RX_N, TX_N>,
    TC6LowPrio<'s, S, RX_N, TX_N>,
)
where
    T: Tc6Interface,
    S: TransmitSignal,
{
    // Post-monomorphization checks: too-small rings would only surface at runtime, as grants
    // that can never succeed.
    const {
        assert!(
            RX_N >= MIN_RX_RING,
            "receive ring RX_N is too small - see MIN_RX_RING"
        );
        assert!(
            TX_N >= MIN_TX_RING,
            "transmit ring TX_N is too small - see MIN_TX_RING"
        );
    }

    debug!(
        "tc6: split() - RX_N={}, TX_N={}, driver starts unconfigured",
        RX_N, TX_N
    );

    // bbqueue does not enforce single-producer / single-consumer itself. Each of these four
    // handles is created exactly once, here, and the storage fields are private, so every ring
    // has one producer and one consumer for the whole borrow.
    let storage: &'s HermesStorage<RX_N, TX_N> = storage;
    let rx_producer = storage.rx.framed_producer();
    let rx_consumer = storage.rx.framed_consumer();
    let tx_producer = storage.tx.framed_producer();
    let tx_consumer = storage.tx.framed_consumer();

    (
        TC6HighPrio {
            interface,
            driver_state: TC6DriverState::Idle,
            current_transmit_buffer_cursor: 0,
            scratch_header: [0; 4],
            scratch_buffer: [0; 68],
            current_transmit_buffer: None,
            current_receive_buffer: None,
            rx_producer,
            tx_consumer,
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
            rx_consumer,
            tx_producer,
            transmit_signal,
        },
    )
}

/// Like [`split`], then run [`TC6HighPrio::configure`] on the high-priority half before handing
/// both back, so the caller can never obtain an unconfigured driver. On error the driver halves
/// are dropped; there is nothing useful to do with them.
#[allow(clippy::type_complexity)]
pub fn split_configured<'s, T, S, const RX_N: usize, const TX_N: usize>(
    interface: T,
    storage: &'s mut HermesStorage<RX_N, TX_N>,
    transmit_signal: S,
    config: &Config,
) -> Result<
    (
        TC6HighPrio<'s, T, RX_N, TX_N>,
        TC6LowPrio<'s, S, RX_N, TX_N>,
    ),
    ConfigError<T::Error>,
>
where
    T: Tc6Interface,
    S: TransmitSignal,
{
    let (mut hi, lo) = split(interface, storage, transmit_signal);
    hi.configure(config)?;
    Ok((hi, lo))
}

#[cfg(test)]
mod tests;
