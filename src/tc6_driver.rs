use crate::tc6_interface::Tc6Interface;
use core::mem::MaybeUninit;
use heapless::pool::boxed::{Box, BoxPool};
use heapless::spsc::{Consumer, Producer, Queue};

// NOTE: `box_pool!` singletons must name a concrete type, so `RxP`/`TxP` (and the `M` they were
// built with) have to be declared by the application, not by this generic driver. Pass the
// resulting pool types + `'static` queue storage into `split()` below.

/// A single frame's worth of storage; `start`/`length` mark the populated region within `buffer`.
/// `pub` because the application must name this type concretely in its own `box_pool!` call.
pub struct FrameBuffer<const M: usize> {
    start: usize,
    length: usize,
    buffer: [MaybeUninit<u8>; M],
}

enum TC6DriverMode {
    Idle,
    WaitingForTransmitComplete,
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
    driver_mode: TC6DriverMode,
    current_transmit_buffer_length: usize, // keeps track of how much have been written to the tc6 interface, so we know what to transfer in the next block
    scratch_header: [u8; 4], // used for temporary storage of the header in the case where a block needs to be moved
    scratch_buffer: [u8; 64], // used for temporary storage of a block in the case where a block needs to be moved
    rx_ready_producer: Producer<'static, Box<RxP>>, // frames pulled off the wire, handed to the low-prio side
    tx_ready_consumer: Consumer<'static, Box<TxP>>, // frames built by the low-prio side, waiting to go out over SPI
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
            driver_mode: TC6DriverMode::Idle,
            current_transmit_buffer_length: 0,
            scratch_header: [0; 4],
            scratch_buffer: [0; 64],
            rx_ready_producer,
            tx_ready_consumer,
        },
        TC6LowPrio {
            rx_ready_consumer,
            tx_ready_producer,
            transmit_signal,
        },
    )
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
        todo!()
    }

    // This should be called whenever the low-prio side may have queued a frame to transmit:
    // either from whatever `TransmitSignal::signal` triggers, or from periodic polling if the
    // caller would rather not wire up a signal at all.
    pub fn on_transmit_ready(&mut self) -> Result<(), T::Error> {
        if matches!(self.driver_mode, TC6DriverMode::Idle) {
            // still waiting for the previous transfer to complete, so we can't start a new one yet
            return Ok(());
        }
        todo!()
    }
}

pub struct TC6DriverRxToken<RxP: BoxPool> {
    frame: Box<RxP>,
}

impl<RxP, const M: usize> smoltcp::phy::RxToken for TC6DriverRxToken<RxP>
where
    RxP: BoxPool<Data = FrameBuffer<M>> + 'static,
{
    fn consume<R, F>(self, _f: F) -> R
    where
        F: FnOnce(&[u8]) -> R,
    {
        todo!()
    }

    fn meta(&self) -> smoltcp::phy::PacketMeta {
        smoltcp::phy::PacketMeta::default()
    }
}

pub struct TC6DriverTxToken<TxP: BoxPool> {
    frame: Box<TxP>,
}

impl<TxP, const M: usize> smoltcp::phy::TxToken for TC6DriverTxToken<TxP>
where
    TxP: BoxPool<Data = FrameBuffer<M>> + 'static,
{
    fn consume<R, F>(self, _len: usize, _f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        todo!()
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
        = TC6DriverTxToken<TxP>
    where
        Self: 'a;

    fn capabilities(&self) -> smoltcp::phy::DeviceCapabilities {
        todo!()
    }

    fn receive(
        &mut self,
        _timestamp: smoltcp::time::Instant,
    ) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        // only report a received frame if a tx buffer will also be available (see paired-token
        // note: smoltcp always wants both, so this doubles as the anti-deadlock guard)
        todo!()
    }

    fn transmit(&mut self, _timestamp: smoltcp::time::Instant) -> Option<Self::TxToken<'_>> {
        todo!()
    }
}
