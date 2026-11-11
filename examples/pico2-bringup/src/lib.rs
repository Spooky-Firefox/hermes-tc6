//! Board support shared by the bring-up binary (`src/main.rs`) and the on-target test suites
//! (`tests/`): one Raspberry Pi Pico 2 (RP2350) driving two LAN8651s, joined by a twisted pair.
//!
//! | LAN8651 | Chip A (SPI1) | Chip B (SPI0) | Notes |
//! |---------|---------------|---------------|-------|
//! | SCLK    | GPIO10        | GPIO2         | SPI SCK |
//! | SDI     | GPIO11        | GPIO3         | SPI TX (MOSI) |
//! | SDO     | GPIO12        | GPIO4         | SPI RX (MISO) |
//! | CS_N    | GPIO13        | GPIO5         | plain GPIO, driven by `ExclusiveDevice` |
//! | IRQ_N   | GPIO14        | GPIO6         | input, pull-up |
//! | RESET_N | GPIO15        | GPIO7         | output |

#![no_std]

use core::sync::atomic::{AtomicBool, Ordering};

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::spi::SpiDevice;
use embedded_hal_bus::spi::ExclusiveDevice;
use heapless::box_pool;
use heapless::pool::boxed::{Box, BoxBlock};
use heapless::spsc::Queue;
use rp235x_hal::{
    self as hal, Clock,
    fugit::{HertzU32, RateExtU32},
    gpio::{self, PinState, bank0},
    pac,
};

pub use hermes_tc6;
use hermes_tc6::embedded_io_implementation::EmbeddedHalImplementation;
use hermes_tc6::frame_buffer::FrameBuffer;
use hermes_tc6::tc6_driver::{self, ControlError, TC6HighPrio, TC6LowPrio, TransmitSignal};
use hermes_tc6::tc6_footer::ReceiveFooter;
use hermes_tc6::tc6_header::{ControlHeader, TransmitHeader};
use hermes_tc6::tc6_interface::Tc6Interface;

/// Tell the RP2350 boot ROM about our application. Lives here so every binary linking this
/// crate (the demo and each test binary) gets it.
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: hal::block::ImageDef = hal::block::ImageDef::secure_exe();

/// Pico 2 crystal.
pub const XTAL_FREQ_HZ: u32 = 12_000_000;
/// Default SPI clock. Low for logic-analyzer-friendly bring-up; the LAN8651 is rated for 25 MHz.
pub const SPI_HZ: u32 = 1_000_000;

/// Frame buffer capacity (>= the driver's 1600-byte minimum).
pub const M: usize = 1600;
/// SPSC queue depth (usable depth NQ - 1).
pub const NQ: usize = 4;
/// Frame buffers per pool.
pub const POOL_BLOCKS: usize = 4;

box_pool!(RxP: FrameBuffer<M>);
box_pool!(TxP: FrameBuffer<M>);

pub type SpiBus = hal::Spi<
    hal::spi::Enabled,
    pac::SPI1,
    (
        gpio::Pin<bank0::Gpio11, gpio::FunctionSpi, gpio::PullDown>,
        gpio::Pin<bank0::Gpio12, gpio::FunctionSpi, gpio::PullDown>,
        gpio::Pin<bank0::Gpio10, gpio::FunctionSpi, gpio::PullDown>,
    ),
>;
pub type CsPin = gpio::Pin<bank0::Gpio13, gpio::FunctionSioOutput, gpio::PullDown>;
pub type Timer = hal::Timer<hal::timer::CopyableTimer0>;
pub type Spi = ExclusiveDevice<SpiBus, CsPin, Timer>;
pub type Iface = EmbeddedHalImplementation<Spi>;
pub type SpiBusB = hal::Spi<
    hal::spi::Enabled,
    pac::SPI0,
    (
        gpio::Pin<bank0::Gpio3, gpio::FunctionSpi, gpio::PullDown>,
        gpio::Pin<bank0::Gpio4, gpio::FunctionSpi, gpio::PullDown>,
        gpio::Pin<bank0::Gpio2, gpio::FunctionSpi, gpio::PullDown>,
    ),
>;
pub type CsPinB = gpio::Pin<bank0::Gpio5, gpio::FunctionSioOutput, gpio::PullDown>;
pub type SpiB = ExclusiveDevice<SpiBusB, CsPinB, Timer>;
pub type IfaceB = EmbeddedHalImplementation<SpiB>;

/// The high-priority driver half over transport `T`.
pub type Driver<T> = TC6HighPrio<T, RxP, TxP, M, NQ>;
pub type Hi = Driver<Iface>;
pub type HiB = Driver<IfaceB>;
pub type Lo = TC6LowPrio<RxP, TxP, FlagSignal, M, NQ>;

/// Run the same early fixups `#[rp235x_hal::entry]` does: release all SIO spinlocks (a probe
/// reset can leave the critical-section lock held) and enable the coprocessors. Binaries using
/// `#[hal::entry]` get this for free; the embedded-test harness owns `main`, so its setup hook
/// must call this before anything takes a critical section (defmt-rtt does).
///
/// # Safety
/// Call once, first thing after reset, single core.
pub unsafe fn early_init() {
    const SIO_BASE: u32 = 0xd000_0000;
    const SPINLOCK0_PTR: *mut u32 = (SIO_BASE + 0x100) as *mut u32;
    for i in 0..32 {
        unsafe { SPINLOCK0_PTR.wrapping_add(i).write_volatile(1) };
    }
    const SCB_CPACR_PTR: *mut u32 = 0xE000_ED88 as *mut u32;
    let mut cpacr = unsafe { SCB_CPACR_PTR.read_volatile() };
    cpacr |= 0b11 << (4 * 2);
    cpacr |= 0b11;
    unsafe { SCB_CPACR_PTR.write_volatile(cpacr) };
    core::sync::atomic::compiler_fence(Ordering::SeqCst);
}

pub type ResetPin = gpio::Pin<bank0::Gpio15, gpio::FunctionSioOutput, gpio::PullDown>;
pub type IrqPin = gpio::Pin<bank0::Gpio14, gpio::FunctionSioInput, gpio::PullUp>;
pub type ResetPinB = gpio::Pin<bank0::Gpio7, gpio::FunctionSioOutput, gpio::PullDown>;
pub type IrqPinB = gpio::Pin<bank0::Gpio6, gpio::FunctionSioInput, gpio::PullUp>;

/// The non-SPI MAC-PHY lines plus a timer. Defaults to chip A's pins.
pub struct Ctl<R = ResetPin, I = IrqPin> {
    pub reset_n: R,
    pub irq_n: I,
    pub timer: Timer,
    pub peri_hz: HertzU32,
}
pub type CtlB = Ctl<ResetPinB, IrqPinB>;

impl<R: OutputPin, I: InputPin> Ctl<R, I> {
    /// Pulse RESET_N low for 10 ms, then wait 10 ms for the chip to come out of reset.
    pub fn hw_reset(&mut self) {
        self.reset_n.set_low().unwrap();
        self.timer.delay_ms(10);
        self.reset_n.set_high().unwrap();
        self.timer.delay_ms(10);
    }

    /// `true` while IRQ_N is low.
    pub fn irq_asserted(&mut self) -> bool {
        self.irq_n.is_low().unwrap()
    }

    /// Busy-wait until IRQ_N reaches `asserted`, up to `timeout_ms`. Returns whether it did.
    pub fn wait_irq(&mut self, asserted: bool, timeout_ms: u32) -> bool {
        for _ in 0..timeout_ms * 10 {
            if self.irq_asserted() == asserted {
                return true;
            }
            self.timer.delay_us(100);
        }
        self.irq_asserted() == asserted
    }

    pub fn delay_ms(&mut self, ms: u32) {
        self.timer.delay_ms(ms);
    }
}

/// Everything for both chips, as returned by [`init_both`].
pub struct Boards {
    pub a: Iface,
    pub ctl_a: Ctl,
    pub b: IfaceB,
    pub ctl_b: CtlB,
}

/// Bring up clocks, pins and both SPI buses (mode 0, [`SPI_HZ`]). Both RESET_N lines are left
/// **deasserted** (high); call [`Ctl::hw_reset`] to get a chip into a known state.
///
/// Panics if called twice in one boot (takes the PAC peripherals).
pub fn init_both() -> Boards {
    let mut pac = pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let clocks = hal::clocks::init_clocks_and_plls(
        XTAL_FREQ_HZ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .unwrap();
    let timer = hal::Timer::new_timer0(pac.TIMER0, &mut pac.RESETS, &clocks);

    let sio = hal::Sio::new(pac.SIO);
    let pins = hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let peri_hz = clocks.peripheral_clock.freq();

    let reset_n = pins.gpio15.into_push_pull_output_in_state(PinState::High);
    let irq_n = pins.gpio14.into_pull_up_input();
    let cs_n = pins.gpio13.into_push_pull_output_in_state(PinState::High);
    let sclk = pins.gpio10.into_function::<gpio::FunctionSpi>();
    let mosi = pins.gpio11.into_function::<gpio::FunctionSpi>();
    let miso = pins.gpio12.into_function::<gpio::FunctionSpi>();
    let spi = hal::spi::Spi::<_, _, _, 8>::new(pac.SPI1, (mosi, miso, sclk)).init(
        &mut pac.RESETS,
        peri_hz,
        SPI_HZ.Hz(),
        embedded_hal::spi::MODE_0,
    );

    let reset_n_b = pins.gpio7.into_push_pull_output_in_state(PinState::High);
    let irq_n_b = pins.gpio6.into_pull_up_input();
    let cs_n_b = pins.gpio5.into_push_pull_output_in_state(PinState::High);
    let sclk_b = pins.gpio2.into_function::<gpio::FunctionSpi>();
    let mosi_b = pins.gpio3.into_function::<gpio::FunctionSpi>();
    let miso_b = pins.gpio4.into_function::<gpio::FunctionSpi>();
    let spi_b = hal::spi::Spi::<_, _, _, 8>::new(pac.SPI0, (mosi_b, miso_b, sclk_b)).init(
        &mut pac.RESETS,
        peri_hz,
        SPI_HZ.Hz(),
        embedded_hal::spi::MODE_0,
    );

    Boards {
        a: EmbeddedHalImplementation::new(ExclusiveDevice::new(spi, cs_n, timer).unwrap()),
        ctl_a: Ctl {
            reset_n,
            irq_n,
            timer,
            peri_hz,
        },
        b: EmbeddedHalImplementation::new(ExclusiveDevice::new(spi_b, cs_n_b, timer).unwrap()),
        ctl_b: Ctl {
            reset_n: reset_n_b,
            irq_n: irq_n_b,
            timer,
            peri_hz,
        },
    }
}

/// Chip A only. Chip B is held in reset so it stays silent on the cable: it keeps its
/// configuration across an MCU reset, and a leftover PLCA coordinator would send BEACONs at A.
pub fn init() -> (Iface, Ctl) {
    let mut boards = init_both();
    boards.ctl_b.reset_n.set_low().unwrap();
    (boards.a, boards.ctl_a)
}

/// Change the SPI clock. Returns the rate actually achieved (an integer division of `peri_hz`).
pub fn set_spi_hz(iface: &mut Iface, peri_hz: HertzU32, hz: u32) -> u32 {
    iface
        .inner_mut()
        .bus_mut()
        .set_baudrate(peri_hz, hz.Hz())
        .to_Hz()
}

/// [`set_spi_hz`] for chip B.
pub fn set_spi_hz_b(iface: &mut IfaceB, peri_hz: HertzU32, hz: u32) -> u32 {
    iface
        .inner_mut()
        .bus_mut()
        .set_baudrate(peri_hz, hz.Hz())
        .to_Hz()
}

/// Polling transmit signal: [`TC6LowPrio`] sets the flag, the main loop calls
/// `on_transmit_ready` when it sees it (see [`take_tx_pending`]).
pub struct FlagSignal(&'static AtomicBool);
static TX_PENDING: AtomicBool = AtomicBool::new(false);
static TX_PENDING_B: AtomicBool = AtomicBool::new(false);
impl TransmitSignal for FlagSignal {
    fn signal(&mut self) {
        self.0.store(true, Ordering::Release);
    }
}
/// Consume the flag set by chip A's [`FlagSignal`].
pub fn take_tx_pending() -> bool {
    TX_PENDING.swap(false, Ordering::Acquire)
}
/// Consume the flag set by chip B's [`FlagSignal`].
pub fn take_tx_pending_b() -> bool {
    TX_PENDING_B.swap(false, Ordering::Acquire)
}

/// `'static` storage for one driver instance: adds `POOL_BLOCKS` buffers to each shared pool and
/// yields that instance's queues. Each expansion owns its statics and can run once per boot.
macro_rules! driver_storage {
    () => {{
        static mut RX_BLOCKS: [BoxBlock<FrameBuffer<M>>; POOL_BLOCKS] =
            [const { BoxBlock::new() }; POOL_BLOCKS];
        static mut TX_BLOCKS: [BoxBlock<FrameBuffer<M>>; POOL_BLOCKS] =
            [const { BoxBlock::new() }; POOL_BLOCKS];
        static mut RX_READY: Queue<Box<RxP>, NQ> = Queue::new();
        static mut TX_READY: Queue<Box<TxP>, NQ> = Queue::new();
        static TAKEN: AtomicBool = AtomicBool::new(false);

        assert!(
            !TAKEN.swap(true, Ordering::AcqRel),
            "driver for this chip taken twice in one boot"
        );
        // SAFETY: guarded by TAKEN, so each static is borrowed exactly once per boot.
        unsafe {
            for b in (*(&raw mut RX_BLOCKS)).iter_mut() {
                RxP.manage(b);
            }
            for b in (*(&raw mut TX_BLOCKS)).iter_mut() {
                TxP.manage(b);
            }
            (&mut *(&raw mut RX_READY), &mut *(&raw mut TX_READY))
        }
    }};
}

/// Hand the pools and queues to chip A's driver and return the **unconfigured** pair (run
/// `hi.configure(..)` next).
///
/// The pools/queues are `'static` singletons, so this works once per boot; a second call
/// panics. The test harness resets the MCU before every test, so each test may call it.
pub fn take_driver(iface: Iface) -> (Hi, Lo) {
    let (rx_ready, tx_ready) = driver_storage!();
    tc6_driver::split::<_, RxP, TxP, _, M, NQ>(iface, rx_ready, tx_ready, FlagSignal(&TX_PENDING))
}

/// [`take_driver`] for chip B.
pub fn take_driver_b(iface: IfaceB) -> (HiB, Lo) {
    let (rx_ready, tx_ready) = driver_storage!();
    tc6_driver::split::<_, RxP, TxP, _, M, NQ>(iface, rx_ready, tx_ready, FlagSignal(&TX_PENDING_B))
}

/// Error from the pumped register helpers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum RegError {
    /// `begin_register_*` refused (busy / bad count / interface error).
    Begin,
    /// `on_interrupt` returned an interface error.
    Interface,
    /// The chip flagged the control header as bad (HDRB / echo parity).
    HeaderBad,
    /// No result after the poll budget.
    Timeout,
}

fn pump<T: Tc6Interface>(hi: &mut Driver<T>) -> Result<tc6_driver::ControlResult, RegError> {
    for _ in 0..1000 {
        hi.on_interrupt().map_err(|_| RegError::Interface)?;
        if let Some(r) = hi.take_control_result() {
            if r.header_bad {
                return Err(RegError::HeaderBad);
            }
            return Ok(r);
        }
    }
    Err(RegError::Timeout)
}

fn begin_err<E>(_: ControlError<E>) -> RegError {
    RegError::Begin
}

/// Read `out.len()` consecutive registers through the driver's control path.
pub fn read_regs<T: Tc6Interface>(
    hi: &mut Driver<T>,
    mms: u8,
    addr: u16,
    out: &mut [u32],
) -> Result<(), RegError> {
    hi.begin_register_read(mms, addr, out.len() as u8)
        .map_err(begin_err)?;
    let r = pump(hi)?;
    out.copy_from_slice(r.values());
    Ok(())
}

/// Read one register through the driver's control path.
pub fn read_reg<T: Tc6Interface>(hi: &mut Driver<T>, mms: u8, addr: u16) -> Result<u32, RegError> {
    let mut v = [0];
    read_regs(hi, mms, addr, &mut v)?;
    Ok(v[0])
}

/// Write one register through the driver's control path; returns the echoed write data.
pub fn write_reg<T: Tc6Interface>(
    hi: &mut Driver<T>,
    mms: u8,
    addr: u16,
    value: u32,
) -> Result<u32, RegError> {
    hi.begin_register_write(mms, addr, &[value])
        .map_err(begin_err)?;
    let r = pump(hi)?;
    Ok(r.values()[0])
}

/// LAN865x MAC registers (MMS 1). These are Microchip-specific, not part of TC6, and the driver
/// doesn't touch them yet. Addresses and bits come from the Linux `lan865x` driver; check them
/// against the LAN8650/1 datasheet before relying on them.
pub mod lan865x {
    /// MAC network control register.
    pub const MAC_NCR: u16 = 0x0000;
    pub const NCR_RXEN: u32 = 1 << 2;
    pub const NCR_TXEN: u32 = 1 << 3;
    /// MAC network configuration register.
    pub const MAC_NCFG: u16 = 0x0001;
    /// Copy all frames (promiscuous).
    pub const NCFG_CAF: u32 = 1 << 4;
}

/// Turn on the LAN865x MAC's transmitter and receiver (read-modify-write of `MAC_NCR`), and
/// optionally promiscuous receive. Returns `(NCR, NCFG)` as read before the change.
pub fn enable_mac<T: Tc6Interface>(
    hi: &mut Driver<T>,
    promiscuous: bool,
) -> Result<(u32, u32), RegError> {
    use hermes_tc6::registers::mms;
    let ncr = read_reg(hi, mms::MAC, lan865x::MAC_NCR)?;
    let ncfg = read_reg(hi, mms::MAC, lan865x::MAC_NCFG)?;
    if promiscuous {
        write_reg(hi, mms::MAC, lan865x::MAC_NCFG, ncfg | lan865x::NCFG_CAF)?;
    }
    write_reg(
        hi,
        mms::MAC,
        lan865x::MAC_NCR,
        ncr | lan865x::NCR_TXEN | lan865x::NCR_RXEN,
    )?;
    Ok((ncr, ncfg))
}

/// Raw SPI access that bypasses the driver - for checking the wire format against silicon and
/// for deliberately malformed transactions. Only use while the driver is idle.
pub mod raw {
    use super::*;

    /// Result of a raw control transaction.
    pub struct Control {
        /// MISO bytes `[4..8)`: the header as echoed by the MAC-PHY.
        pub echoed: ControlHeader,
        /// MISO words after the echoed header: read data, or echoed write data.
        pub values: heapless::Vec<u32, 8>,
    }

    /// One full-duplex transfer with CS_N held for its whole length.
    pub fn transfer<S: SpiDevice>(spi: &mut S, tx: &[u8], rx: &mut [u8]) {
        spi.transfer(rx, tx).unwrap();
    }

    /// Run a control transaction with an arbitrary (possibly corrupt) `header`. `count` words
    /// are clocked after the header (`write_data` for a write, zeros for a read).
    pub fn control_with_header<S: SpiDevice>(
        spi: &mut S,
        header: ControlHeader,
        count: usize,
        write_data: &[u32],
    ) -> Control {
        assert!((1..=8).contains(&count));
        let len = 4 * (count + 2);
        let mut tx = [0u8; 40];
        let mut rx = [0u8; 40];
        tx[..4].copy_from_slice(&header.raw().to_be_bytes());
        for (i, w) in write_data.iter().take(count).enumerate() {
            tx[4 + 4 * i..8 + 4 * i].copy_from_slice(&w.to_be_bytes());
        }
        transfer(spi, &tx[..len], &mut rx[..len]);
        let word = |i: usize| u32::from_be_bytes(rx[i..i + 4].try_into().unwrap());
        let mut values = heapless::Vec::new();
        for i in 0..count {
            values.push(word(8 + 4 * i)).unwrap();
        }
        Control {
            echoed: ControlHeader::from_raw(word(4)),
            values,
        }
    }

    /// A well-formed control header (DNC = 0, AID = 0, correct parity).
    pub fn header(write: bool, mms: u8, addr: u16, count: usize) -> ControlHeader {
        ControlHeader::new()
            .with_wnr(write)
            .with_mms(mms)
            .with_addr(addr)
            .with_len((count - 1) as u8)
            .with_correct_parity()
    }

    pub fn read<S: SpiDevice>(spi: &mut S, mms: u8, addr: u16) -> Control {
        control_with_header(spi, header(false, mms, addr, 1), 1, &[])
    }

    pub fn write<S: SpiDevice>(spi: &mut S, mms: u8, addr: u16, value: u32) -> Control {
        control_with_header(spi, header(true, mms, addr, 1), 1, &[value])
    }

    /// Send one data chunk (`header` as given, 64-byte `payload`) and return the receive footer
    /// plus the 64 receive payload bytes clocked out alongside it.
    pub fn data_chunk<S: SpiDevice>(
        spi: &mut S,
        header: TransmitHeader,
        payload: &[u8; 64],
    ) -> (ReceiveFooter, [u8; 64]) {
        let mut tx = [0u8; 68];
        let mut rx = [0u8; 68];
        tx[..4].copy_from_slice(&header.raw().to_be_bytes());
        tx[4..].copy_from_slice(payload);
        transfer(spi, &tx, &mut rx);
        let footer = ReceiveFooter::from_raw(u32::from_be_bytes(rx[64..68].try_into().unwrap()));
        let mut data = [0u8; 64];
        data.copy_from_slice(&rx[..64]);
        (footer, data)
    }

    /// An empty data chunk: DNC = 1, no transmit data, receive allowed.
    pub fn empty_chunk_header() -> TransmitHeader {
        TransmitHeader::new().with_dnc(true).with_correct_parity()
    }
}
