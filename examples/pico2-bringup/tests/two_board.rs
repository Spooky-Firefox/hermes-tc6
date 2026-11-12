//! On-target tests for two LAN8651s on one Pico 2, joined by a terminated twisted pair (wiring:
//! `src/lib.rs`). Run with `cargo test --test two_board`.
//!
//! `init` pulses both RESET_N lines, so every test starts with both chips fresh. These tests
//! check that chip B answers, that its control lines aren't crossed with chip A's, and that the
//! cable carries a PLCA BEACON. Frame transfer between the chips is not covered yet.

#![no_std]
#![no_main]

use defmt_rtt as _;

#[embedded_test::setup]
fn setup() {
    // SAFETY: first code after reset, single core.
    unsafe { pico2_bringup::early_init() };
}

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use defmt::{Debug2Format, assert, assert_eq, info};
    use pico2_bringup::hermes_tc6::registers::{PlcaStatus, Status0, addr, mms, plca};
    use pico2_bringup::hermes_tc6::tc6_driver::{Config, PlcaConfig};
    use pico2_bringup::{self as board, Boards, raw};

    const EXPECTED_IDVER: u32 = 0x0000_0011;
    /// PHYID without the revision nibble: Microchip OUI slice + LAN865x model.
    const PHYID_MASK: u32 = 0xFFFF_FFF0;
    const EXPECTED_PHYID: u32 = 0x0007_C1B0;
    /// How long a PLCA state change gets to show up in PLCA_STATUS.
    const PLCA_SETTLE_MS: u32 = 100;

    #[init]
    fn init() -> Boards {
        let mut boards = board::init_both();
        boards.ctl_a.hw_reset();
        boards.ctl_b.hw_reset();
        boards
    }

    fn resetc<S: embedded_hal::spi::SpiDevice>(spi: &mut S) -> bool {
        let v = raw::read(spi, mms::STANDARD, addr::STATUS0).values[0];
        Status0::from_raw(v).resetc()
    }

    fn clear_resetc<S: embedded_hal::spi::SpiDevice>(spi: &mut S) {
        let w1c = Status0::from_raw(0).with_resetc(true).raw();
        raw::write(spi, mms::STANDARD, addr::STATUS0, w1c);
    }

    /// Both chips answer a hand-built control read with the expected identity.
    #[test]
    #[timeout(10)]
    fn both_chips_respond(mut b: Boards) {
        let sent = raw::header(false, mms::STANDARD, addr::IDVER, 1);

        let a = raw::control_with_header(b.a.inner_mut(), sent, 1, &[]);
        let a_phy = raw::read(b.a.inner_mut(), mms::STANDARD, addr::PHYID).values[0];
        info!(
            "A: echo {=u32:#010x} IDVER {=u32:#010x} PHYID {=u32:#010x}",
            a.echoed.raw(),
            a.values[0],
            a_phy
        );

        let c = raw::control_with_header(b.b.inner_mut(), sent, 1, &[]);
        let b_phy = raw::read(b.b.inner_mut(), mms::STANDARD, addr::PHYID).values[0];
        info!(
            "B: echo {=u32:#010x} IDVER {=u32:#010x} PHYID {=u32:#010x}",
            c.echoed.raw(),
            c.values[0],
            b_phy
        );

        assert_eq!(a.echoed.raw(), sent.raw(), "chip A echo");
        assert_eq!(a.values[0], EXPECTED_IDVER, "chip A IDVER");
        assert_eq!(a_phy & PHYID_MASK, EXPECTED_PHYID, "chip A PHYID");
        assert_eq!(c.echoed.raw(), sent.raw(), "chip B echo");
        assert_eq!(c.values[0], EXPECTED_IDVER, "chip B IDVER");
        assert_eq!(b_phy & PHYID_MASK, EXPECTED_PHYID, "chip B PHYID");
    }

    /// Each RESET_N resets only its own chip, seen through STATUS0.RESETC on the matching SPI bus.
    #[test]
    #[timeout(10)]
    fn reset_lines_are_not_crossed(mut b: Boards) {
        assert!(
            resetc(b.a.inner_mut()),
            "chip A: RESETC not set after reset"
        );
        assert!(
            resetc(b.b.inner_mut()),
            "chip B: RESETC not set after reset"
        );

        clear_resetc(b.a.inner_mut());
        clear_resetc(b.b.inner_mut());
        assert!(!resetc(b.a.inner_mut()), "chip A: RESETC did not clear");
        assert!(!resetc(b.b.inner_mut()), "chip B: RESETC did not clear");

        b.ctl_b.hw_reset();
        assert!(resetc(b.b.inner_mut()), "GPIO7 did not reset chip B");
        assert!(!resetc(b.a.inner_mut()), "GPIO7 also reset chip A");

        clear_resetc(b.b.inner_mut());
        b.ctl_a.hw_reset();
        assert!(resetc(b.a.inner_mut()), "GPIO15 did not reset chip A");
        assert!(!resetc(b.b.inner_mut()), "GPIO15 also reset chip B");
    }

    /// Each IRQ_N follows its own chip: both low after reset, and servicing one releases only it.
    #[test]
    #[timeout(10)]
    fn irq_lines_are_not_crossed(mut b: Boards) {
        assert!(b.ctl_a.wait_irq(true, 50), "GPIO14 not low after reset");
        assert!(b.ctl_b.wait_irq(true, 50), "GPIO6 not low after reset");

        let (mut hi_b, _lo_b) = board::take_driver_b(b.b);
        if let Err(e) = hi_b.configure(&Config::default()) {
            defmt::panic!("chip B configure() failed: {}", Debug2Format(&e));
        }
        hi_b.on_interrupt().unwrap();
        assert!(
            b.ctl_b.wait_irq(false, 50),
            "GPIO6 still low after servicing chip B"
        );
        assert!(
            b.ctl_a.irq_asserted(),
            "GPIO14 released by servicing chip B"
        );

        let (mut hi_a, _lo_a) = board::take_driver(b.a);
        if let Err(e) = hi_a.configure(&Config::default()) {
            defmt::panic!("chip A configure() failed: {}", Debug2Format(&e));
        }
        hi_a.on_interrupt().unwrap();
        assert!(
            b.ctl_a.wait_irq(false, 50),
            "GPIO14 still low after servicing chip A"
        );
    }

    /// Chip B's SPI wiring is clean up to the LAN8651's rated 25 MHz.
    #[test]
    #[timeout(30)]
    fn chip_b_spi_clock_sweep(mut b: Boards) {
        const READS: u32 = 1000;
        let peri = b.ctl_b.peri_hz;
        let sent = raw::header(false, mms::STANDARD, addr::IDVER, 1);
        let mut failures = 0;
        for want in [
            1_000_000, 5_000_000, 10_000_000, 15_000_000, 20_000_000, 25_000_000,
        ] {
            let got = board::set_spi_hz_b(&mut b.b, peri, want);
            let spi = b.b.inner_mut();
            let mut errors = 0u32;
            for _ in 0..READS {
                let r = raw::control_with_header(spi, sent, 1, &[]);
                if r.echoed.raw() != sent.raw() || r.values[0] != EXPECTED_IDVER {
                    errors += 1;
                }
            }
            info!("SPI {=u32} Hz: {=u32}/{=u32} bad reads", got, errors, READS);
            failures += (errors > 0) as u32;
        }
        assert_eq!(failures, 0, "chip B SPI errors");
    }

    /// The cable carries a BEACON from A to B. A follower reports PLCA active only once it hears
    /// a coordinator, so B must go inactive -> active when A starts.
    #[test]
    #[timeout(10)]
    fn beacon_from_a_reaches_b(mut b: Boards) {
        let (mut hi_b, _lo_b) = board::take_driver_b(b.b);
        let follower = Config {
            plca: Some(PlcaConfig::follower(1)),
            ..Config::default()
        };
        if let Err(e) = hi_b.configure(&follower) {
            defmt::panic!("chip B configure() failed: {}", Debug2Format(&e));
        }
        b.ctl_b.delay_ms(PLCA_SETTLE_MS);
        let before =
            PlcaStatus::from_raw(hi_b.read_register(mms::PHY_VS_PLCA, plca::STATUS).unwrap());
        info!("B PLCA STATUS with A silent: {=u32:#06x}", before.raw());
        assert!(
            !before.pst(),
            "chip B active with no coordinator on the line"
        );

        let (mut hi_a, _lo_a) = board::take_driver(b.a);
        let coordinator = Config {
            plca: Some(PlcaConfig::coordinator(2)),
            ..Config::default()
        };
        if let Err(e) = hi_a.configure(&coordinator) {
            defmt::panic!("chip A configure() failed: {}", Debug2Format(&e));
        }
        b.ctl_b.delay_ms(PLCA_SETTLE_MS);
        let a = PlcaStatus::from_raw(hi_a.read_register(mms::PHY_VS_PLCA, plca::STATUS).unwrap());
        let after =
            PlcaStatus::from_raw(hi_b.read_register(mms::PHY_VS_PLCA, plca::STATUS).unwrap());
        info!(
            "A PLCA STATUS {=u32:#06x}, B PLCA STATUS {=u32:#06x}",
            a.raw(),
            after.raw()
        );
        assert!(a.pst(), "chip A coordinator not active");
        assert!(after.pst(), "chip B never heard a BEACON from A");
    }

    /// The same check in the other direction, B to A.
    #[test]
    #[timeout(10)]
    fn beacon_from_b_reaches_a(mut b: Boards) {
        let (mut hi_a, _lo_a) = board::take_driver(b.a);
        let follower = Config {
            plca: Some(PlcaConfig::follower(1)),
            ..Config::default()
        };
        if let Err(e) = hi_a.configure(&follower) {
            defmt::panic!("chip A configure() failed: {}", Debug2Format(&e));
        }
        b.ctl_a.delay_ms(PLCA_SETTLE_MS);
        let before =
            PlcaStatus::from_raw(hi_a.read_register(mms::PHY_VS_PLCA, plca::STATUS).unwrap());
        info!("A PLCA STATUS with B silent: {=u32:#06x}", before.raw());
        assert!(
            !before.pst(),
            "chip A active with no coordinator on the line"
        );

        let (mut hi_b, _lo_b) = board::take_driver_b(b.b);
        let coordinator = Config {
            plca: Some(PlcaConfig::coordinator(2)),
            ..Config::default()
        };
        if let Err(e) = hi_b.configure(&coordinator) {
            defmt::panic!("chip B configure() failed: {}", Debug2Format(&e));
        }
        b.ctl_a.delay_ms(PLCA_SETTLE_MS);
        let s = PlcaStatus::from_raw(hi_b.read_register(mms::PHY_VS_PLCA, plca::STATUS).unwrap());
        let after =
            PlcaStatus::from_raw(hi_a.read_register(mms::PHY_VS_PLCA, plca::STATUS).unwrap());
        info!(
            "B PLCA STATUS {=u32:#06x}, A PLCA STATUS {=u32:#06x}",
            s.raw(),
            after.raw()
        );
        assert!(s.pst(), "chip B coordinator not active");
        assert!(after.pst(), "chip A never heard a BEACON from B");
    }
}
