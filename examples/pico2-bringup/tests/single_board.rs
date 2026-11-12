//! On-target tests that need only **one** LAN8651 board: no T1S cable or peer node required.
//!
//! Run from `examples/pico2-bringup` with `cargo test` (probe-rs + embedded-test). probe-rs
//! resets the MCU before every test, and `init` pulses RESET_N, so every test starts with a fresh
//! MCU *and* a freshly reset MAC-PHY. Run one test with `cargo test -- <name>`.
//!
//! Several expectations below come from the OPEN Alliance TC6 spec, not from LAN8651-specific
//! documentation. A failure there may mean the chip interprets the spec differently from the
//! driver (which is exactly what these tests exist to find). Read the log before "fixing" the
//! test.

#![no_std]
#![no_main]

use defmt_rtt as _;

// Needs probe-rs >= 0.32: 0.31 kept its RTT attachment across the per-test reset, and the RP2350
// reset leaves SRAM scrambled, giving "control block has been corrupted: channel mode flags are
// invalid".
#[embedded_test::setup]
fn setup() {
    // SAFETY: first code after reset, single core.
    unsafe { pico2_bringup::early_init() };
}

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use defmt::{Debug2Format, assert, assert_eq, info, warn};
    use pico2_bringup::hermes_tc6::registers::{
        BufSts, Config0, IMask0, IdVer, PhyId, PlcaCtrl0, PlcaCtrl1, PlcaStatus, Status0, addr,
        mms, plca,
    };
    use pico2_bringup::hermes_tc6::tc6_driver::{Config, ConfigError, PlcaConfig};
    use pico2_bringup::hermes_tc6::tc6_header::TransmitHeader;
    use pico2_bringup::{self as board, Ctl, Iface, raw};

    /// Microchip LAN8650/1 as seen by the first bring-up (2026-11-03).
    const EXPECTED_IDVER: u32 = 0x0000_0011;
    const MICROCHIP_OUI_SLICE: u32 = 0x0001F0;
    const LAN865X_MODEL: u8 = 0x1B;

    /// The SPI sweep must be error-free at and below this clock. The LAN8651's rated maximum;
    /// the first sweep (2026-11-03) was clean at every step up to it. Lower it if you rewire with
    /// longer leads and it starts failing.
    const SWEEP_MUST_PASS_UP_TO_HZ: u32 = 25_000_000;

    struct State {
        iface: Iface,
        ctl: Ctl,
    }

    #[init]
    fn init() -> State {
        let (iface, mut ctl) = board::init();
        ctl.hw_reset();
        State { iface, ctl }
    }

    fn configured(state: State, cfg: &Config) -> (board::Hi, Ctl) {
        let (mut hi, _lo) = board::take_driver(state.iface);
        if let Err(e) = hi.configure(cfg) {
            defmt::panic!("configure() failed: {}", Debug2Format(&e));
        }
        assert!(hi.is_configured());
        (hi, state.ctl)
    }

    // ------------------------------------------------------------------------------------------
    // Wire format, checked against silicon without the driver
    // ------------------------------------------------------------------------------------------

    /// A hand-built control read of IDVER returns TC6 v1.1 and echoes our header unchanged.
    #[test]
    #[timeout(10)]
    fn raw_control_read_echoes_header_and_reads_idver(mut state: State) {
        let spi = state.iface.inner_mut();
        let sent = raw::header(false, mms::STANDARD, addr::IDVER, 1);
        let r = raw::control_with_header(spi, sent, 1, &[]);
        info!(
            "sent {=u32:#010x}, echoed {=u32:#010x}, IDVER {=u32:#010x}",
            sent.raw(),
            r.echoed.raw(),
            r.values[0]
        );
        assert!(!r.echoed.hdrb(), "chip flagged our header as bad");
        assert_eq!(r.echoed.raw(), sent.raw(), "echoed header differs");
        assert_eq!(r.values[0], EXPECTED_IDVER);
    }

    /// After a hardware reset (before configure) an empty data chunk's footer has a valid
    /// parity, SYNC = 0 and EXST = 1 (RESETC is pending).
    #[test]
    #[timeout(10)]
    fn footer_before_configure_reports_no_sync_and_exst(mut state: State) {
        let (f, _) = raw::data_chunk(state.iface.inner_mut(), raw::empty_chunk_header(), &[0; 64]);
        info!("footer {=u32:#010x}", f.raw());
        assert!(f.parity_ok(), "footer parity");
        assert!(!f.hdrb(), "chip rejected our data header");
        assert!(!f.sync(), "SYNC should be 0 before configure");
        assert!(f.exst(), "EXST should be set while RESETC is pending");
        assert!(!f.dv(), "no frame data expected");
    }

    /// After configure an empty data chunk's footer is fully consistent with BUFSTS: SYNC = 1,
    /// no data, and TXC agrees with the register (saturated to 5 bits in the footer).
    #[test]
    #[timeout(10)]
    fn footer_after_configure_reports_sync_and_credits(state: State) {
        let (mut hi, _ctl) = configured(state, &Config::default());
        let bufsts = BufSts::from_raw(hi.read_register(mms::STANDARD, addr::BUFSTS).unwrap());
        let (f, _) = raw::data_chunk(
            hi.interface_mut().inner_mut(),
            raw::empty_chunk_header(),
            &[0; 64],
        );
        info!(
            "footer {=u32:#010x} (txc {=u8}, rca {=u8}), BUFSTS txc {=u8} rca {=u8}",
            f.raw(),
            f.txc(),
            f.rca(),
            bufsts.txc(),
            bufsts.rca()
        );
        assert!(f.valid(), "footer fails the driver's validity check");
        assert!(f.sync(), "SYNC should be 1 after configure");
        assert!(!f.dv());
        assert_eq!(f.rca(), 0, "nothing on the wire, nothing to receive");
        assert_eq!(
            f.txc(),
            bufsts.txc().min(31),
            "footer TXC disagrees with BUFSTS"
        );
        assert!(f.txc() > 0);
    }

    // ------------------------------------------------------------------------------------------
    // Bring-up and reset
    // ------------------------------------------------------------------------------------------

    /// `configure()` succeeds and leaves CONFIG0 / STATUS0 / IDs as expected.
    #[test]
    #[timeout(10)]
    fn configure_after_hw_reset(state: State) {
        let (mut hi, _ctl) = configured(state, &Config::default());

        let id = IdVer::from_raw(hi.read_register(mms::STANDARD, addr::IDVER).unwrap());
        assert_eq!((id.majver(), id.minver()), (1, 1));

        let phy = PhyId::from_raw(hi.read_register(mms::STANDARD, addr::PHYID).unwrap());
        info!(
            "PHYID oui {=u32:#08x} model {=u8:#04x} rev {=u8}",
            phy.oui(),
            phy.model(),
            phy.revision()
        );
        assert_eq!(phy.oui(), MICROCHIP_OUI_SLICE);
        assert_eq!(phy.model(), LAN865X_MODEL);

        let c0 = Config0::from_raw(hi.read_register(mms::STANDARD, addr::CONFIG0).unwrap());
        assert!(c0.sync());
        assert_eq!(c0.cps_bytes(), 64);

        let s0 = hi.read_register(mms::STANDARD, addr::STATUS0).unwrap();
        assert_eq!(s0, 0, "STATUS0 should be clear after configure");
    }

    /// With the chip fresh out of a hardware reset, configure works without SWRESET too.
    #[test]
    #[timeout(10)]
    fn configure_without_swreset_after_hw_reset(state: State) {
        let cfg = Config {
            software_reset: false,
            ..Config::default()
        };
        configured(state, &cfg);
    }

    /// MCU-only reboot path: repeated configure() on an already-configured chip keeps working
    /// (each run issues SWRESET first).
    #[test]
    #[timeout(20)]
    fn reconfigure_repeatedly_without_hw_reset(state: State) {
        let cfg = Config::default();
        let (mut hi, _ctl) = configured(state, &cfg);
        for i in 0..5 {
            if let Err(e) = hi.configure(&cfg) {
                defmt::panic!("reconfigure #{} failed: {}", i, Debug2Format(&e));
            }
            assert!(hi.is_configured());
        }
    }

    /// Without SWRESET, configuring an already-configured chip must fail: RESETC was cleared by
    /// the first configure and nothing sets it again.
    #[test]
    #[timeout(10)]
    fn configure_without_swreset_on_configured_chip_fails(state: State) {
        let (mut hi, _ctl) = configured(state, &Config::default());
        let cfg = Config {
            software_reset: false,
            ..Config::default()
        };
        let r = hi.configure(&cfg);
        assert!(
            matches!(r, Err(ConfigError::ResetNotComplete)),
            "expected ResetNotComplete, got {}",
            Debug2Format(&r)
        );
    }

    /// IRQ_N is asserted after reset (RESETC), still asserted after configure (no data chunk
    /// has been exchanged yet), and released by the first `on_interrupt()`.
    #[test]
    #[timeout(10)]
    fn irq_asserted_after_reset_and_released_by_on_interrupt(mut state: State) {
        assert!(
            state.ctl.wait_irq(true, 50),
            "IRQ_N not asserted after reset"
        );
        let (mut hi, mut ctl) = configured(state, &Config::default());
        assert!(ctl.irq_asserted(), "IRQ_N released before any data chunk");
        hi.on_interrupt().unwrap();
        assert!(
            ctl.wait_irq(false, 50),
            "IRQ_N still asserted after on_interrupt"
        );
    }

    /// Pulling RESET_N under a running driver: the next data chunk's footer reports SYNC = 0,
    /// the driver notices (`is_configured() == false`), and configure() recovers.
    #[test]
    #[timeout(10)]
    fn hw_reset_under_driver_is_detected_and_recoverable(state: State) {
        let cfg = Config::default();
        let (mut hi, mut ctl) = configured(state, &cfg);
        hi.on_interrupt().unwrap();
        assert!(ctl.wait_irq(false, 50));

        ctl.hw_reset();
        assert!(ctl.wait_irq(true, 50), "IRQ_N not asserted after reset");
        for _ in 0..10 {
            hi.on_interrupt().unwrap();
            if !hi.is_configured() {
                break;
            }
        }
        assert!(
            !hi.is_configured(),
            "driver did not notice the chip lost SYNC"
        );

        if let Err(e) = hi.configure(&cfg) {
            defmt::panic!("re-configure after reset failed: {}", Debug2Format(&e));
        }
        assert!(hi.is_configured());
    }

    // ------------------------------------------------------------------------------------------
    // Control path
    // ------------------------------------------------------------------------------------------

    /// Writes to IMASK0 echo back the written data (checked by `write_register`) and read back as
    /// written (RESETCM, bit 6, is read-only 0).
    #[test]
    #[timeout(10)]
    fn imask0_write_readback(state: State) {
        let (mut hi, _ctl) = configured(state, &Config::default());
        let writable = IMask0::RESET_VALUE.raw(); // 0x1FBF: every defined bit except RESETCM
        for pattern in [
            0x0000_0000,
            writable,
            0x0000_0AAA & writable,
            0x0000_1515 & writable,
        ] {
            // write_register() itself fails if the echoed write data differs (spec 7.4.4)
            hi.write_register(mms::STANDARD, addr::IMASK0, pattern)
                .unwrap();
            let back = hi.read_register(mms::STANDARD, addr::IMASK0).unwrap();
            info!("IMASK0 wrote {=u32:#010x} read {=u32:#010x}", pattern, back);
            assert_eq!(back, pattern, "readback differs");
        }
        hi.write_register(mms::STANDARD, addr::IMASK0, writable)
            .unwrap();
    }

    /// Multi-register reads (address auto-increment) match the same registers read one by one.
    #[test]
    #[timeout(10)]
    fn multi_register_read_matches_single_reads(state: State) {
        let (mut hi, _ctl) = configured(state, &Config::default());
        for count in [2usize, 3, 8] {
            let mut burst = [0u32; 8];
            hi.read_registers(mms::STANDARD, 0x0000, &mut burst[..count])
                .unwrap();
            for (i, &v) in burst[..count].iter().enumerate() {
                let single = hi.read_register(mms::STANDARD, i as u16).unwrap();
                assert_eq!(v, single, "count {}: reg {=usize:#06x} differs", count, i);
            }
        }
    }

    /// The non-blocking control path (`begin_register_read` + `on_interrupt` +
    /// `take_control_result`) returns the same values as the blocking `read_registers`.
    #[test]
    #[timeout(10)]
    fn non_blocking_register_read_matches_blocking(state: State) {
        let (mut hi, _ctl) = configured(state, &Config::default());
        let mut blocking = [0u32; 3];
        hi.read_registers(mms::STANDARD, addr::IDVER, &mut blocking)
            .unwrap();

        hi.begin_register_read(mms::STANDARD, addr::IDVER, 3)
            .unwrap();
        let mut result = None;
        for _ in 0..1000 {
            hi.on_interrupt().unwrap();
            result = hi.take_control_result();
            if result.is_some() {
                break;
            }
        }
        let result = result.expect("no control result after 1000 on_interrupt calls");
        assert!(!result.header_bad);
        info!(
            "blocking {=[?]:#010x}, non-blocking {=[?]:#010x}",
            blocking,
            result.values()
        );
        assert_eq!(result.values(), &blocking[..]);
    }

    /// The SPI clock sweep: 1000 raw IDVER reads per clock. Logs the error count per clock, and
    /// must be clean up to [`SWEEP_MUST_PASS_UP_TO_HZ`].
    #[test]
    #[timeout(60)]
    fn spi_clock_sweep(mut state: State) {
        const READS: u32 = 1000;
        let peri = state.ctl.peri_hz;
        let mut failures = 0;
        for want in [
            1_000_000, 5_000_000, 10_000_000, 15_000_000, 20_000_000, 25_000_000,
        ] {
            let got = board::set_spi_hz(&mut state.iface, peri, want);
            let spi = state.iface.inner_mut();
            let sent = raw::header(false, mms::STANDARD, addr::IDVER, 1);
            let mut errors = 0u32;
            for _ in 0..READS {
                let r = raw::control_with_header(spi, sent, 1, &[]);
                if r.echoed.raw() != sent.raw() || r.values[0] != EXPECTED_IDVER {
                    errors += 1;
                }
            }
            info!(
                "SPI {=u32} Hz (asked {=u32}): {=u32}/{=u32} bad reads",
                got, want, errors, READS
            );
            if errors > 0 && want <= SWEEP_MUST_PASS_UP_TO_HZ {
                failures += 1;
            }
        }
        board::set_spi_hz(&mut state.iface, peri, board::SPI_HZ);
        assert_eq!(failures, 0, "errors at or below the required clock");
    }

    // ------------------------------------------------------------------------------------------
    // PLCA (register map in the driver is a best guess - these check it)
    // ------------------------------------------------------------------------------------------

    /// MMS 4 0xCA00 is the OPEN Alliance PLCA IDVER: map ID 0x0A in bits 15..8.
    #[test]
    #[timeout(10)]
    fn plca_idver_is_open_alliance_map(state: State) {
        let (mut hi, _ctl) = configured(state, &Config::default());
        let v = hi.read_register(mms::PHY_VS_PLCA, plca::IDVER).unwrap();
        info!("PLCA IDVER = {=u32:#010x}", v);
        assert_eq!((v >> 8) & 0xFF, 0x0A, "not the OPEN Alliance PLCA map");
    }

    /// Coordinator setup reads back, and a lone coordinator runs PLCA (it sends the BEACON).
    #[test]
    #[timeout(10)]
    fn plca_coordinator_config_reads_back(state: State) {
        let cfg = Config {
            plca: Some(PlcaConfig::coordinator(8)),
            ..Config::default()
        };
        let (mut hi, mut ctl) = configured(state, &cfg);
        let c0 = PlcaCtrl0::from_raw(hi.read_register(mms::PHY_VS_PLCA, plca::CTRL0).unwrap());
        let c1 = PlcaCtrl1::from_raw(hi.read_register(mms::PHY_VS_PLCA, plca::CTRL1).unwrap());
        info!(
            "PLCA CTRL0 {=u32:#06x} CTRL1 {=u32:#06x}",
            c0.raw(),
            c1.raw()
        );
        assert!(c0.enable());
        assert_eq!(c1.id(), 0);
        assert_eq!(c1.node_count(), 8);

        ctl.delay_ms(100);
        let st = PlcaStatus::from_raw(hi.read_register(mms::PHY_VS_PLCA, plca::STATUS).unwrap());
        info!("PLCA STATUS {=u32:#06x}", st.raw());
        assert!(st.pst(), "lone coordinator should report PLCA active");
    }

    /// Follower setup reads back; with no coordinator on the line there is no BEACON, so PLCA
    /// must not report active.
    #[test]
    #[timeout(10)]
    fn plca_follower_config_reads_back(state: State) {
        let cfg = Config {
            plca: Some(PlcaConfig::follower(3)),
            ..Config::default()
        };
        let (mut hi, mut ctl) = configured(state, &cfg);
        let c0 = PlcaCtrl0::from_raw(hi.read_register(mms::PHY_VS_PLCA, plca::CTRL0).unwrap());
        let c1 = PlcaCtrl1::from_raw(hi.read_register(mms::PHY_VS_PLCA, plca::CTRL1).unwrap());
        info!(
            "PLCA CTRL0 {=u32:#06x} CTRL1 {=u32:#06x}",
            c0.raw(),
            c1.raw()
        );
        assert!(c0.enable());
        assert_eq!(c1.id(), 3);

        ctl.delay_ms(100);
        let st = PlcaStatus::from_raw(hi.read_register(mms::PHY_VS_PLCA, plca::STATUS).unwrap());
        info!("PLCA STATUS {=u32:#06x}", st.raw());
        assert!(
            !st.pst(),
            "follower without a coordinator should not report PLCA active"
        );
    }

    // ------------------------------------------------------------------------------------------
    // Error injection: the chip must flag each, and the driver must be able to read it back
    // ------------------------------------------------------------------------------------------

    /// A control header with flipped parity is rejected (HDRB in the echo, or an echo that
    /// itself fails parity - spec 7.5.1) and latches STATUS0.HDRE.
    #[test]
    #[timeout(10)]
    fn bad_control_header_parity_is_rejected(state: State) {
        let (mut hi, _ctl) = configured(state, &Config::default());
        let good = raw::header(false, mms::STANDARD, addr::IDVER, 1);
        let bad = good.with_parity(!good.parity());
        let r = raw::control_with_header(hi.interface_mut().inner_mut(), bad, 1, &[]);
        info!(
            "sent {=u32:#010x}, echoed {=u32:#010x}, data {=u32:#010x}",
            bad.raw(),
            r.echoed.raw(),
            r.values[0]
        );
        assert!(
            r.echoed.hdrb() || !r.echoed.parity_ok(),
            "chip accepted a header with bad parity"
        );

        let s0 = Status0::from_raw(hi.read_register(mms::STANDARD, addr::STATUS0).unwrap());
        info!("STATUS0 {=u32:#010x}", s0.raw());
        assert!(s0.hdre(), "STATUS0.HDRE not set");
    }

    /// Releasing CS_N in the middle of a data chunk latches STATUS0.LOFE.
    #[test]
    #[timeout(10)]
    fn early_cs_release_sets_lofe(state: State) {
        let (mut hi, _ctl) = configured(state, &Config::default());
        let mut tx = [0u8; 34]; // 4-byte header + 30 of 64 payload bytes
        let mut rx = [0u8; 34];
        tx[..4].copy_from_slice(&raw::empty_chunk_header().raw().to_be_bytes());
        raw::transfer(hi.interface_mut().inner_mut(), &tx, &mut rx);

        let s0 = Status0::from_raw(hi.read_register(mms::STANDARD, addr::STATUS0).unwrap());
        info!("STATUS0 {=u32:#010x}", s0.raw());
        assert!(s0.lofe(), "STATUS0.LOFE not set");
    }

    /// A data chunk that ends a frame that was never started (DV = 1, SV = 0, EV = 1) is a
    /// transmit protocol error: STATUS0.TXPE.
    #[test]
    #[timeout(10)]
    fn end_without_start_sets_txpe(state: State) {
        let (mut hi, _ctl) = configured(state, &Config::default());
        let header = TransmitHeader::new()
            .with_dnc(true)
            .with_dv(true)
            .with_sv(false)
            .with_ev(true)
            .with_ebo(63)
            .with_correct_parity();
        let (f, _) = raw::data_chunk(hi.interface_mut().inner_mut(), header, &[0xA5; 64]);
        info!("footer {=u32:#010x}", f.raw());

        let s0 = Status0::from_raw(hi.read_register(mms::STANDARD, addr::STATUS0).unwrap());
        info!("STATUS0 {=u32:#010x}", s0.raw());
        if !s0.txpe() {
            warn!("no TXPE - check the LAN8651 datasheet for which chunk sequences it flags");
        }
        assert!(s0.txpe(), "STATUS0.TXPE not set");
    }
}
