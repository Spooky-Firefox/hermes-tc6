//! Bring-up: `configure`, `split_configured`, extended status and sync loss.

use super::*;

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
        ..Config::default()
    };
    hi.configure(&cfg).expect("bring-up ok");

    let c0 = registers::Config0::from_raw(hi.interface.reg(0, 0x0004));
    assert_eq!(c0.txcthresh(), 0b10);
    assert!(!c0.txcte() && !c0.rxcte(), "cut-through stays off");
    assert!(!c0.ftse() && !c0.ftss(), "timestamps stay off");
    assert!(c0.sync());
}

/// Cut-through is rejected even when the chip advertises `STDCAP.CTC`, because the data path
/// has no cut-through handling - same policy as `timestamp`.
#[test]
fn configure_rejects_cut_through_even_when_supported() {
    setup!(hi, _lo, RegDouble::new(), NoopSignal); // default STDCAP has CTC
    let cfg = Config {
        cut_through_tx: true,
        ..Config::default()
    };
    assert!(matches!(
        hi.configure(&cfg),
        Err(ConfigError::NotImplemented("cut-through"))
    ));
    assert!(!hi.is_configured());
    assert!(!registers::Config0::from_raw(hi.interface.reg(0, 0x0004)).txcte());
}

/// `Config::timestamp` is accepted by the type system but the receive path can't strip the
/// prepended timestamp yet, so `configure` refuses it (even though `RegDouble`'s STDCAP
/// advertises FTSC) rather than half-wiring FTSE and corrupting every received frame.
#[test]
fn configure_rejects_timestamp_until_rx_path_supports_it() {
    setup!(hi, _lo, RegDouble::new(), NoopSignal);
    let cfg = Config {
        timestamp: Some(TimestampSize::Bits64),
        ..Config::default()
    };
    assert!(matches!(
        hi.configure(&cfg),
        Err(ConfigError::NotImplemented("frame timestamp"))
    ));
    assert!(!hi.is_configured());
    assert!(!registers::Config0::from_raw(hi.interface.reg(0, 0x0004)).ftse());
}

/// With `software_reset` set (the default), `configure` asserts `RESET.SWRESET` as its very
/// first control write, before it starts polling `STATUS0.RESETC`.
#[test]
fn configure_asserts_swreset_first_by_default() {
    setup!(hi, _lo, RegDouble::new(), NoopSignal);
    hi.configure(&Config::default()).expect("bring-up ok");

    let first = hi
        .interface
        .writes
        .first()
        .copied()
        .expect("a write happened");
    assert_eq!(
        first,
        (0, registers::addr::RESET, registers::Reset::SWRESET.raw()),
        "SWRESET is the first register written"
    );
}

/// `software_reset = false` skips the `RESET.SWRESET` write entirely.
#[test]
fn configure_skips_swreset_when_disabled() {
    setup!(hi, _lo, RegDouble::new(), NoopSignal);
    let cfg = Config {
        software_reset: false,
        ..Config::default()
    };
    hi.configure(&cfg).expect("bring-up ok");

    assert!(
        !hi.interface
            .writes
            .iter()
            .any(|&(_, addr, _)| addr == registers::addr::RESET),
        "RESET register never written"
    );
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
    // plausible-looking word (not all-zero / all-ones) but MAJVER = 0
    hi.interface.regs.insert((0, 0x0000), 0x0000_0005);

    assert!(matches!(
        hi.configure(&Config::default()),
        Err(ConfigError::BadVersion { major: 0, minor: 5 })
    ));
}

/// An all-zero `IDVER` read means the SPI bus itself is not working (no clock, stuck MISO,
/// wrong mode) - distinct from "wrong device".
#[test]
fn configure_flags_unresponsive_bus_all_zero() {
    setup!(hi, _lo, RegDouble::new(), NoopSignal);
    hi.interface.regs.insert((0, 0x0000), 0x0000_0000);
    assert!(matches!(
        hi.configure(&Config::default()),
        Err(ConfigError::BusUnresponsive)
    ));
}

/// Same for an all-ones `IDVER` read (MISO stuck high / floating).
#[test]
fn configure_flags_unresponsive_bus_all_ones() {
    setup!(hi, _lo, RegDouble::new(), NoopSignal);
    hi.interface.regs.insert((0, 0x0000), 0xFFFF_FFFF);
    assert!(matches!(
        hi.configure(&Config::default()),
        Err(ConfigError::BusUnresponsive)
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
#[cfg(feature = "embedded-hal")]
fn configure_through_fake_tc6() {
    use crate::embedded_io_implementation::EmbeddedHalImplementation;
    use fake_tc6::fake_tc6_device::FakeTc6SpiDevice;

    let interface = EmbeddedHalImplementation::new(FakeTc6SpiDevice::new());
    setup!(hi, _lo, interface, NoopSignal);

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
#[cfg(feature = "embedded-hal")]
fn configure_with_plca_through_fake_tc6() {
    use crate::embedded_io_implementation::EmbeddedHalImplementation;
    use fake_tc6::fake_tc6_device::FakeTc6SpiDevice;

    let interface = EmbeddedHalImplementation::new(FakeTc6SpiDevice::new());
    setup!(hi, _lo, interface, NoopSignal);

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
#[cfg(feature = "embedded-hal")]
fn control_header_parity_error_surfaced_through_fake_tc6() {
    use crate::embedded_io_implementation::EmbeddedHalImplementation;
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

/// End to end: `service_extended_status` reads `STATUS0` from `fake_tc6`, reports the set
/// error bit, write-1-to-clears it, and clears the pending latch.
#[test]
#[cfg(feature = "embedded-hal")]
fn service_extended_status_reads_and_clears_status0_through_fake_tc6() {
    use crate::embedded_io_implementation::EmbeddedHalImplementation;
    use fake_tc6::fake_tc6_device::FakeTc6SpiDevice;

    let mut spi = FakeTc6SpiDevice::new();
    // one bad control header makes the fake latch STATUS0.HDRE (bit 5); the flag is one-shot
    spi.device.force_bad_control_header = true;
    setup!(hi, _lo, EmbeddedHalImplementation::new(spi), NoopSignal);

    hi.begin_register_read(0, 0x0000, 1).expect("queued");
    hi.handle_control_complete().unwrap();
    let _ = hi.take_control_result();
    assert_ne!(
        hi.interface.inner_mut().device.read_reg(0, 0x0008) & (1 << 5),
        0,
        "fake latched STATUS0.HDRE"
    );

    // a data footer reported EXST=1 at some point
    hi.extended_status_pending = true;

    let s0 = hi
        .service_extended_status(10_000)
        .expect("no interface error")
        .expect("something was pending");
    assert!(s0.hdre(), "service read STATUS0 and saw HDRE");
    assert!(!hi.extended_status_pending(), "pending latch cleared");
    assert_eq!(
        hi.interface.inner_mut().device.read_reg(0, 0x0008) & (1 << 5),
        0,
        "HDRE write-1-cleared by the service"
    );

    // nothing pending now
    assert!(hi.service_extended_status(10_000).unwrap().is_none());
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
