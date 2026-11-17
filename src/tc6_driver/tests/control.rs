//! Control path: non-blocking command mode and blocking register access.

use super::*;

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

    hi.begin_register_write(0, 0x0004, &[0xDEAD_BEEF])
        .expect("accepted");

    let txn = hi.interface.last_control();
    let sent = tc6_header::ControlHeader::from_bytes(txn.tx[0..4].try_into().unwrap());
    assert!(!sent.dnc());
    assert!(sent.wnr(), "a write");
    assert_eq!(sent.addr(), 0x0004);
    assert_eq!(sent.len(), 0);
    assert!(sent.parity_ok());
    assert_eq!(
        &txn.tx[4..8],
        &0xDEAD_BEEFu32.to_be_bytes(),
        "data word follows header"
    );

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

    hi.interface
        .respond_control_read(&[0x0000_0011, 0x0000_0011, 0xAABB_CCDD]);
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

    hi.begin_register_read(0, 0x0002, 1)
        .expect("first accepted");
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
// blocking register access (read_register / write_register and friends)
// --------------------------------------------------------------------------------------

/// Works before `configure`: the control path doesn't need SYNC.
#[test]
#[cfg(feature = "embedded-hal")]
fn blocking_read_register_reads_idver_before_configure() {
    use crate::embedded_io_implementation::EmbeddedHalImplementation;
    use fake_tc6::fake_tc6_device::FakeTc6SpiDevice;

    setup!(
        hi,
        _lo,
        EmbeddedHalImplementation::new(FakeTc6SpiDevice::new()),
        NoopSignal
    );
    assert!(matches!(hi.read_register(0, 0x0000), Ok(0x0000_0011)));
    assert!(!hi.control_busy());
}

/// A two-register write lands in both registers, and one two-register read returns both.
#[test]
#[cfg(feature = "embedded-hal")]
fn blocking_multi_register_write_then_read_back() {
    use crate::embedded_io_implementation::EmbeddedHalImplementation;
    use fake_tc6::fake_tc6_device::FakeTc6SpiDevice;

    setup!(
        hi,
        _lo,
        EmbeddedHalImplementation::new(FakeTc6SpiDevice::new()),
        NoopSignal
    );
    // IMASK0 / IMASK1 are adjacent plain read-write registers in the fake.
    hi.write_registers(0, 0x000C, &[0x0000_0ABC, 0x0000_0123])
        .expect("write");
    let mut back = [0u32; 2];
    hi.read_registers(0, 0x000C, &mut back).expect("read");
    assert_eq!(back, [0x0000_0ABC, 0x0000_0123]);
    assert_eq!(hi.interface.inner_mut().device.read_reg(0, 0x000D), 0x123);
}

#[test]
fn blocking_register_functions_reject_bad_counts_without_a_transaction() {
    setup!(hi, _lo, SpyInterface::default(), NoopSignal);
    assert_eq!(
        hi.read_registers(0, 0, &mut []),
        Err(RegisterError::UnsupportedCount)
    );
    assert_eq!(
        hi.read_registers(0, 0, &mut [0; CONTROL_MAX_REGS + 1]),
        Err(RegisterError::UnsupportedCount)
    );
    assert_eq!(
        hi.write_registers(0, 0, &[]),
        Err(RegisterError::UnsupportedCount)
    );
    assert!(hi.interface.control_txns.is_empty());
}

/// A queued non-blocking request owns the control path until its result is taken.
#[test]
fn blocking_read_is_busy_while_a_queued_request_is_in_flight() {
    setup!(hi, _lo, SpyInterface::default(), NoopSignal);
    hi.begin_register_read(0, 0x0000, 1).expect("queued");
    assert_eq!(hi.read_register(0, 0x0000), Err(RegisterError::Busy));
    assert_eq!(hi.interface.control_txns.len(), 1, "no second transaction");
}

/// Starting a control transaction mid data chunk would lose that chunk's footer, so both
/// the register functions and `configure` refuse.
#[test]
fn blocking_access_is_busy_while_a_data_chunk_is_in_flight() {
    setup!(hi, lo, SpyInterface::default(), NoopSignal);
    hi.sync_established = true; // data-path test: pretend configure() ran
    let payload = ethernet_like(64);
    lo.transmit(Instant::ZERO)
        .expect("tx token")
        .consume(payload.len(), |buf| buf.copy_from_slice(&payload));
    hi.on_transmit_ready().unwrap();
    assert!(matches!(
        hi.driver_state,
        TC6DriverState::WaitingForTransmitComplete
    ));

    assert_eq!(hi.read_register(0, 0x0000), Err(RegisterError::Busy));
    assert_eq!(hi.write_register(0, 0x000C, 0), Err(RegisterError::Busy));
    assert_eq!(hi.configure(&Config::default()), Err(ConfigError::Busy));
    assert!(hi.interface.control_txns.is_empty());
}

#[test]
fn blocking_read_times_out_and_leaves_the_driver_idle() {
    setup!(hi, _lo, SpyInterface::default(), NoopSignal);
    hi.transfer_poll_limit = 10; // the spy never completes
    assert_eq!(hi.read_register(0, 0x0000), Err(RegisterError::Timeout));
    assert!(!hi.control_busy());
    assert!(matches!(hi.driver_state, TC6DriverState::Idle));
}

#[test]
#[cfg(feature = "embedded-hal")]
fn blocking_read_reports_a_rejected_header() {
    use crate::embedded_io_implementation::EmbeddedHalImplementation;
    use fake_tc6::fake_tc6_device::FakeTc6SpiDevice;

    let mut spi = FakeTc6SpiDevice::new();
    spi.device.force_bad_control_header = true;
    setup!(hi, _lo, EmbeddedHalImplementation::new(spi), NoopSignal);
    assert!(matches!(
        hi.read_register(0, 0x0000),
        Err(RegisterError::HeaderRejected)
    ));
    assert!(!hi.control_busy());
}

/// A bit flipped in the echoed write data (spec 7.4.4) is reported with its offset.
#[test]
fn blocking_write_reports_an_echo_mismatch() {
    setup!(hi, _lo, SpyInterface::default(), NoopSignal);
    hi.interface.control_reply = Some(|tx, rx| {
        // MISO = [don't care][echoed header][echoed data...]; corrupt the second word.
        let n = tx.len() / 4 - 2;
        rx[4..8 + 4 * n].copy_from_slice(&tx[..4 + 4 * n]);
        rx[8 + 4 + 3] ^= 0x01;
    });
    assert_eq!(
        hi.write_registers(0, 0x000C, &[0x11, 0x22]),
        Err(RegisterError::EchoMismatch { index: 1 })
    );
    assert!(!hi.control_busy());
}

#[test]
#[cfg(feature = "embedded-hal")]
fn configure_keeps_the_transfer_poll_limit_for_blocking_access() {
    use crate::embedded_io_implementation::EmbeddedHalImplementation;
    use fake_tc6::fake_tc6_device::FakeTc6SpiDevice;

    setup!(
        hi,
        _lo,
        EmbeddedHalImplementation::new(FakeTc6SpiDevice::new()),
        NoopSignal
    );
    assert_eq!(hi.transfer_poll_limit, DEFAULT_TRANSFER_POLL_LIMIT);
    let cfg = Config {
        transfer_poll_limit: 1234,
        ..Config::default()
    };
    hi.configure(&cfg).expect("bring-up");
    assert_eq!(hi.transfer_poll_limit, 1234);
}
