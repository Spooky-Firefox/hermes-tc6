//! Low-priority side (`smoltcp::phy::Device`).

use super::*;

// --------------------------------------------------------------------------------------
// low-priority side (smoltcp::phy::Device)
// --------------------------------------------------------------------------------------

#[test]
fn low_prio_reports_expected_capabilities() {
    setup!(_hi, lo, SpyInterface::default(), NoopSignal);
    let caps = lo.capabilities();
    assert_eq!(caps.medium, Medium::Ethernet);
    assert_eq!(
        caps.max_transmission_unit, 1514,
        "Ethernet header + 1500 payload, no FCS"
    );
    // 8192-byte ring, 1542-byte worst-case records: (8191 / 1542) - 1 guaranteed
    assert_eq!(
        caps.max_burst_size,
        Some(4),
        "worst-case frames the transmit ring always accepts"
    );
}

#[test]
fn transmit_returns_none_once_tx_ring_is_full() {
    setup!(_hi, lo, SpyInterface::default(), NoopSignal);
    let n = fill_tx_ring(&mut lo);
    assert!(
        n >= lo.capabilities().max_burst_size.unwrap(),
        "at least max_burst_size full-MTU frames fit, got {n}"
    );
    assert!(
        lo.transmit(Instant::ZERO).is_none(),
        "no token once the transmit ring is full"
    );
}

/// Small frames only take their own (chunk-padded) size, so far more than `max_burst_size`
/// of them fit.
#[test]
fn small_tx_frames_pack_densely() {
    setup!(_hi, lo, SpyInterface::default(), NoopSignal);
    let mut n = 0;
    // stop once a worst-case grant no longer fits (transmit() always asks for one)
    while let Some(tok) = lo.transmit(Instant::ZERO) {
        tok.consume(60, |_| {});
        n += 1;
    }
    // each 60-byte frame takes 2 + 4 + 64 = 70 ring bytes
    assert!(
        n > 80,
        "expected ~95 small frames in an 8 KiB ring, got {n}"
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
    assert!(commit_rx_frame(&mut hi, &ethernet_like(60)));
    // ...but smoltcp always wants a paired tx token, and the transmit ring is full
    fill_tx_ring(&mut lo);

    assert!(
        lo.receive(Instant::ZERO).is_none(),
        "receive must not hand out an rx token it can't pair with a tx token"
    );
    assert!(lo.rx_consumer.read().is_ok(), "the frame is still waiting");
}

/// smoltcp may drop an rx token without consuming it; the frame must be gone afterwards, not
/// delivered again (a dropped bbqueue read grant would otherwise keep it).
#[test]
fn dropped_rx_token_releases_its_frame() {
    setup!(hi, lo, SpyInterface::default(), NoopSignal);
    assert!(commit_rx_frame(&mut hi, &ethernet_like(60)));
    assert!(commit_rx_frame(&mut hi, &ethernet_like(70)));

    drop(lo.receive(Instant::ZERO).expect("first frame"));

    let (rx, _tx) = lo.receive(Instant::ZERO).expect("second frame");
    rx.consume(|b| assert_eq!(b, ethernet_like(70).as_slice()));
    assert!(lo.receive(Instant::ZERO).is_none());
}

/// A tx token dropped unconsumed commits nothing, and its space is reusable.
#[test]
fn dropped_tx_token_commits_nothing() {
    setup!(hi, lo, SpyInterface::default(), NoopSignal);
    for _ in 0..1000 {
        drop(lo.transmit(Instant::ZERO).expect("tx token"));
    }
    // the high-prio side sees no frame, and a worst-case grant still fits
    assert!(hi.tx_consumer.read().is_err());
    assert!(lo.tx_producer.grant(TX_GRANT as u16).is_ok());
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
    assert!(commit_rx_frame(&mut hi, &ethernet_like(60)));

    {
        let (rx, tx) = lo.receive(Instant::ZERO).expect("rx + tx tokens");
        rx.consume(|_| {});
        tx.consume(64, |b| b.copy_from_slice(&ethernet_like(64)));
    }
    assert_eq!(lo.transmit_signal.count, 1);
}
