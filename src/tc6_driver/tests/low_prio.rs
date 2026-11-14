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
    assert_eq!(
        caps.max_burst_size,
        Some(NQ - 1),
        "usable tx-ready queue depth"
    );
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
