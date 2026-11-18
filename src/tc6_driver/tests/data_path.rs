//! High-priority data path: transmit, receive, multi-block transmit, the `begin_send_joined_block`
//! default implementation, and regression pins for once-broken paths.

use super::*;

// --------------------------------------------------------------------------------------
// high-priority side: transmit path
// --------------------------------------------------------------------------------------

#[test]
fn transmit_token_builds_one_full_data_chunk() {
    setup!(hi, lo, SpyInterface::default(), NoopSignal);
    hi.sync_established = true; // data-path test: pretend configure() ran

    let payload = ethernet_like(64);
    {
        let tok = lo.transmit(Instant::ZERO).expect("tx token");
        tok.consume(payload.len(), |buf| {
            assert_eq!(buf.len(), 64);
            buf.copy_from_slice(&payload);
        });
    }

    hi.on_transmit_ready().unwrap();

    assert_eq!(hi.interface.chunks.len(), 1, "exactly one chunk sent");
    let c = hi.interface.last();
    assert!(!c.joined);
    assert_eq!(&c.data[..], payload.as_slice());

    let h = tc6_header::TransmitHeader::from_raw(c.header);
    assert!(h.dnc(), "TC6 7.3.6: data chunks must set DNC");
    assert!(h.dv());
    assert!(h.sv());
    assert_eq!(h.swo(), 0);
    assert!(h.ev(), "a 64-byte frame ends within this block");
    assert_eq!(h.ebo(), 63);
    assert!(h.parity_ok());

    assert!(matches!(
        hi.driver_state,
        TC6DriverState::WaitingForTransmitComplete
    ));
}

/// A frame shorter than one 64-byte block (e.g. smoltcp's 42-byte ARP request) still goes
/// out as a single chunk; the bytes past the frame end are don't-care, EBO marks the last
/// valid one. Panicked on silicon before the first-chunk path stopped requiring a full
/// 64 initialized bytes.
#[test]
fn transmits_frame_shorter_than_one_block() {
    setup!(hi, lo, SpyInterface::default(), NoopSignal);
    hi.sync_established = true; // data-path test: pretend configure() ran

    let payload = ethernet_like(42);
    {
        let tok = lo.transmit(Instant::ZERO).expect("tx token");
        tok.consume(payload.len(), |buf| buf.copy_from_slice(&payload));
    }

    hi.on_transmit_ready().unwrap();

    assert_eq!(hi.interface.chunks.len(), 1, "exactly one chunk sent");
    let c = hi.interface.last();
    assert!(!c.joined);
    assert_eq!(&c.data[..42], payload.as_slice());

    let h = tc6_header::TransmitHeader::from_raw(c.header);
    assert!(h.dnc() && h.dv() && h.sv() && h.ev());
    assert_eq!(h.swo(), 0);
    assert_eq!(h.ebo(), 41, "last valid byte is at payload offset 41");
    assert!(h.parity_ok());
}

#[test]
fn transmit_completion_without_receive_data_returns_to_idle() {
    setup!(hi, lo, SpyInterface::default(), NoopSignal);
    hi.sync_established = true; // data-path test: pretend configure() ran

    {
        let tok = lo.transmit(Instant::ZERO).unwrap();
        tok.consume(64, |buf| buf.copy_from_slice(&ethernet_like(64)));
    }
    hi.on_transmit_ready().unwrap();

    // MAC-PHY says: transfer done, nothing to receive (DV = 0 footer).
    hi.interface.respond(
        &[],
        crate::tc6_footer::ReceiveFooter::new().with_correct_parity(),
    );
    hi.handle_transmit_complete().unwrap();

    assert!(matches!(hi.driver_state, TC6DriverState::Idle));
    assert_eq!(
        hi.interface.chunks.len(),
        1,
        "completion of a chunk must not itself emit another chunk"
    );
}

// --------------------------------------------------------------------------------------
// high-priority side: receive path
// --------------------------------------------------------------------------------------

#[test]
fn receives_single_block_frame_and_hands_it_to_low_prio() {
    setup!(hi, lo, SpyInterface::default(), NoopSignal);
    hi.sync_established = true; // data-path test: pretend configure() ran

    // no frame queued for tx -> handle_idle issues a receive-only ("joined") chunk
    hi.handle_idle().unwrap();
    {
        let c = hi.interface.last();
        assert!(c.joined);
        assert!(
            !tc6_header::TransmitHeader::from_raw(c.header).dv(),
            "receive-only chunk carries no tx data"
        );
    }

    let frame = ethernet_like(20);
    let footer = crate::tc6_footer::ReceiveFooter::new()
        .with_dv(true)
        .with_sv(true)
        .with_swo(0)
        .with_ev(true)
        .with_ebo(frame.len() as u8 - 1)
        .with_correct_parity();
    hi.interface.respond(&frame, footer);

    hi.handle_transmit_complete().unwrap();
    assert!(matches!(hi.driver_state, TC6DriverState::Idle));

    // the low-priority side should now see exactly that frame
    let (rx, _tx) = lo.receive(Instant::ZERO).expect("rx + tx tokens");
    rx.consume(|buf| assert_eq!(buf, frame.as_slice()));
    assert!(
        lo.receive(Instant::ZERO).is_none(),
        "rx-ready queue drained after one frame"
    );
}

/// When the end-of-frame footer has FD (Frame Drop) set the frame is discarded, not handed
/// to the low-priority side, and the receive buffer is freed for reuse (spec Section 7.3.7).
#[test]
fn receive_footer_frame_drop_discards_the_frame() {
    setup!(hi, lo, SpyInterface::default(), NoopSignal);
    hi.sync_established = true; // data-path test: pretend configure() ran

    hi.handle_idle().unwrap();

    let frame = ethernet_like(20);
    let footer = crate::tc6_footer::ReceiveFooter::new()
        .with_dv(true)
        .with_sv(true)
        .with_swo(0)
        .with_ev(true)
        .with_ebo(frame.len() as u8 - 1)
        .with_fd(true)
        .with_correct_parity();
    hi.interface.respond(&frame, footer);

    hi.handle_transmit_complete().unwrap();
    assert!(matches!(hi.driver_state, TC6DriverState::Idle));
    assert!(
        hi.current_receive_buffer.is_none(),
        "dropped frame's buffer was released"
    );
    assert!(
        lo.receive(Instant::ZERO).is_none(),
        "a dropped frame is never surfaced to smoltcp"
    );

    // the driver keeps running: the next chunk still gets a fresh receive buffer
    hi.handle_idle().unwrap();
    assert!(hi.current_receive_buffer.is_some());
}

/// A data-chunk footer that fails its parity check (a bit error on MISO, or the
/// `0xC0000001` header-error code the MAC-PHY streams per spec 7.5.1) is not trusted: the
/// transmit chunk is treated as not accepted, so the cursor does not advance and the same
/// first block is re-issued on the next `handle_idle`.
#[test]
fn corrupt_footer_reissues_the_transmit_chunk() {
    setup!(hi, lo, SpyInterface::default(), NoopSignal);
    hi.sync_established = true;

    let payload = ethernet_like(64);
    {
        let tok = lo.transmit(Instant::ZERO).expect("tx token");
        tok.consume(64, |b| b.copy_from_slice(&payload));
    }
    hi.on_transmit_ready().unwrap();
    assert_eq!(hi.interface.chunks.len(), 1);

    // footer with a broken parity bit (all-zero footer already fails odd parity)
    hi.interface.footer_sync = false;
    hi.interface
        .respond(&[], crate::tc6_footer::ReceiveFooter::new());
    hi.handle_transmit_complete().unwrap();

    assert!(matches!(hi.driver_state, TC6DriverState::Idle));
    assert!(
        hi.current_transmit_buffer.is_some(),
        "chunk not accepted - frame retained"
    );
    assert_eq!(hi.current_transmit_buffer_cursor, 0, "cursor not advanced");

    // next handle_idle re-issues the identical first block
    hi.handle_idle().unwrap();
    assert_eq!(hi.interface.chunks.len(), 2);
    let reissued = tc6_header::TransmitHeader::from_raw(hi.interface.last().header);
    assert!(reissued.dnc() && reissued.dv() && reissued.sv());
    assert_eq!(&hi.interface.last().data[..], &payload[..]);

    // a good footer this time -> frame is done
    hi.interface.footer_sync = true;
    hi.interface
        .respond(&[], crate::tc6_footer::ReceiveFooter::new());
    hi.handle_transmit_complete().unwrap();
    assert!(hi.current_transmit_buffer.is_none());
}

/// On a corrupt footer for a *continuation* block, the 4 real frame bytes that were borrowed
/// to carry the joined chunk header are restored and the cursor is left where it was, so the
/// same continuation block can be re-sent intact.
#[test]
fn corrupt_footer_on_continuation_block_restores_borrowed_bytes() {
    setup!(hi, lo, SpyInterface::default(), NoopSignal);
    hi.sync_established = true;

    let payload = ethernet_like(128); // two blocks
    {
        let tok = lo.transmit(Instant::ZERO).expect("tx token");
        tok.consume(payload.len(), |b| b.copy_from_slice(&payload));
    }

    // block 0 accepted
    hi.on_transmit_ready().unwrap();
    hi.interface.respond(
        &[],
        crate::tc6_footer::ReceiveFooter::new().with_correct_parity(),
    );
    hi.handle_transmit_complete().unwrap();
    assert_eq!(hi.current_transmit_buffer_cursor, 64);

    // block 1 (joined continuation) - borrows payload[60..64] for its header
    hi.handle_idle().unwrap();
    assert_eq!(&hi.scratch_header[..], &payload[60..64]);

    // corrupt footer for block 1
    hi.interface.footer_sync = false;
    hi.interface
        .respond(&[], crate::tc6_footer::ReceiveFooter::new());
    hi.handle_transmit_complete().unwrap();

    assert_eq!(hi.current_transmit_buffer_cursor, 64, "cursor held");
    assert!(hi.current_transmit_buffer.is_some());
    assert_eq!(
        &hi.current_transmit_buffer.as_ref().unwrap().as_slice()[60..64],
        &payload[60..64],
        "borrowed frame bytes restored"
    );
}

/// A data-chunk footer with EXST = 1 latches `extended_status_pending`; the DV payload in the
/// same footer is still processed.
#[test]
fn exst_footer_latches_pending_and_still_delivers_the_frame() {
    setup!(hi, lo, SpyInterface::default(), NoopSignal);
    hi.sync_established = true;

    hi.handle_idle().unwrap();
    assert!(!hi.extended_status_pending());

    let frame = ethernet_like(40);
    hi.interface.respond(
        &frame,
        crate::tc6_footer::ReceiveFooter::new()
            .with_exst(true)
            .with_dv(true)
            .with_sv(true)
            .with_swo(0)
            .with_ev(true)
            .with_ebo(frame.len() as u8 - 1)
            .with_correct_parity(),
    );
    hi.handle_transmit_complete().unwrap();

    assert!(hi.extended_status_pending(), "EXST latched");
    let (rx, _tx) = lo.receive(Instant::ZERO).expect("frame still delivered");
    rx.consume(|b| assert_eq!(b, frame.as_slice()));
}

/// SV=1 EV=1 with the end before the start on a *fresh* buffer: per spec 7.3.8.1 the EV ends a
/// frame we no longer hold (dropped earlier) and the SV starts a new one. The old frame's tail
/// is ignored and the new frame is kept, instead of losing both.
#[test]
fn concatenated_chunk_on_fresh_buffer_keeps_the_new_frame() {
    setup!(hi, lo, SpyInterface::default(), NoopSignal);
    hi.sync_established = true;

    let chunk0 = ethernet_like(64);
    hi.handle_idle().unwrap();
    hi.interface.respond(
        &chunk0,
        crate::tc6_footer::ReceiveFooter::new()
            .with_dv(true)
            .with_sv(true)
            .with_swo(10) // new frame starts at byte 40
            .with_ev(true)
            .with_ebo(3) // dropped frame's last byte at 3
            .with_correct_parity(),
    );
    hi.handle_transmit_complete().unwrap();
    assert!(lo.receive(Instant::ZERO).is_none(), "no frame complete yet");

    let tail: Vec<u8> = (0..20u8).map(|i| 200 + i).collect();
    hi.handle_idle().unwrap();
    hi.interface.respond(
        &tail,
        crate::tc6_footer::ReceiveFooter::new()
            .with_dv(true)
            .with_ev(true)
            .with_ebo(tail.len() as u8 - 1)
            .with_correct_parity(),
    );
    hi.handle_transmit_complete().unwrap();

    let mut expected = chunk0[40..].to_vec();
    expected.extend_from_slice(&tail);
    let (rx, _tx) = lo.receive(Instant::ZERO).expect("new frame delivered");
    rx.consume(|b| assert_eq!(b, expected.as_slice()));
}

/// Plays the MAC-PHY sending `frame` as receive-only chunks, starting `swo * 4` bytes into the
/// first chunk's payload.
fn feed_frame(hi: &mut TC6HighPrio<'_, SpyInterface, RX_N, TX_N>, frame: &[u8], swo: u8) {
    let start = swo as usize * 4;
    let mut sent = 0;
    while sent < frame.len() {
        let first = sent == 0;
        let offset = if first { start } else { 0 };
        let n = (64 - offset).min(frame.len() - sent);
        let mut payload = [0u8; 64];
        payload[offset..offset + n].copy_from_slice(&frame[sent..sent + n]);
        sent += n;
        let last = sent == frame.len();

        let mut footer = crate::tc6_footer::ReceiveFooter::new().with_dv(true);
        if first {
            footer = footer.with_sv(true).with_swo(swo);
        }
        if last {
            footer = footer.with_ev(true).with_ebo((offset + n - 1) as u8);
        }
        hi.handle_idle().unwrap();
        hi.interface.respond(&payload, footer.with_correct_parity());
        hi.handle_transmit_complete().unwrap();
    }
}

/// The largest frame the buffers are sized for, starting at the largest start offset
/// (SWO = 15, byte 60), is received whole. With the old 1600-byte minimum the last chunk's
/// receive window did not fit and `get_mut_at_end` panicked.
#[test]
fn max_size_frame_at_max_start_offset_is_received() {
    setup!(hi, lo, SpyInterface::default(), NoopSignal);
    hi.sync_established = true;

    for len in [1514, 1518, MAX_RX_FRAME] {
        let frame = ethernet_like(len);
        feed_frame(&mut hi, &frame, 15);
        let (rx, _tx) = lo.receive(Instant::ZERO).expect("frame delivered");
        rx.consume(|b| assert_eq!(b, frame.as_slice(), "len {len}"));
    }
}

/// A frame longer than the receive grant can hold is dropped (no panic), its remaining chunks
/// are discarded, and the next frame is received normally.
#[test]
fn oversize_frame_is_dropped_and_next_frame_received() {
    setup!(hi, lo, SpyInterface::default(), NoopSignal);
    hi.sync_established = true;

    feed_frame(&mut hi, &ethernet_like(MIN_FRAME_BUFFER_CAPACITY + 100), 0);
    assert!(
        lo.receive(Instant::ZERO).is_none(),
        "oversize frame not delivered"
    );

    let frame = ethernet_like(100);
    feed_frame(&mut hi, &frame, 0);
    let (rx, _tx) = lo.receive(Instant::ZERO).expect("next frame delivered");
    rx.consume(|b| assert_eq!(b, frame.as_slice()));
    assert!(lo.receive(Instant::ZERO).is_none());

    // no receive space leaked: nothing left committed, and once the driver lets go of its
    // in-progress grant the ring can hand out a worst-case grant again
    assert!(lo.rx_consumer.read().is_err(), "receive ring is empty");
    hi.current_receive_buffer = None;
    assert!(hi.rx_producer.grant(RX_GRANT as u16).is_ok());
}

/// With the receive ring full, `handle_idle` neither panics nor churns the bus when
/// there is nothing to transmit; when a frame *is* queued it still goes out with NORX set.
#[test]
fn rx_ring_full_defers_instead_of_panicking() {
    setup!(hi, lo, SpyInterface::default(), NoopSignal);
    hi.sync_established = true;

    // fill the receive ring with frames the low-prio side never picks up
    let frame = ethernet_like(MAX_RX_FRAME);
    let mut hog = 0;
    while commit_rx_frame(&mut hi, &frame) {
        hog += 1;
    }
    assert!(hog > 0);

    // nothing queued + no rx buffer: defer, stay Idle, issue no chunk
    hi.handle_idle().unwrap();
    assert!(matches!(hi.driver_state, TC6DriverState::Idle));
    assert!(hi.current_receive_buffer.is_none());
    assert!(hi.interface.chunks.is_empty());

    // a queued transmit still goes out, with NORX because inbound data can't be accepted
    {
        let tok = lo.transmit(Instant::ZERO).expect("tx token");
        tok.consume(64, |b| b.copy_from_slice(&ethernet_like(64)));
    }
    hi.on_transmit_ready().unwrap();
    assert_eq!(hi.interface.chunks.len(), 1);
    assert!(tc6_header::TransmitHeader::from_raw(hi.interface.last().header).norx());
}

#[test]
fn receive_only_chunk_targets_the_receive_buffer_tail() {
    setup!(hi, _lo, SpyInterface::default(), NoopSignal);
    hi.sync_established = true; // data-path test: pretend configure() ran

    hi.handle_idle().unwrap();

    // the MISO target must be inside the freshly allocated receive FrameBuffer, and 68
    // bytes wide (64 payload + 4 footer)
    let c = hi.interface.last();
    assert_eq!(c.result_len, 68);
    assert!(hi.current_receive_buffer.is_some());
}

// --------------------------------------------------------------------------------------
// interface: begin_send_joined_block default implementation
// --------------------------------------------------------------------------------------

#[test]
fn joined_block_default_impl_forwards_header_and_payload() {
    #[derive(Default)]
    struct Recording {
        last: Option<(u32, [u8; 64], usize)>,
    }
    impl Tc6Interface for Recording {
        type Error = core::convert::Infallible;
        fn begin_send_block(
            &mut self,
            header: u32,
            data: impl ReadBuffer<Word = u8>,
            mut result: impl WriteBuffer<Word = u8>,
        ) -> Result<(), Self::Error> {
            let (d_ptr, d_len) = unsafe { data.read_buffer() };
            assert_eq!(d_len, 64);
            let mut snap = [0u8; 64];
            snap.copy_from_slice(unsafe { core::slice::from_raw_parts(d_ptr, 64) });
            let (_r_ptr, r_len) = unsafe { result.write_buffer() };
            self.last = Some((header, snap, r_len));
            Ok(())
        }
        fn transfer_complete(&mut self) -> Result<bool, Self::Error> {
            Ok(true)
        }
        fn begin_control_transaction(
            &mut self,
            _tx: impl ReadBuffer<Word = u8>,
            _rx: impl WriteBuffer<Word = u8>,
        ) -> Result<(), Self::Error> {
            unreachable!("this test does not exercise control transactions")
        }
    }

    let header = tc6_header::TransmitHeader::new()
        .with_dnc(true)
        .with_dv(false)
        .with_correct_parity();

    let mut data = [0u8; 68];
    data[0..4].copy_from_slice(&header.to_bytes());
    for (i, b) in data[4..68].iter_mut().enumerate() {
        *b = i as u8;
    }
    let mut result = [0u8; 68];

    let mut i_face = Recording::default();
    i_face
        .begin_send_joined_block(RawBuf::new(&mut data[..]), RawBuf::new(&mut result[..]))
        .unwrap();

    let (h, payload, r_len) = i_face.last.expect("forwarded to begin_send_block");
    assert_eq!(h, header.raw());
    assert_eq!(r_len, 68);
    assert_eq!(payload, core::array::from_fn::<u8, 64, _>(|i| i as u8));
}

// --------------------------------------------------------------------------------------
// regression pins for known-broken paths (see module-level notes / PR discussion)
// --------------------------------------------------------------------------------------

/// A receive frame larger than one 64-byte block: the first block carries the start
/// (SV, no EV) and fills the whole chunk, the second carries the end (EV, no SV). The
/// driver must stitch them into one contiguous frame and hand it to the low-prio side.
///
/// (Previously pinned as `#[should_panic]`: `handle_data_valid` used to null
/// `current_receive_buffer` right before `if frame_done { ...take().unwrap() }`. That
/// line has since been removed, so this is now a positive test.)
#[test]
fn receives_frame_spanning_two_blocks() {
    setup!(hi, lo, SpyInterface::default(), NoopSignal);
    hi.sync_established = true; // data-path test: pretend configure() ran

    let head = ethernet_like(64);
    let tail: Vec<u8> = (0..20u8).map(|i| 200 + i).collect();

    // block 0: start of frame, fills the whole chunk (SV, no EV)
    hi.handle_idle().unwrap();
    hi.interface.respond(
        &head,
        crate::tc6_footer::ReceiveFooter::new()
            .with_dv(true)
            .with_sv(true)
            .with_swo(0)
            .with_correct_parity(),
    );
    hi.handle_transmit_complete().unwrap();
    assert!(
        hi.current_receive_buffer.is_some(),
        "frame unfinished, buffer retained for the next block"
    );

    // block 1: end of frame (EV, no SV)
    hi.handle_idle().unwrap();
    hi.interface.respond(
        &tail,
        crate::tc6_footer::ReceiveFooter::new()
            .with_dv(true)
            .with_ev(true)
            .with_ebo(tail.len() as u8 - 1)
            .with_correct_parity(),
    );
    hi.handle_transmit_complete().unwrap();

    let mut expected = head.clone();
    expected.extend_from_slice(&tail);

    let (rx, _tx) = lo.receive(Instant::ZERO).expect("rx + tx tokens");
    rx.consume(|buf| assert_eq!(buf, expected.as_slice()));
    assert!(
        lo.receive(Instant::ZERO).is_none(),
        "rx-ready queue drained"
    );
}

/// Once a single-block frame's chunk completes, `handle_transmit_complete` advances the
/// cursor past the frame length and drops `current_transmit_buffer`, so it is not re-sent.
/// The completing footer reports `RCA = 0` and nothing else is queued, so `on_interrupt`
/// stops there - it does not spin issuing receive polls.
#[test]
fn transmit_frame_is_sent_once_then_driver_stops_when_rca_zero() {
    setup!(hi, lo, SpyInterface::default(), NoopSignal);
    hi.sync_established = true; // data-path test: pretend configure() ran
    hi.tx_credits = 1;

    let payload = ethernet_like(64);
    {
        let tok = lo.transmit(Instant::ZERO).unwrap();
        tok.consume(64, |buf| buf.copy_from_slice(&payload));
    }
    hi.on_transmit_ready().unwrap();
    assert_eq!(hi.interface.chunks.len(), 1);
    assert!(!hi.interface.chunks[0].joined);

    // footer: DV = 0, RCA = 0, TXC = 0 -> nothing more for the chip to do
    hi.interface.respond(
        &[],
        crate::tc6_footer::ReceiveFooter::new().with_correct_parity(),
    );
    hi.on_interrupt().unwrap();

    assert!(
        hi.current_transmit_buffer.is_none(),
        "tx buffer freed once the whole frame is on the wire"
    );
    assert_eq!(hi.current_transmit_buffer_cursor, 0);
    assert_eq!(
        hi.interface.chunks.len(),
        1,
        "no extra receive poll - the drain loop stopped at RCA = 0"
    );
    assert!(matches!(hi.driver_state, TC6DriverState::Idle));
}

/// When the completing footer reports `RCA > 0` the drain loop keeps going: `on_interrupt`
/// issues a follow-up receive poll (joined chunk, DV = 0), not another copy of the transmit
/// frame. (The edge-triggered spy then reports the follow-up still in flight, so the call
/// returns; a real burst would drain over successive completion interrupts.)
#[test]
fn transmit_completion_with_rca_set_triggers_a_receive_poll() {
    setup!(hi, lo, SpyInterface::default(), NoopSignal);
    hi.sync_established = true; // data-path test: pretend configure() ran
    hi.tx_credits = 1;

    let payload = ethernet_like(64);
    {
        let tok = lo.transmit(Instant::ZERO).unwrap();
        tok.consume(64, |buf| buf.copy_from_slice(&payload));
    }
    hi.on_transmit_ready().unwrap();
    assert_eq!(hi.interface.chunks.len(), 1);

    // the chip still has a receive chunk buffered (RCA = 1)
    hi.interface.respond(
        &[],
        crate::tc6_footer::ReceiveFooter::new()
            .with_rca(1)
            .with_correct_parity(),
    );
    hi.on_interrupt().unwrap();

    assert!(hi.current_transmit_buffer.is_none(), "frame sent once");
    assert_eq!(
        hi.interface.chunks.len(),
        2,
        "RCA > 0 -> a follow-up receive poll was issued"
    );
    assert!(hi.interface.chunks[1].joined);
    assert!(!tc6_header::TransmitHeader::from_raw(hi.interface.chunks[1].header).dv());
    assert_eq!(hi.rx_chunks_available, 1, "RCA copied out of the footer");

    // drain the follow-up poll: footer now reports RCA = 0, so the loop settles
    hi.interface.respond(
        &[],
        crate::tc6_footer::ReceiveFooter::new().with_correct_parity(),
    );
    hi.on_interrupt().unwrap();
    assert_eq!(
        hi.interface.chunks.len(),
        2,
        "no further polls once RCA = 0"
    );
    assert!(matches!(hi.driver_state, TC6DriverState::Idle));
}

// --------------------------------------------------------------------------------------
// high-priority side: multi-block transmit
// --------------------------------------------------------------------------------------

/// A frame longer than 64 bytes goes out as: block 0 via `begin_send_block` (SV, no EV),
/// then one joined header+data chunk per following 64-byte block, the last with EV set.
/// Stepped by hand (`handle_idle` / `handle_transmit_complete`) so the test controls
/// exactly when each block is issued and completed.
#[test]
fn transmits_frame_spanning_two_blocks_as_block_then_joined_chunk() {
    setup!(hi, lo, SpyInterface::default(), NoopSignal);
    hi.sync_established = true; // data-path test: pretend configure() ran

    // 128 bytes == exactly two blocks
    let payload = ethernet_like(128);
    {
        let tok = lo.transmit(Instant::ZERO).expect("tx token");
        tok.consume(payload.len(), |buf| buf.copy_from_slice(&payload));
    }

    // block 0
    hi.on_transmit_ready().unwrap();
    {
        let c0 = &hi.interface.chunks[0];
        assert!(!c0.joined);
        let h = tc6_header::TransmitHeader::from_raw(c0.header);
        assert!(h.dnc() && h.dv() && h.sv());
        assert_eq!(h.swo(), 0);
        assert!(!h.ev(), "more blocks follow");
        assert!(h.parity_ok());
        assert_eq!(&c0.data[..], &payload[..64]);
    }
    hi.interface.respond(
        &[],
        crate::tc6_footer::ReceiveFooter::new().with_correct_parity(),
    );
    hi.handle_transmit_complete().unwrap();
    assert_eq!(hi.current_transmit_buffer_cursor, 64);
    assert!(hi.current_transmit_buffer.is_some());
    // the 4 bytes about to be borrowed for the next header are saved first
    let saved = &payload[60..64];

    // block 1 (continuation, joined chunk)
    hi.handle_idle().unwrap();
    assert_eq!(
        &hi.scratch_header[..],
        saved,
        "overwritten frame bytes stashed"
    );
    {
        let c1 = hi.interface.last();
        assert!(c1.joined);
        let h = tc6_header::TransmitHeader::from_raw(c1.header);
        assert!(h.dnc() && h.dv());
        assert!(!h.sv(), "a continuation block is not a frame start");
        assert!(h.ev());
        assert_eq!(h.ebo(), 63);
        assert!(h.parity_ok());
        assert_eq!(&c1.data[..], &payload[64..128]);
    }
    hi.interface.respond(
        &[],
        crate::tc6_footer::ReceiveFooter::new().with_correct_parity(),
    );
    hi.handle_transmit_complete().unwrap();

    assert!(
        hi.current_transmit_buffer.is_none(),
        "frame fully sent, buffer freed"
    );
    assert_eq!(hi.current_transmit_buffer_cursor, 0);

    // the two chunk payloads reassemble into the original frame
    let mut sent = Vec::new();
    sent.extend_from_slice(&hi.interface.chunks[0].data);
    sent.extend_from_slice(&hi.interface.chunks[1].data);
    assert_eq!(sent, payload);
}

/// Final block is partial (frame length not a multiple of 64). Per TC6 the chunk is
/// still a full fixed-size block; EBO marks the last valid byte and the chip ignores
/// the rest. The driver hands the interface `[cursor-4, cursor+64)`, whose tail sits in
/// not-yet-written `FrameBuffer` capacity here - fine for real hardware/DMA, and the
/// fake device drops everything past EBO.
#[test]
fn transmits_frame_with_partial_final_block_sets_ebo() {
    setup!(hi, lo, SpyInterface::default(), NoopSignal);
    hi.sync_established = true; // data-path test: pretend configure() ran

    let payload = ethernet_like(100); // 64 + 36
    {
        let tok = lo.transmit(Instant::ZERO).expect("tx token");
        tok.consume(payload.len(), |buf| buf.copy_from_slice(&payload));
    }

    // block 0
    hi.on_transmit_ready().unwrap();
    hi.interface.respond(
        &[],
        crate::tc6_footer::ReceiveFooter::new().with_correct_parity(),
    );
    hi.handle_transmit_complete().unwrap();

    // block 1 (partial: 36 bytes)
    hi.handle_idle().unwrap();
    let c1 = hi.interface.last();
    assert!(c1.joined);
    let h = tc6_header::TransmitHeader::from_raw(c1.header);
    assert!(h.dnc() && h.dv() && !h.sv() && h.ev());
    assert_eq!(h.ebo(), 35, "last valid byte is at payload offset 35");
    assert!(h.parity_ok());
    assert_eq!(&c1.data[..36], &payload[64..100]);

    hi.interface.respond(
        &[],
        crate::tc6_footer::ReceiveFooter::new().with_correct_parity(),
    );
    hi.handle_transmit_complete().unwrap();
    assert!(hi.current_transmit_buffer.is_none());
}

#[test]
#[cfg(feature = "embedded-hal")]
fn round_trip_multi_block_transmit_through_fake_tc6() {
    use crate::embedded_io_implementation::EmbeddedHalImplementation;
    use fake_tc6::fake_tc6_device::FakeTc6SpiDevice;

    let interface = EmbeddedHalImplementation::new(FakeTc6SpiDevice::new());
    setup!(hi, lo, interface, NoopSignal);
    hi.configure(&Config::default()).expect("bring-up");

    let payload = ethernet_like(192); // three blocks
    {
        let tok = lo.transmit(Instant::ZERO).expect("tx token");
        tok.consume(payload.len(), |buf| buf.copy_from_slice(&payload));
    }

    // Step three blocks by hand. `on_interrupt` can't be used here: this interface is
    // blocking so `transfer_complete()` is always true, and the loop only exits on the
    // `transfer_complete() == false` branch a real DMA interface would hit while a
    // transfer is in flight.
    for _ in 0..3 {
        hi.handle_idle().unwrap();
        hi.handle_transmit_complete().unwrap();
    }

    assert!(hi.current_transmit_buffer.is_none());
    let got = hi
        .interface
        .inner_mut()
        .device
        .pop_transmit_frame()
        .expect("fake device reassembled the frame");
    assert_eq!(got, payload);
}

/// Black-box round trip: no driver internals are touched. A frame is seeded on the
/// MAC-PHY (before it is handed to the driver), the IRQ handler is pumped via the public
/// `on_interrupt`, and the frame is collected through the public `smoltcp::phy::Device`
/// `receive`. A frame is also pushed the other way with `transmit` so both directions run
/// through `on_interrupt` (the outgoing side isn't asserted here - that would need a peek
/// at the interface).
#[test]
#[cfg(feature = "embedded-hal")]
fn black_box_round_trip_via_on_interrupt_and_device_receive() {
    use fake_tc6::fake_tc6_device::FakeTc6SpiDevice;

    let rx_frame: Vec<u8> = (0..128u32).map(|i| (i as u8) ^ 0x5A).collect();

    setup!(
        hi,
        lo,
        PollingFake::new(FakeTc6SpiDevice::new()),
        NoopSignal
    );
    hi.configure(&Config::default()).expect("bring-up");
    // seed the MAC-PHY with a frame to receive (after bring-up so the control traffic
    // doesn't run over it)
    hi.interface
        .inner
        .inner_mut()
        .device
        .enqueue_receive_frame(rx_frame.clone());

    // queue a frame to go out as well
    {
        let tok = lo.transmit(Instant::ZERO).expect("tx token");
        tok.consume(64, |buf| buf.copy_from_slice(&ethernet_like(64)));
    }

    // pump the IRQ handler; each call issues/finishes at most one chunk then returns
    for _ in 0..16 {
        hi.on_interrupt().unwrap();
    }

    // collect the received frame through the public Device API
    let (rx, _tx) = lo
        .receive(Instant::ZERO)
        .expect("driver surfaced the received frame");
    rx.consume(|buf| assert_eq!(buf, rx_frame.as_slice()));
    assert!(
        lo.receive(Instant::ZERO).is_none(),
        "only the one frame was received"
    );
}

// --------------------------------------------------------------------------------------
// frame rings: wrap-around, tailroom, full ring
// --------------------------------------------------------------------------------------

/// Plays the MAC-PHY for one queued transmit frame: issues chunks until the one with EV,
/// acknowledging each with a DV = 0 footer, and returns the frame bytes as they went out.
fn drain_one_tx_frame(hi: &mut TC6HighPrio<'_, SpyInterface, RX_N, TX_N>) -> Vec<u8> {
    let mut out = Vec::new();
    loop {
        hi.handle_idle().unwrap();
        let c = hi.interface.last();
        let h = tc6_header::TransmitHeader::from_raw(c.header);
        assert!(h.dv(), "expected a transmit chunk");
        if h.ev() {
            out.extend_from_slice(&c.data[..h.ebo() as usize + 1]);
        } else {
            out.extend_from_slice(&c.data);
        }
        hi.interface
            .respond(&[], crate::tc6_footer::ReceiveFooter::new());
        hi.handle_transmit_complete().unwrap();
        if h.ev() {
            return out;
        }
    }
}

fn patterned(len: usize, seed: usize) -> Vec<u8> {
    (0..len)
        .map(|i| (i.wrapping_mul(13) ^ seed.wrapping_mul(101)) as u8)
        .collect()
}

/// The transmit window always spans a full 64-byte chunk, so transmit records are committed
/// as whole chunks. Lengths around chunk boundaries go out intact, and `get_mut_at_index`'s
/// bounds check (the record's capacity) never fires.
#[test]
fn tx_lengths_around_chunk_boundaries_stay_inside_their_record() {
    setup!(hi, lo, SpyInterface::default(), NoopSignal);
    hi.sync_established = true;

    for (seed, len) in [1usize, 63, 64, 65, 127, 128, 129, 1513, 1514]
        .into_iter()
        .enumerate()
    {
        let frame = patterned(len, seed);
        lo.transmit(Instant::ZERO)
            .expect("tx token")
            .consume(len, |b| b.copy_from_slice(&frame));
        assert_eq!(drain_one_tx_frame(&mut hi), frame, "len {len}");
    }
}

/// Mixed-size transmit frames, several queued at a time, pushed until the ring has wrapped many
/// times. Every frame arrives intact and in order.
#[test]
fn tx_ring_wraps_without_corrupting_or_reordering_frames() {
    setup!(hi, lo, SpyInterface::default(), NoopSignal);
    hi.sync_established = true;

    let lens = [60usize, 1514, 342, 64, 900, 65, 1200, 42, 1514, 128];
    let mut next = 0usize;
    let mut sent = 0usize;
    let mut bytes = 0usize;
    while sent < 300 {
        // queue up to three frames, as smoltcp would in one poll
        let mut queued = Vec::new();
        for _ in 0..3 {
            let len = lens[next % lens.len()];
            let Some(tok) = lo.transmit(Instant::ZERO) else {
                break;
            };
            let frame = patterned(len, next);
            tok.consume(len, |b| b.copy_from_slice(&frame));
            queued.push(frame);
            next += 1;
        }
        assert!(!queued.is_empty(), "an empty ring always grants");
        for frame in queued {
            assert_eq!(drain_one_tx_frame(&mut hi), frame, "frame {sent}");
            bytes += frame.len();
            sent += 1;
        }
    }
    assert!(hi.tx_consumer.read().is_err(), "transmit ring drained");
    assert!(bytes > 20 * TX_N, "ring wrapped many times ({bytes} bytes)");
}

/// Mixed-size receive frames at varying start offsets, two held by the low-prio side at a
/// time, until the receive ring has wrapped many times.
#[test]
fn rx_ring_wraps_without_corrupting_or_reordering_frames() {
    setup!(hi, lo, SpyInterface::default(), NoopSignal);
    hi.sync_established = true;

    let lens = [60usize, 1514, 300, 64, 1522, 65, 1000, 42];
    let mut bytes = 0usize;
    for round in 0..120usize {
        let a = patterned(lens[(2 * round) % lens.len()], 2 * round);
        let b = patterned(lens[(2 * round + 1) % lens.len()], 2 * round + 1);
        feed_frame(&mut hi, &a, (round % 16) as u8);
        feed_frame(&mut hi, &b, ((round * 7) % 16) as u8);
        for expected in [a, b] {
            let (rx, _tx) = lo.receive(Instant::ZERO).expect("frame delivered");
            rx.consume(|got| assert_eq!(got, expected.as_slice(), "round {round}"));
            bytes += expected.len();
        }
    }
    assert!(lo.receive(Instant::ZERO).is_none());
    assert!(bytes > 10 * RX_N, "ring wrapped many times ({bytes} bytes)");
}

/// Once the low-prio side frees space in a full receive ring, the next `handle_idle` gets a
/// receive grant again and a receive-only chunk goes out (no NORX).
#[test]
fn rx_ring_full_recovers_once_low_prio_drains() {
    setup!(hi, lo, SpyInterface::default(), NoopSignal);
    hi.sync_established = true;

    let frame = ethernet_like(MAX_RX_FRAME);
    while commit_rx_frame(&mut hi, &frame) {}

    hi.handle_idle().unwrap();
    assert!(hi.current_receive_buffer.is_none(), "ring full: no grant");
    assert!(hi.interface.chunks.is_empty(), "and no bus churn");

    // drain everything the ring held
    while let Some((rx, _tx)) = lo.receive(Instant::ZERO) {
        rx.consume(|b| assert_eq!(b, frame.as_slice()));
    }

    hi.handle_idle().unwrap();
    assert!(hi.current_receive_buffer.is_some(), "grant available again");
    let c = hi.interface.last();
    let h = tc6_header::TransmitHeader::from_raw(c.header);
    assert!(c.joined && !h.dv() && !h.norx(), "plain receive-only chunk");
}
