//! End to end through the real SPI wrapper and `fake_tc6`.

use super::*;

// --------------------------------------------------------------------------------------
// end-to-end through the real SPI wrapper + fake_tc6
// --------------------------------------------------------------------------------------

#[test]
#[cfg(feature = "embedded-hal")]
fn round_trip_transmit_through_fake_tc6() {
    use crate::embedded_io_implementation::EmbeddedHalImplementation;
    use fake_tc6::fake_tc6_device::FakeTc6SpiDevice;

    let i_face = EmbeddedHalImplementation::new(FakeTc6SpiDevice::new());
    setup!(hi, lo, i_face, NoopSignal);
    hi.sync_established = true; // data-path test: pretend configure() ran

    let payload = ethernet_like(64);
    {
        let tok = lo.transmit(Instant::ZERO).expect("tx token");
        tok.consume(payload.len(), |buf| buf.copy_from_slice(&payload));
    }

    assert!(hi.on_transmit_ready().is_ok());

    // get interface to the underlying fake device
    let got = hi
        .interface
        .inner_mut()
        .device
        .pop_transmit_frame()
        .expect("fake device received a frame");
    assert_eq!(got, payload);
    assert!(
        hi.interface
            .inner_mut()
            .device
            .pop_transmit_frame()
            .is_none(),
        "exactly one frame put on the wire"
    );
}

/// A fully blocking interface (`transfer_complete` always `true`) whose footer forever
/// reports `RCA` > 0 would drain without end; `on_interrupt` must bound itself at
/// `MAX_INTERRUPT_STEPS` and return so the caller (and lower-priority work) is not starved.
#[test]
fn on_interrupt_is_bounded_when_the_footer_always_reports_more_chunks() {
    /// Always "done", always "31 more receive chunks available", never any frame data.
    #[derive(Default)]
    struct AlwaysMoreRx {
        chunks_issued: u32,
    }
    impl AlwaysMoreRx {
        fn fill_footer(result: &mut (impl WriteBuffer<Word = u8> + ?Sized)) {
            let (ptr, len) = unsafe { result.write_buffer() };
            assert_eq!(len, 68);
            let buf = unsafe { core::slice::from_raw_parts_mut(ptr, len) };
            buf.fill(0);
            let footer = crate::tc6_footer::ReceiveFooter::new()
                .with_sync(true)
                .with_rca(31)
                .with_correct_parity();
            buf[64..68].copy_from_slice(&footer.to_bytes());
        }
    }
    impl Tc6Interface for AlwaysMoreRx {
        type Error = core::convert::Infallible;
        fn begin_send_block(
            &mut self,
            _h: u32,
            _d: impl ReadBuffer<Word = u8>,
            mut result: impl WriteBuffer<Word = u8>,
        ) -> Result<(), Self::Error> {
            self.chunks_issued += 1;
            Self::fill_footer(&mut result);
            Ok(())
        }
        fn begin_send_joined_block(
            &mut self,
            _d: impl ReadBuffer<Word = u8>,
            mut result: impl WriteBuffer<Word = u8>,
        ) -> Result<(), Self::Error> {
            self.chunks_issued += 1;
            Self::fill_footer(&mut result);
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
            unreachable!("no control traffic in this test")
        }
    }

    setup!(hi, _lo, AlwaysMoreRx::default(), NoopSignal);
    hi.sync_established = true; // data-path test: pretend configure() ran

    hi.on_interrupt().expect("returns rather than hanging");

    // 64 loop steps alternate issue-chunk / handle-completion -> ~half are chunks.
    let cap = TC6HighPrio::<AlwaysMoreRx, RX_N, TX_N>::MAX_INTERRUPT_STEPS;
    let issued = hi.interface.chunks_issued;
    assert!(
        issued > 0 && issued <= cap,
        "bounded by the {cap}-step cap, got {issued}"
    );
}
