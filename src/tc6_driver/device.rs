//! The low-priority half: `smoltcp::phy::Device` for `TC6LowPrio` and its tokens.

use super::*;

/// A received frame: a read grant on the receive ring. The record is released when the token
/// is consumed *or dropped*. bbqueue keeps a read grant's frame in the ring if the grant is
/// dropped without `release()`, so without the `Drop` impl a token smoltcp discards unconsumed
/// would deliver the same frame again.
pub struct TC6DriverRxToken<'s, const RX_N: usize> {
    frame: Option<FrameBuffer<RingGrantR<'s, RX_N>>>,
}

impl<const RX_N: usize> smoltcp::phy::RxToken for TC6DriverRxToken<'_, RX_N> {
    fn consume<R, F>(mut self, f: F) -> R
    where
        F: FnOnce(&[u8]) -> R,
    {
        let frame = self
            .frame
            .take()
            .expect("rx token holds a frame until consumed");
        let r = f(frame.as_slice());
        frame.into_grant().release();
        r
    }

    fn meta(&self) -> smoltcp::phy::PacketMeta {
        smoltcp::phy::PacketMeta::default()
    }
}

impl<const RX_N: usize> Drop for TC6DriverRxToken<'_, RX_N> {
    fn drop(&mut self) {
        if let Some(frame) = self.frame.take() {
            frame.into_grant().release();
        }
    }
}

/// Room for one outgoing frame: a worst-case (`TX_GRANT`) write grant on the transmit ring.
/// Dropping the token without consuming it aborts the grant, so nothing is sent.
pub struct TC6DriverTxToken<'a, 's, S, const TX_N: usize> {
    grant: RingGrantW<'s, TX_N>,
    // poked once the frame has been committed, so the high-prio side wakes up and pushes it
    // over SPI (see `TransmitSignal`)
    signal: &'a mut S,
}

impl<S, const TX_N: usize> smoltcp::phy::TxToken for TC6DriverTxToken<'_, '_, S, TX_N>
where
    S: TransmitSignal,
{
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let TC6DriverTxToken { grant, signal } = self;
        assert!(
            len <= ETHERNET_MTU,
            "tc6: smoltcp asked for a {}-byte frame, above the {}-byte MTU",
            len,
            ETHERNET_MTU
        );
        let mut frame = FrameBuffer::fresh(grant);
        // `fresh` is empty with start 0, so this exposes exactly `len` region bytes for `f`.
        frame.add_len(len);
        let r = f(frame.as_mut_slice());
        frame.write_meta();

        // Commit whole 64-byte chunks: the high-prio side always hands the interface a full
        // chunk window, so the padding past `len` must belong to this record.
        let used = (META_LEN + len.next_multiple_of(64)) as u16; // <= TX_GRANT
        frame.into_grant().commit(used);

        trace!("tc6: low-prio queued a {}-byte frame for transmit", len);
        // frame is committed: wake the high-prio side to service it
        signal.signal();
        r
    }
}

impl<'s, S, const RX_N: usize, const TX_N: usize> smoltcp::phy::Device
    for TC6LowPrio<'s, S, RX_N, TX_N>
where
    S: TransmitSignal,
{
    type RxToken<'a>
        = TC6DriverRxToken<'s, RX_N>
    where
        Self: 'a;
    type TxToken<'a>
        = TC6DriverTxToken<'a, 's, S, TX_N>
    where
        Self: 'a;

    fn capabilities(&self) -> smoltcp::phy::DeviceCapabilities {
        let mut cap = DeviceCapabilities::default();
        cap.medium = smoltcp::phy::Medium::Ethernet;
        // smoltcp's MTU is the L2 frame size *including* the 14-byte Ethernet header but
        // excluding the FCS (the MAC-PHY appends/strips the FCS itself, spec 7.3): 14 + 1500.
        cap.max_transmission_unit = ETHERNET_MTU;
        // How many frames smoltcp may hand us back-to-back before it must poll again. Small
        // frames pack much denser, but only worst-case frames are guaranteed: from any pointer
        // position an empty bip ring fits at least `(TX_N - 1) / TX_RECORD_MAX - 1` of them
        // (one record's worth can be lost to the wrap).
        cap.max_burst_size = Some(tx_burst_frames::<TX_N>());
        // The MAC-PHY only computes/checks the Ethernet FCS; IP/TCP/UDP checksums are all
        // software's job, which is exactly what `ChecksumCapabilities::default()` says.
        cap.checksum = smoltcp::phy::ChecksumCapabilities::default();
        cap
    }

    fn receive(
        &mut self,
        _timestamp: smoltcp::time::Instant,
    ) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        // smoltcp always wants a tx token alongside the rx one (e.g. to answer an ARP
        // request), so only report a received frame if a transmit grant is also available.
        // Take the transmit grant first: if there is no frame to receive it just aborts on drop.
        let grant = self.tx_producer.grant(TX_GRANT as u16).ok()?;
        let rx_frame = self.rx_consumer.read().ok()?;

        let rx_token = TC6DriverRxToken {
            frame: Some(FrameBuffer::from_record(rx_frame)),
        };
        let tx_token = TC6DriverTxToken {
            grant,
            signal: &mut self.transmit_signal,
        };
        Some((rx_token, tx_token))
    }

    fn transmit(&mut self, _timestamp: smoltcp::time::Instant) -> Option<Self::TxToken<'_>> {
        let Ok(grant) = self.tx_producer.grant(TX_GRANT as u16) else {
            trace!("tc6: transmit() returning None - transmit ring is full");
            return None;
        };
        Some(TC6DriverTxToken {
            grant,
            signal: &mut self.transmit_signal,
        })
    }
}

/// Worst-case frames an empty transmit ring of `TX_N` bytes is guaranteed to accept back to
/// back (at least 1, which `MIN_TX_RING` ensures).
pub(super) const fn tx_burst_frames<const TX_N: usize>() -> usize {
    let n = (TX_N - 1) / TX_RECORD_MAX - 1;
    if n == 0 { 1 } else { n }
}
