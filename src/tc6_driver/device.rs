//! The low-priority half: `smoltcp::phy::Device` for `TC6LowPrio` and its tokens.

use super::*;

pub struct TC6DriverRxToken<RxP: BoxPool> {
    frame: Box<RxP>,
}

impl<RxP, const M: usize> smoltcp::phy::RxToken for TC6DriverRxToken<RxP>
where
    RxP: BoxPool<Data = FrameBuffer<M>> + 'static,
{
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&[u8]) -> R,
    {
        f(self.frame.as_slice())
    }

    fn meta(&self) -> smoltcp::phy::PacketMeta {
        smoltcp::phy::PacketMeta::default()
    }
}

pub struct TC6DriverTxToken<'a, TxP: BoxPool + 'static, S> {
    frame: Box<TxP>,
    queue: &'a mut Producer<'static, Box<TxP>>,
    // poked once the frame has been written and queued, so the high-prio side wakes up and
    // pushes it over SPI (see `TransmitSignal`)
    signal: &'a mut S,
}

impl<'a, TxP, S, const M: usize> smoltcp::phy::TxToken for TC6DriverTxToken<'a, TxP, S>
where
    TxP: BoxPool<Data = FrameBuffer<M>> + 'static,
    S: TransmitSignal,
{
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let TC6DriverTxToken {
            mut frame,
            queue,
            signal,
        } = self;
        // safety: we say that this is initialized, as we assume that FnOnce will write to the buffer
        let r = unsafe {
            frame.add_len(len);
            f(frame.as_mut_slice())
        };

        trace!("tc6: low-prio queued a {}-byte frame for transmit", len);
        queue.enqueue(frame).unwrap();
        // frame is queued: wake the high-prio side to service it
        signal.signal();
        r
    }
}

impl<RxP, TxP, S, const M: usize, const NQ: usize> smoltcp::phy::Device
    for TC6LowPrio<RxP, TxP, S, M, NQ>
where
    RxP: BoxPool<Data = FrameBuffer<M>> + 'static,
    TxP: BoxPool<Data = FrameBuffer<M>> + 'static,
    S: TransmitSignal,
{
    type RxToken<'a>
        = TC6DriverRxToken<RxP>
    where
        Self: 'a;
    type TxToken<'a>
        = TC6DriverTxToken<'a, TxP, S>
    where
        Self: 'a;

    fn capabilities(&self) -> smoltcp::phy::DeviceCapabilities {
        let mut cap = DeviceCapabilities::default();
        cap.medium = smoltcp::phy::Medium::Ethernet;
        // smoltcp's MTU is the L2 frame size *including* the 14-byte Ethernet header but
        // excluding the FCS (the MAC-PHY appends/strips the FCS itself, spec 7.3): 14 + 1500.
        cap.max_transmission_unit = ETHERNET_MTU;
        // How many frames smoltcp may hand us back-to-back before it must poll again. The
        // tx-ready SPSC queue holds `NQ - 1` (one slot is the ring's empty/full sentinel).
        cap.max_burst_size = Some(NQ - 1);
        // The MAC-PHY only computes/checks the Ethernet FCS; IP/TCP/UDP checksums are all
        // software's job, which is exactly what `ChecksumCapabilities::default()` says.
        cap.checksum = smoltcp::phy::ChecksumCapabilities::default();
        cap
    }

    fn receive(
        &mut self,
        _timestamp: smoltcp::time::Instant,
    ) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        // only report a received frame if a tx buffer will also be available (see paired-token
        // note: smoltcp always wants both, so this doubles as the anti-deadlock guard)
        if self.rx_ready_consumer.is_empty() {
            return None;
        }
        // TODO check if box can be created as well
        if !self.tx_ready_producer.ready() {
            return None;
        }

        // safety: we have already checked that the rx_ready_consumer is not empty, so dequeue is safe
        let rx_frame = self.rx_ready_consumer.dequeue().unwrap();
        let rx_token = TC6DriverRxToken { frame: rx_frame };

        // TODO make sure the compiler is smart and avoids a hugely expensive copy/move of the FrameBuffer struct here, since it is a large array. It should be optimized away by the compiler, but we should check.
        let tx_frame = TxP::alloc(FrameBuffer::new()).unwrap();
        let tx_token = TC6DriverTxToken {
            frame: tx_frame,
            queue: &mut self.tx_ready_producer,
            signal: &mut self.transmit_signal,
        };
        Some((rx_token, tx_token))
    }

    fn transmit(&mut self, _timestamp: smoltcp::time::Instant) -> Option<Self::TxToken<'_>> {
        if !self.tx_ready_producer.ready() {
            trace!("tc6: transmit() returning None - tx-ready queue is full");
            return None;
        }
        let tx_frame = TxP::alloc(FrameBuffer::new()).unwrap();
        Some(TC6DriverTxToken {
            frame: tx_frame,
            queue: &mut self.tx_ready_producer,
            signal: &mut self.transmit_signal,
        })
    }
}
