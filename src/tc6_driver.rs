use crate::tc6_footer::ReceiveFooter;
use crate::tc6_interface::Tc6Interface;
use crate::{frame_buffer::FrameBuffer, tc6_footer, tc6_header};
use embedded_dma::{ReadBuffer, WriteBuffer};
use heapless::pool::boxed::{Box, BoxPool};
use heapless::spsc::{Consumer, Producer, Queue};
use smoltcp::phy::DeviceCapabilities;

// NOTE: `box_pool!` singletons must name a concrete type, so `RxP`/`TxP` (and the `M` they were
// built with) have to be declared by the application, not by this generic driver. Pass the
// resulting pool types + `'static` queue storage into `split()` below.

// TODO, is this a good idea? this was a hack to avoid the lifetime issues with passing &mut [u8] to the tc6 interface
struct DMAPtr {
    ptr: *mut u8,
    len: usize,
}

impl DMAPtr {
    fn from_slice(slice: &mut [u8]) -> Self {
        Self {
            ptr: slice.as_mut_ptr(),
            len: slice.len(),
        }
    }
}

unsafe impl WriteBuffer for DMAPtr {
    type Word = u8;

    unsafe fn write_buffer(&mut self) -> (*mut Self::Word, usize) {
        (self.ptr, self.len)
    }
}

unsafe impl ReadBuffer for DMAPtr {
    type Word = u8;

    unsafe fn read_buffer(&self) -> (*const Self::Word, usize) {
        (self.ptr, self.len)
    }
}

enum TC6DriverState {
    Idle,
    WaitingForTransmitComplete,
}

/// Implemented by the application to wake the high-priority side when the low-priority side has
/// queued a frame to transmit (e.g. pend a software interrupt, or spawn an RTIC task that ends up
/// calling `TC6HighPrio::on_transmit_ready`). Without this, a pure-transmit workload would never
/// get serviced, since `on_interrupt` only fires on chip-initiated events.
pub trait TransmitSignal {
    fn signal(&mut self);
}

/// Owned by the high-priority context (e.g. the TC6 IRQ task): services the chip over SPI.
pub struct TC6HighPrio<T, RxP, TxP, const M: usize, const NQ: usize>
where
    T: Tc6Interface,
    RxP: BoxPool<Data = FrameBuffer<M>> + 'static,
    TxP: BoxPool<Data = FrameBuffer<M>> + 'static,
{
    interface: T,
    current_transmit_buffer: Option<Box<TxP>>, // the frame currently being transmitted, if any
    current_receive_buffer: Option<Box<RxP>>,  // the frame currently being received, if any
    // todo, do we need Option, can we use empty vec to indicate no data?
    extra_start_buffer: Option<heapless::Vec<u8, 64>>, // used to store the start of a receive frame if the start was written to the end of the previous framebuffer
    driver_state: TC6DriverState,
    current_transmit_buffer_cursor: usize, // keeps track of how much have been written to the tc6 interface, so we know what to transfer in the next block
    scratch_header: [u8; 4], // used for temporary storage of the header in the case where a block needs to be moved
    scratch_buffer: [u8; 68], // used for temporary storage of a block in the case where a block needs to be moved
    rx_ready_producer: Producer<'static, Box<RxP>>, // frames pulled off the wire, handed to the low-prio side
    tx_ready_consumer: Consumer<'static, Box<TxP>>, // frames built by the low-prio side, waiting to go out over SPI
}

/// Owned by the low-priority context (e.g. `idle`): implements `smoltcp::phy::Device`.
pub struct TC6LowPrio<RxP, TxP, S, const M: usize, const NQ: usize>
where
    RxP: BoxPool<Data = FrameBuffer<M>> + 'static,
    TxP: BoxPool<Data = FrameBuffer<M>> + 'static,
    S: TransmitSignal,
{
    rx_ready_consumer: Consumer<'static, Box<RxP>>,
    tx_ready_producer: Producer<'static, Box<TxP>>,
    transmit_signal: S, // poked whenever a frame is pushed onto `tx_ready_producer`
}

/// Splits the driver into its high-priority (IRQ) and low-priority (smoltcp) halves.
///
/// `rx_ready`/`tx_ready` need `'static` storage from the caller (e.g. `static_cell` or a
/// `static mut` in `#[init]`). `RxP`/`TxP` must already have their memory blocks `manage`d.
pub fn split<T, RxP, TxP, S, const M: usize, const NQ: usize>(
    interface: T,
    rx_ready: &'static mut Queue<Box<RxP>, NQ>,
    tx_ready: &'static mut Queue<Box<TxP>, NQ>,
    transmit_signal: S,
) -> (
    TC6HighPrio<T, RxP, TxP, M, NQ>,
    TC6LowPrio<RxP, TxP, S, M, NQ>,
)
where
    T: Tc6Interface,
    RxP: BoxPool<Data = FrameBuffer<M>> + 'static,
    TxP: BoxPool<Data = FrameBuffer<M>> + 'static,
    S: TransmitSignal,
{
    let (rx_ready_producer, rx_ready_consumer) = rx_ready.split();
    let (tx_ready_producer, tx_ready_consumer) = tx_ready.split();

    (
        TC6HighPrio {
            interface,
            driver_state: TC6DriverState::Idle,
            current_transmit_buffer_cursor: 0,
            scratch_header: [0; 4],
            scratch_buffer: [0; 68],
            current_transmit_buffer: None,
            current_receive_buffer: None,
            rx_ready_producer,
            tx_ready_consumer,
            extra_start_buffer: None,
        },
        TC6LowPrio {
            rx_ready_consumer,
            tx_ready_producer,
            transmit_signal,
        },
    )
}

impl<T, RxP, TxP, const M: usize, const NQ: usize> TC6HighPrio<T, RxP, TxP, M, NQ>
where
    T: Tc6Interface,
    RxP: BoxPool<Data = FrameBuffer<M>> + 'static,
    TxP: BoxPool<Data = FrameBuffer<M>> + 'static,
{
    // Call this from every interrupt source the chip/interface can raise: the TC6 chip's own IRQ
    // pin *and* whatever signals SPI/DMA transfer completion (often a separate vector). This is
    // re-entrant/idempotent — it just re-checks `driver_mode` / `interface.transfer_complete()`
    // rather than assuming which source triggered it, so spurious or overlapping calls are fine.
    pub fn on_interrupt(&mut self) -> Result<(), T::Error> {
        // we might have a frame to receive
        loop {
            match self.driver_state {
                TC6DriverState::Idle => self.handle_idle()?,
                TC6DriverState::WaitingForTransmitComplete => {
                    if self.interface.transfer_complete()? {
                        self.handle_transmit_complete()?;
                    } else {
                        break;
                    }
                }
            }
        }
        Ok(())
    }

    // This should be called whenever the low-prio side may have queued a frame to transmit:
    // either from whatever `TransmitSignal::signal` triggers, or from periodic polling if the
    // caller would rather not wire up a signal at all.
    pub fn on_transmit_ready(&mut self) -> Result<(), T::Error> {
        if !matches!(self.driver_state, TC6DriverState::Idle) {
            // still waiting for the previous transfer to complete, so we can't start a new one yet
            return Ok(());
        }
        self.handle_idle()?;
        Ok(())
    }

    fn handle_transmit_complete(&mut self) -> Result<(), T::Error> {
        let footer;
        {
            let bytes = if let (Some(_transmit_frame), Some(receive_frame)) = (
                self.current_transmit_buffer.as_mut(),
                self.current_receive_buffer.as_mut(),
            ) {
                // read the footer
                // Safety: before this function can be called, we have started a block send and the transfer has completed,
                // so the tc6 chip should have populated the 68 bytes at the end of the receive frame
                // that we sent in handle idle, so we can read the footer from the receive frame
                unsafe { receive_frame.get_mut_at_end(68) }
            } else if self.current_transmit_buffer.as_mut().is_some() {
                // read the footer
                &mut self.scratch_buffer
            } else if let Some(receive_frame) = self.current_receive_buffer.as_mut() {
                // read the footer
                // Safety: before this function can be called, we have started a block send and the transfer has completed,
                // so the tc6 chip should have populated the 68 bytes at the end of the receive frame
                // that we sent in handle idle, so we can read the footer from the receive frame
                unsafe { receive_frame.get_mut_at_end(68) }
            } else {
                unreachable!(
                    "how did we get here? We can't have a transfer complete without either the goal of reading (Some(ReceiveFrame)), or writing (Some(TransmitFrame)) or both, so this should not happen"
                )
            };

            if self.current_transmit_buffer.is_some() {
                let cursor = self.current_transmit_buffer_cursor;
                // A continuation block (cursor != 0) was sent as a joined header+data chunk,
                // which meant overwriting the 4 real frame bytes at [cursor-4, cursor) with
                // the chunk header. Those bytes were already transmitted in the previous
                // block, but restore them from `scratch_header` so the frame stays intact
                // (needed if it ever has to be retransmitted).
                if cursor != 0 {
                    let frame = self.current_transmit_buffer.as_mut().unwrap();
                    frame.as_mut_slice()[cursor - 4..cursor].copy_from_slice(&self.scratch_header);
                }

                self.current_transmit_buffer_cursor += 64;
                // if we have reached the end of the current transmit buffer drop/free the transmit box
                if self.current_transmit_buffer_cursor
                    >= self
                        .current_transmit_buffer
                        .as_ref()
                        .unwrap()
                        .as_slice()
                        .len()
                {
                    self.current_transmit_buffer = None;
                    self.current_transmit_buffer_cursor = 0;
                }
            }

            footer = tc6_footer::ReceiveFooter::from_bytes(bytes[64..68].try_into().unwrap());
        }
        if footer.dv() {
            self.handle_data_valid(footer)?;
        }
        self.driver_state = TC6DriverState::Idle;
        Ok(())
    }

    fn handle_data_valid(&mut self, footer: ReceiveFooter) -> Result<(), T::Error> {
        // if we did not have a receive frame, then we have sent the norx flag with the header,
        // and therefore the tc6 should not have set dv.
        // but if it did either we messed up and did not set the norx flag, or the tc6 chip is misbehaving, either way we should panic.
        assert!(self.current_receive_buffer.is_some());
        let receive_frame = self.current_receive_buffer.as_mut().unwrap();
        // if receive frame is empty,
        let frame_done: bool;
        if receive_frame.as_slice().is_empty() {
            if footer.sv() {
                unsafe { receive_frame.set_start((footer.swo() << 2) as usize) };
                if footer.ev() {
                    unsafe {
                        receive_frame.add_len(footer.ebo() as usize + 1 - footer.swo() as usize * 4)
                    }
                    frame_done = true;
                } else {
                    unsafe { receive_frame.add_len(64 - footer.swo() as usize * 4) };
                    frame_done = false;
                }
            } else {
                panic!(
                    "we are expecting a start valid bit, as data valid is set and we are beginning a new frame (indicated by empty receive frame), but the start valid bit is not set, this should not happen"
                );
            }
        } else {
            if footer.ev() && footer.sv() {
                // if for some reason a new frame is started before the previous one is finished, we will panic, as this should not happen
                assert!(footer.ebo() < footer.swo() * 4);
            }
            // note this is only valid if ev is set
            if footer.sv() {
                assert!(footer.ev());

                // TODO possible optimization is to not write to temp buffer, and get box immediately.
                self.extra_start_buffer = Some(
                    heapless::Vec::from_slice(
                        // would want to use bytes here but due to lifetimes i had to move it out of scope, so we have an extra unsafe block here
                        &unsafe { receive_frame.get_mut_at_end(64) }[footer.swo() as usize * 4..],
                    )
                    .unwrap(),
                );
            }

            if footer.ev() {
                unsafe { receive_frame.add_len(footer.ebo() as usize + 1) }
                frame_done = true;
            } else {
                // dv is set, but ev and sv not set so whole block is valid and it's ok to add the whole block to the receive frame
                unsafe { receive_frame.add_len(64) }
                frame_done = false;
            }
        }

        if frame_done {
            self.rx_ready_producer
                .enqueue(self.current_receive_buffer.take().unwrap())
                .unwrap();
        }
        Ok(())
    }

    fn handle_idle(&mut self) -> Result<(), T::Error> {
        // do we have a receive buffer ready to fill? if not allocate one from the pool and queue it up for the tc6 chip to fill
        if self.current_receive_buffer.is_none() {
            // TODO make sure the compiler is smart and avoids a hugely expensive copy/move of the FrameBuffer struct here, since it is a large array. It should be optimized away by the compiler, but we should check.
            self.current_receive_buffer = RxP::alloc(FrameBuffer::new()).ok();
            if let Some(buff) = self.extra_start_buffer.as_ref() {
                self.current_receive_buffer
                    .as_mut()
                    .unwrap()
                    .push_slice(buff.as_slice());
            }
            // TODO, maybe a option::take() would be better, but doing this way as im not sure if take would clone the inner vec.
            self.extra_start_buffer = None;
        }

        if self.current_transmit_buffer.is_none() {
            // do we have a transmit buffer ready to send? if not, try to get one from the low-prio side
            self.current_transmit_buffer = self.tx_ready_consumer.dequeue();
            self.current_transmit_buffer_cursor = 0;
        }

        // if we have a transmit buffer ready send that
        if let Some(frame) = self.current_transmit_buffer.as_mut() {
            let len = frame.as_slice()[self.current_transmit_buffer_cursor..].len();
            let mut header = tc6_header::TransmitHeader::new();
            // TC6 spec 7.3.6: DNC shall be 1 for every transmit data chunk (0 = control command).
            header = header.with_dnc(true);
            header = header.with_dv(true);

            // end valid, if current block fits the last bit of the frame,
            let ev = len <= 64;
            header = header.with_ev(ev);
            // end byte offset, if current block fits the last bit of the frame,
            // then which byte in the frame is the last byte
            // -1, because zero indexed, and the end byte offset is inclusive, so if the frame is 1 byte long, the end byte offset is 0
            let ebo: u8 = if ev { len as u8 - 1 } else { 0 };
            header = header.with_ebo(ebo);

            if self.current_transmit_buffer_cursor == 0 {
                // we have no space we can use to write a header, so we can not use the more efficient write joined
                // also this is the start of data transmission, sv = 1, swo = 0.
                header = header.with_sv(true);
                header = header.with_swo(0);
                //safety: we are reading from a valid index, we might read more than what is initialized, but the swo and ebo will tell the tc6 chip how much of the data is valid, so it will ignore the rest
                let data = unsafe {
                    DMAPtr::from_slice(
                        frame.get_raw_mut_from_index(self.current_transmit_buffer_cursor, 64),
                    )
                };

                // we can't receive any data due to some reason,(like full receive box pool)
                if self.current_receive_buffer.is_none() {
                    header = header.with_norx(true)
                }

                let result = if let Some(f) = self.current_receive_buffer.as_mut() {
                    // SAFETY, we are writing to a point which is outside what is considered initialized, and we are not marking this part as initialized,
                    // that is done when we read the footer returned from the tc6 chip, and then we can determine what is valid and update the length.
                    unsafe { DMAPtr::from_slice(f.get_mut_at_end(68)) }
                } else {
                    DMAPtr::from_slice(&mut self.scratch_buffer)
                };
                header = header.with_correct_parity();
                let header_raw: u32 = header.raw();
                self.interface.begin_send_block(header_raw, data, result)?;
                self.driver_state = TC6DriverState::WaitingForTransmitComplete;
            } else {
                // Continuation block: the 4 frame bytes just before the cursor were already
                // sent in the previous block, so we can borrow them to hold this chunk's
                // header and send header + 64 data bytes as one joined transfer. `header`
                // (dnc/dv/ev/ebo) is already set correctly above; a continuation block is
                // never a frame start, so sv/swo stay 0.
                let cursor = self.current_transmit_buffer_cursor;
                let header = header.with_correct_parity();

                // stash the real bytes we're about to overwrite so the frame can be restored
                // once this transfer completes (see handle_transmit_complete)
                self.scratch_header
                    .copy_from_slice(&frame.as_slice()[cursor - 4..cursor]);
                frame.as_mut_slice()[cursor - 4..cursor]
                    .copy_from_slice(&header.raw().to_be_bytes());

                // SAFETY: the window is [cursor-4, cursor+64); on the final (partial) block
                // its tail runs past the initialized frame data into spare capacity - the
                // chip ignores those bytes because ebo marks the last valid one.
                let data = DMAPtr::from_slice(unsafe { frame.get_mut_at_index(cursor - 4, 68) });
                let result = if let Some(f) = self.current_receive_buffer.as_mut() {
                    // SAFETY, we are writing to a point which is outside what is considered initialized, and we are not marking this part as initialized,
                    // that is done when we read the footer returned from the tc6 chip, and then we can determine what is valid and update the length.
                    unsafe { DMAPtr::from_slice(f.get_mut_at_end(68)) }
                } else {
                    DMAPtr::from_slice(&mut self.scratch_buffer)
                };
                self.interface.begin_send_joined_block(data, result)?;
                self.driver_state = TC6DriverState::WaitingForTransmitComplete;
            }
        } else if let Some(f) = self.current_receive_buffer.as_mut() {
            // irq fired, we have data to be read
            let mut header = tc6_header::TransmitHeader::new();
            // still a data chunk (DNC = 1), we just have nothing to send, so dv = 0
            header = header.with_dnc(true);
            header = header.with_dv(false);
            header = header.with_correct_parity();
            self.scratch_buffer[0..4].copy_from_slice(&header.raw().to_be_bytes());
            let data = DMAPtr::from_slice(&mut self.scratch_buffer);
            // SAFETY, we are writing to a point which is outside what is considered initialized, and we are not marking this part as initialized,
            // that is done when we read the footer returned from the tc6 chip, and then we can determine what is valid and update the length.
            let result = unsafe { DMAPtr::from_slice(f.get_mut_at_end(68)) };
            self.interface.begin_send_joined_block(data, result)?;
            self.driver_state = TC6DriverState::WaitingForTransmitComplete;
        } else {
            // irq fired, but we have no data to send nor space to receive data
            todo!("what should we do here?")
        }
        Ok(())
    }
}

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
        cap.max_burst_size = Some(NQ);
        // set default checksum capabilities, which is calculated and verified by software (TODO find out if this is the case, and that i have not mixed it up)
        cap.checksum = smoltcp::phy::ChecksumCapabilities::default();
        cap.max_transmission_unit = M; //TODO actually determine what the correct MTU should be for a ethernet frame
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

#[cfg(test)]
mod driver_tests {
    //! Tests for the split TC6 driver.
    //!
    //! Two kinds of test double are used:
    //!
    //! * [`SpyInterface`] - a hand-rolled [`Tc6Interface`] that records every chunk the
    //!   driver pushes over "SPI" and lets the test play the MAC-PHY side: write a footer
    //!   (+ optional receive payload) back into the MISO buffer of the last chunk and raise
    //!   "transfer complete". This is the "custom tc6 interface to track what's sent/read".
    //! * [`fake_tc6`]'s `FakeTc6SpiDevice` behind the real [`EmbeddedHalImplementation`]
    //!   SPI wrapper, for an end-to-end round trip through the actual wire-format code.
    //!
    //! Both halves returned by [`split`] are deconstructed directly (their producer/consumer
    //! and internal state-machine fields are reachable from this child module) so a test can
    //! drive one side and observe the other.
    //!
    //! Pool plumbing uses `heapless`'s box pool: <https://docs.rs/heapless/latest/heapless/pool/boxed/index.html>

    use super::*;
    use heapless::pool::boxed::BoxBlock;
    use smoltcp::phy::{Device, Medium, RxToken, TxToken};
    use smoltcp::time::Instant;
    use std::vec::Vec;

    const M: usize = 1536; // FrameBuffer capacity (bytes)
    const NQ: usize = 4; // spsc queue const param; usable capacity is NQ - 1 == 3

    // --------------------------------------------------------------------------------------
    // test doubles
    // --------------------------------------------------------------------------------------

    struct NoopSignal;
    impl TransmitSignal for NoopSignal {
        fn signal(&mut self) {}
    }

    /// Counts how many times the low-prio side asked to wake the high-prio side.
    #[derive(Default)]
    struct CountingSignal {
        count: usize,
    }
    impl TransmitSignal for CountingSignal {
        fn signal(&mut self) {
            self.count += 1;
        }
    }

    /// One chunk the driver handed to the interface.
    struct Chunk {
        header: u32,
        /// snapshot of the 64 MOSI payload bytes
        data: [u8; 64],
        /// true if it came in via `begin_send_joined_block`
        joined: bool,
        /// where the driver wants the MISO bytes (64 data + 4 footer) written back
        result_ptr: *mut u8,
        result_len: usize,
    }

    #[derive(Default)]
    struct SpyInterface {
        chunks: Vec<Chunk>,
        complete: bool,
    }

    impl SpyInterface {
        fn last(&self) -> &Chunk {
            self.chunks.last().expect("no chunk sent yet")
        }

        /// Pretend the MAC-PHY filled in the MISO buffer of the most recent chunk:
        /// `payload` at the front (received frame bytes), `footer` in the trailing 4 bytes,
        /// and mark the transfer complete.
        fn respond(&mut self, payload: &[u8], footer: crate::tc6_footer::ReceiveFooter) {
            assert!(payload.len() <= 64);
            let c = self.chunks.last().expect("no chunk sent yet");
            assert_eq!(c.result_len, 68, "MISO buffer is 64B data + 4B footer");
            // SAFETY: `result_ptr` came from `WriteBuffer::write_buffer` of the buffer the
            // driver handed us for this chunk (the tail of the in-flight receive FrameBuffer,
            // or the scratch buffer). That buffer is still alive and, since the driver is
            // blocked waiting for completion, un-aliased - exactly the situation a real DMA
            // completion writes into.
            let buf = unsafe { core::slice::from_raw_parts_mut(c.result_ptr, c.result_len) };
            buf[..payload.len()].copy_from_slice(payload);
            buf[64..68].copy_from_slice(&footer.to_bytes());
            self.complete = true;
        }
    }

    impl Tc6Interface for SpyInterface {
        type Error = core::convert::Infallible;

        fn begin_send_block(
            &mut self,
            header: u32,
            data: impl ReadBuffer<Word = u8>,
            mut result: impl WriteBuffer<Word = u8>,
        ) -> Result<(), Self::Error> {
            let (dptr, dlen) = unsafe { data.read_buffer() };
            assert_eq!(dlen, 64, "a data chunk payload is one 64-byte block");
            let mut snap = [0u8; 64];
            snap.copy_from_slice(unsafe { core::slice::from_raw_parts(dptr, 64) });
            let (rptr, rlen) = unsafe { result.write_buffer() };
            self.chunks.push(Chunk {
                header,
                data: snap,
                joined: false,
                result_ptr: rptr,
                result_len: rlen,
            });
            Ok(())
        }

        fn transfer_complete(&mut self) -> Result<bool, Self::Error> {
            // edge triggered: reading the "done" flag clears it, like a real status bit.
            // This also means `on_interrupt`'s internal loop terminates.
            Ok(core::mem::take(&mut self.complete))
        }

        fn begin_send_joined_block(
            &mut self,
            data: impl ReadBuffer<Word = u8>,
            mut result: impl WriteBuffer<Word = u8>,
        ) -> Result<(), Self::Error> {
            let (dptr, dlen) = unsafe { data.read_buffer() };
            assert!(dlen >= 68);
            let hdr: [u8; 4] = unsafe { core::slice::from_raw_parts(dptr, 4) }
                .try_into()
                .unwrap();
            let mut snap = [0u8; 64];
            snap.copy_from_slice(unsafe { core::slice::from_raw_parts(dptr.add(4), 64) });
            let (rptr, rlen) = unsafe { result.write_buffer() };
            self.chunks.push(Chunk {
                header: u32::from_be_bytes(hdr),
                data: snap,
                joined: true,
                result_ptr: rptr,
                result_len: rlen,
            });
            Ok(())
        }
    }

    /// Minimal `ReadBuffer`/`WriteBuffer` over a raw slice, for calling interface methods
    /// directly without going through the driver's private `DMAPtr`.
    struct RawBuf {
        ptr: *mut u8,
        len: usize,
    }
    impl RawBuf {
        fn new(s: &mut [u8]) -> Self {
            Self {
                ptr: s.as_mut_ptr(),
                len: s.len(),
            }
        }
    }
    unsafe impl ReadBuffer for RawBuf {
        type Word = u8;
        unsafe fn read_buffer(&self) -> (*const u8, usize) {
            (self.ptr, self.len)
        }
    }
    unsafe impl WriteBuffer for RawBuf {
        type Word = u8;
        unsafe fn write_buffer(&mut self) -> (*mut u8, usize) {
            (self.ptr, self.len)
        }
    }

    /// `fake_tc6` behind the real SPI wrapper, but with a completion flag that reads `false`
    /// once before it reads `true` - i.e. a transfer looks "in flight" for exactly one poll.
    /// That is what lets `on_interrupt`'s loop take its `transfer_complete() == false` exit
    /// (the plain blocking `EmbeddedHalImplementation` never would). A real DMA interface
    /// behaves like this for real reasons.
    struct PollingFake {
        inner: crate::embeded_io_implementation::EmbeddedHalImplementation<
            fake_tc6::fake_tc6_device::FakeTc6SpiDevice,
        >,
        polls_until_done: u32,
    }
    impl PollingFake {
        fn new(spi: fake_tc6::fake_tc6_device::FakeTc6SpiDevice) -> Self {
            Self {
                inner: crate::embeded_io_implementation::EmbeddedHalImplementation::new(spi),
                polls_until_done: 0,
            }
        }
    }
    impl Tc6Interface for PollingFake {
        type Error = crate::embeded_io_implementation::EmbeddedHalImplementationError<
            fake_tc6::fake_tc6_device::FakeTc6SpiDeviceError,
        >;

        fn begin_send_block(
            &mut self,
            header: u32,
            data: impl ReadBuffer<Word = u8>,
            result: impl WriteBuffer<Word = u8>,
        ) -> Result<(), Self::Error> {
            self.inner.begin_send_block(header, data, result)?;
            self.polls_until_done = 2;
            Ok(())
        }

        fn transfer_complete(&mut self) -> Result<bool, Self::Error> {
            self.polls_until_done = self.polls_until_done.saturating_sub(1);
            Ok(self.polls_until_done == 0)
        }

        fn begin_send_joined_block(
            &mut self,
            data: impl ReadBuffer<Word = u8>,
            result: impl WriteBuffer<Word = u8>,
        ) -> Result<(), Self::Error> {
            self.inner.begin_send_joined_block(data, result)?;
            self.polls_until_done = 2;
            Ok(())
        }
    }

    // --------------------------------------------------------------------------------------
    // environment setup
    // --------------------------------------------------------------------------------------

    /// Builds a fresh, isolated driver pair. Every invocation declares its own `box_pool!`
    /// singletons (they are function-local types) and leaks its queues, so tests running in
    /// parallel never share pool state.
    macro_rules! setup {
        ($hi:ident, $lo:ident, $iface:expr, $signal:expr) => {
            heapless::box_pool!(RxPool: FrameBuffer<M>);
            heapless::box_pool!(TxPool: FrameBuffer<M>);
            for _ in 0..8usize {
                RxPool.manage(::std::boxed::Box::leak(::std::boxed::Box::new(BoxBlock::new())));
                TxPool.manage(::std::boxed::Box::leak(::std::boxed::Box::new(BoxBlock::new())));
            }
            let rx_q: &'static mut Queue<Box<RxPool>, NQ> =
                ::std::boxed::Box::leak(::std::boxed::Box::new(Queue::new()));
            let tx_q: &'static mut Queue<Box<TxPool>, NQ> =
                ::std::boxed::Box::leak(::std::boxed::Box::new(Queue::new()));
            #[allow(unused_mut)]
            let (mut $hi, mut $lo) = split(($iface), rx_q, tx_q, ($signal));
        };
    }

    fn ethernet_like(len: usize) -> Vec<u8> {
        (0..len)
            .map(|i| (i as u8).wrapping_mul(31).wrapping_add(7))
            .collect()
    }

    // --------------------------------------------------------------------------------------
    // low-priority side (smoltcp::phy::Device)
    // --------------------------------------------------------------------------------------

    #[test]
    fn low_prio_reports_expected_capabilities() {
        setup!(_hi, lo, SpyInterface::default(), NoopSignal);
        let caps = lo.capabilities();
        assert_eq!(caps.medium, Medium::Ethernet);
        assert_eq!(caps.max_transmission_unit, M);
        assert_eq!(caps.max_burst_size, Some(NQ));
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

    // --------------------------------------------------------------------------------------
    // high-priority side: transmit path
    // --------------------------------------------------------------------------------------

    #[test]
    fn transmit_token_builds_one_full_data_chunk() {
        setup!(hi, lo, SpyInterface::default(), NoopSignal);

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

    #[test]
    fn transmit_completion_without_receive_data_returns_to_idle() {
        setup!(hi, lo, SpyInterface::default(), NoopSignal);

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

    #[test]
    fn receive_only_chunk_targets_the_receive_buffer_tail() {
        setup!(hi, _lo, SpyInterface::default(), NoopSignal);

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
                let (dptr, dlen) = unsafe { data.read_buffer() };
                assert_eq!(dlen, 64);
                let mut snap = [0u8; 64];
                snap.copy_from_slice(unsafe { core::slice::from_raw_parts(dptr, 64) });
                let (_rptr, rlen) = unsafe { result.write_buffer() };
                self.last = Some((header, snap, rlen));
                Ok(())
            }
            fn transfer_complete(&mut self) -> Result<bool, Self::Error> {
                Ok(true)
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

        let mut iface = Recording::default();
        iface
            .begin_send_joined_block(RawBuf::new(&mut data[..]), RawBuf::new(&mut result[..]))
            .unwrap();

        let (h, payload, rlen) = iface.last.expect("forwarded to begin_send_block");
        assert_eq!(h, header.raw());
        assert_eq!(rlen, 68);
        assert_eq!(payload, core::array::from_fn::<u8, 64, _>(|i| i as u8));
    }

    // --------------------------------------------------------------------------------------
    // end-to-end through the real SPI wrapper + fake_tc6
    // --------------------------------------------------------------------------------------

    #[test]
    fn round_trip_transmit_through_fake_tc6() {
        use crate::embeded_io_implementation::EmbeddedHalImplementation;
        use fake_tc6::fake_tc6_device::FakeTc6SpiDevice;

        let iface = EmbeddedHalImplementation::new(FakeTc6SpiDevice::new());
        setup!(hi, lo, iface, NoopSignal);

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
    /// cursor past the frame length and drops `current_transmit_buffer`, so it is not
    /// re-sent. The follow-up `handle_idle` inside `on_interrupt` then issues a *receive*
    /// poll (joined chunk, DV = 0), not another copy of the frame.
    #[test]
    fn transmit_frame_is_sent_once_then_driver_polls_for_receive() {
        setup!(hi, lo, SpyInterface::default(), NoopSignal);

        let payload = ethernet_like(64);
        {
            let tok = lo.transmit(Instant::ZERO).unwrap();
            tok.consume(64, |buf| buf.copy_from_slice(&payload));
        }
        hi.on_transmit_ready().unwrap();
        assert_eq!(hi.interface.chunks.len(), 1);
        assert!(!hi.interface.chunks[0].joined);

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
        // the only extra chunk is the receive poll, and it does not carry the frame again
        assert_eq!(hi.interface.chunks.len(), 2);
        assert!(hi.interface.chunks[1].joined);
        assert!(!tc6_header::TransmitHeader::from_raw(hi.interface.chunks[1].header).dv());
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
    fn round_trip_multi_block_transmit_through_fake_tc6() {
        use crate::embeded_io_implementation::EmbeddedHalImplementation;
        use fake_tc6::fake_tc6_device::FakeTc6SpiDevice;

        let iface = EmbeddedHalImplementation::new(FakeTc6SpiDevice::new());
        setup!(hi, lo, iface, NoopSignal);

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
    fn black_box_round_trip_via_on_interrupt_and_device_receive() {
        use fake_tc6::fake_tc6_device::FakeTc6SpiDevice;

        let rx_frame: Vec<u8> = (0..128u32).map(|i| (i as u8) ^ 0x5A).collect();

        // seed the MAC-PHY *before* it becomes part of the driver - the only pre-split touch
        let mut spi = FakeTc6SpiDevice::new();
        spi.device.enqueue_receive_frame(rx_frame.clone());

        setup!(hi, lo, PollingFake::new(spi), NoopSignal);

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
}
