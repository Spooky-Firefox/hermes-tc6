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
                self.current_receive_buffer = None;
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
                // todo
                // update swap header with transmit_buff cursor -4 to allow for sending the header and data in one go, save the overwritten data to the scratch header.
            }
        } else if let Some(f) = self.current_receive_buffer.as_mut() {
            // irq fired, we have data to be read
            let mut header = tc6_header::TransmitHeader::new();
            // we don't have any data to send, so dv = 0
            header = header.with_dv(false);
            self.scratch_buffer[0..4].copy_from_slice(&header.raw().to_be_bytes());
            let data = DMAPtr::from_slice(&mut self.scratch_buffer);
            // SAFETY, we are writing to a point which is outside what is considered initialized, and we are not marking this part as initialized,
            // that is done when we read the footer returned from the tc6 chip, and then we can determine what is valid and update the length.
            let result = unsafe { DMAPtr::from_slice(f.get_mut_at_end(68)) };
            self.interface.begin_send_joined_block(data, result)?;
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

pub struct TC6DriverTxToken<'a, TxP: BoxPool + 'static> {
    frame: Box<TxP>,
    queue: &'a mut Producer<'static, Box<TxP>>,
}

impl<'a, TxP, const M: usize> smoltcp::phy::TxToken for TC6DriverTxToken<'a, TxP>
where
    TxP: BoxPool<Data = FrameBuffer<M>> + 'static,
{
    fn consume<R, F>(self, len: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let TC6DriverTxToken { mut frame, queue } = self;
        // safety: we say that this is initialized, as we assume that FnOnce will write to the buffer
        let r = unsafe {
            frame.add_len(len);
            f(frame.as_mut_slice())
        };

        queue.enqueue(frame).unwrap();
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
        = TC6DriverTxToken<'a, TxP>
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
        })
    }
}
