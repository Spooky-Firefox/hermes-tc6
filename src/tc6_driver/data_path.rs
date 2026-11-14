//! Data path: data chunks in both directions, driven by `on_interrupt` / `on_transmit_ready`
//! (TC6 spec Sections 7.3 and 7.7).

use super::*;

impl<T, RxP, TxP, const M: usize, const NQ: usize> TC6HighPrio<T, RxP, TxP, M, NQ>
where
    T: Tc6Interface,
    RxP: BoxPool<Data = FrameBuffer<M>> + 'static,
    TxP: BoxPool<Data = FrameBuffer<M>> + 'static,
{
    /// Service the chip. Call this from every interrupt source the chip/interface can raise: the
    /// TC6 chip's own IRQ pin *and* whatever signals SPI/DMA transfer completion (often a separate
    /// vector). It is re-entrant/idempotent - it just re-checks `driver_state` /
    /// [`Tc6Interface::transfer_complete`] rather than assuming which source triggered it, so
    /// spurious or overlapping calls are fine.
    ///
    /// # Draining and interface requirements
    ///
    /// One `on_interrupt` call services the chip until there is nothing left to do: after each
    /// chunk completes the footer's `RCA` (receive chunks available) and `TXC` (transmit credits)
    /// are read, and the loop keeps issuing chunks while the MAC-PHY still has receive data
    /// buffered or the host has a transmit chunk it has credit to send (TC6 spec Section 7.7).
    /// When both are exhausted it returns and waits for the next interrupt.
    ///
    /// The loop also yields whenever [`Tc6Interface::transfer_complete`] reports a transfer still
    /// in flight (`Ok(false)`), which is how a DMA/async interface hands control back between
    /// chunks; the next completion interrupt re-enters here. A **fully blocking** interface (e.g.
    /// the ready-made `EmbeddedHalImplementation`, whose `transfer_complete` is hard-coded `true`)
    /// never yields that way, so a genuinely long receive burst is drained inside a single call -
    /// capped at [`Self::MAX_INTERRUPT_STEPS`] chunks, after which it returns and resumes on the
    /// next interrupt.
    pub fn on_interrupt(&mut self) -> Result<(), T::Error> {
        // we might have a frame to receive
        let mut steps = 0u32;
        loop {
            if steps >= Self::MAX_INTERRUPT_STEPS {
                warn!(
                    "tc6: on_interrupt hit its {}-step cap (blocking interface draining a long \
                     burst, or a burst larger than the cap) - resuming on the next interrupt",
                    Self::MAX_INTERRUPT_STEPS
                );
                break;
            }
            steps += 1;
            match self.driver_state {
                TC6DriverState::Idle => {
                    self.handle_idle()?;
                    if matches!(self.driver_state, TC6DriverState::Idle) {
                        // `handle_idle` started nothing (not configured yet, or no receive
                        // buffer / transmit frame available) - stop spinning.
                        break;
                    }
                }
                TC6DriverState::WaitingForTransmitComplete => {
                    if self.interface.transfer_complete()? {
                        self.handle_transmit_complete()?;
                        // `handle_transmit_complete` refreshed `rx_chunks_available` /
                        // `tx_credits` from the footer. Stop here unless the chip still has
                        // receive data or a credited transmit is waiting - otherwise a blocking
                        // interface would re-arm a pointless receive poll every step until the
                        // cap (spec Section 7.7: stop reading at RCA = 0).
                        if !self.data_path_has_work() {
                            break;
                        }
                    } else {
                        break;
                    }
                }
                TC6DriverState::WaitingForControlComplete => {
                    if self.interface.transfer_complete()? {
                        self.handle_control_complete()?;
                        if !self.data_path_has_work() {
                            break;
                        }
                    } else {
                        break;
                    }
                }
            }
        }
        Ok(())
    }

    /// Upper bound on chunks [`on_interrupt`](Self::on_interrupt) will service in one call before
    /// returning (it resumes on the next interrupt). With a DMA/async interface the loop yields
    /// well before this via [`Tc6Interface::transfer_complete`]; the cap only bites when a fully
    /// blocking interface is draining a long receive burst in one go. The 5-bit footer `RCA`
    /// field tops out at 31, so 64 covers a maximal burst plus interleaved transmit chunks.
    pub const MAX_INTERRUPT_STEPS: u32 = 64;

    /// Whether [`on_interrupt`](Self::on_interrupt) should issue another data chunk rather than
    /// stop and wait for the next interrupt (TC6 spec Section 7.7): a queued control transaction
    /// always counts; otherwise keep going while the MAC-PHY still has receive chunks buffered
    /// (`RCA` > 0), a multi-block receive frame is half-reassembled, or a transmit chunk is
    /// waiting and the chip has a credit for it (`TXC` > 0).
    fn data_path_has_work(&self) -> bool {
        if self.pending_control.is_some() {
            return true;
        }
        if !self.sync_established {
            return false;
        }
        if self.rx_chunks_available > 0 {
            return true;
        }
        if self
            .current_receive_buffer
            .as_ref()
            .is_some_and(|b| !b.as_slice().is_empty())
        {
            // partway through reassembling a multi-block receive frame
            return true;
        }
        if self.tx_credits > 0
            && (self.current_transmit_buffer.is_some() || !self.tx_ready_consumer.is_empty())
        {
            return true;
        }
        false
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

    pub(super) fn handle_transmit_complete(&mut self) -> Result<(), T::Error> {
        // Copy the 4 footer bytes out of this chunk's MISO region: the 68-byte window past the
        // in-flight receive frame's data, or the scratch buffer for a transmit-only /
        // rx-pool-exhausted chunk.
        let footer_bytes: [u8; 4] = {
            let region: &mut [u8] = if let Some(receive_frame) =
                self.current_receive_buffer.as_mut()
            {
                // Safety: a block send has completed, so the chip has written 64 payload + 4
                // footer bytes into the 68 bytes past the frame's initialized data.
                unsafe { receive_frame.get_mut_at_end(68) }
            } else if self.current_transmit_buffer.is_some() {
                &mut self.scratch_buffer
            } else {
                unreachable!(
                    "transfer completed with neither a receive frame to fill nor a transmit frame in flight"
                )
            };
            region[64..68].try_into().unwrap()
        };
        let footer = tc6_footer::ReceiveFooter::from_bytes(footer_bytes);

        // TC6 spec 7.5.1: on a transmit-header parity error the MAC-PHY streams the constant
        // 0xC0000001 (HDRB = 1, EXST = 1, valid odd parity) from the second 32-bit word onward
        // in place of the footer, and ignores the chunk. Bit errors on MISO instead surface as
        // a failed footer parity check. Either way nothing in the footer is trustworthy and the
        // chunk was not accepted: leave the transmit cursor where it is so the same block is
        // re-issued by the next `handle_idle`, and don't touch RCA / TXC / SYNC / EXST / DV.
        // Restore any real frame bytes borrowed to carry a joined continuation header first.
        if !footer.parity_ok() || footer.hdrb() {
            warn!(
                "tc6: data chunk footer rejected (parity_ok={}, hdrb={}) - re-issuing chunk",
                footer.parity_ok(),
                footer.hdrb()
            );
            // cursor is only non-zero while a multi-block transmit is in flight
            let cursor = self.current_transmit_buffer_cursor;
            if cursor != 0
                && let Some(frame) = self.current_transmit_buffer.as_mut()
            {
                frame.as_mut_slice()[cursor - 4..cursor].copy_from_slice(&self.scratch_header);
            }
            self.driver_state = TC6DriverState::Idle;
            return Ok(());
        }

        // Footer trustworthy from here on: the transmit chunk was accepted, so advance past it
        // (restoring the 4 bytes a joined continuation header borrowed) and free the frame once
        // its last block has gone out.
        if self.current_transmit_buffer.is_some() {
            let cursor = self.current_transmit_buffer_cursor;
            // A continuation block (cursor != 0) was sent as a joined header+data chunk, which
            // meant overwriting the 4 real frame bytes at [cursor-4, cursor) with the chunk
            // header. Those bytes were already transmitted in the previous block; restore them
            // from `scratch_header` so the frame stays intact (needed for any retransmit).
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

        // Every footer carries the chip's live RCA / TXC counters; refresh our copies so the
        // `on_interrupt` drain loop knows whether to keep going (spec Section 7.7).
        self.rx_chunks_available = footer.rca();
        self.tx_credits = footer.txc();
        // Every receive footer mirrors CONFIG0.SYNC. Losing it after bring-up means the chip
        // reset itself; flag that the host needs to re-run `configure` (spec Section 7.7).
        if self.sync_established && !footer.sync() {
            warn!("tc6: receive footer reports SYNC=0 - MAC-PHY reset itself, re-run configure()");
            self.sync_established = false;
        }
        // Extended status (spec Section 7.3.7 / 7.7): a STATUS0 / STATUS1 bit is set and
        // unmasked. Latch it so the application can run `service_extended_status()` (a control
        // read of STATUS0 + write-1-to-clear) from a context where blocking control I/O is OK.
        if footer.exst() {
            if !self.extended_status_pending {
                warn!(
                    "tc6: data footer EXST=1 - STATUS0 needs servicing (call service_extended_status)"
                );
            }
            self.extended_status_pending = true;
        }
        if footer.dv() {
            self.handle_data_valid(footer)?;
        }
        self.driver_state = TC6DriverState::Idle;
        Ok(())
    }

    fn handle_data_valid(&mut self, footer: ReceiveFooter) -> Result<(), T::Error> {
        // If we have no receive buffer we sent NORX = 1 (rx pool exhausted) and the chip should
        // have returned DV = 0. A DV = 1 here means a footer bit error slipped past the parity
        // check or the chip is desynced - drop the chunk rather than unwrap a `None`.
        if self.current_receive_buffer.is_none() {
            warn!("tc6: footer DV=1 with no receive buffer (NORX chunk / desync) - dropping chunk");
            return Ok(());
        }

        let sv = footer.sv();
        let ev = footer.ev();
        // SWO is 4 bits (start <= 60) and EBO is 6 bits (end <= 64), so these can't themselves
        // run off a 64-byte payload; the checks below are about ordering and buffer capacity.
        let start = footer.swo() as usize * 4;
        let end = footer.ebo() as usize + 1;
        let (fresh, spare) = {
            let f = self.current_receive_buffer.as_ref().unwrap();
            (f.as_slice().is_empty(), f.spare_capacity())
        };

        // Reject footer field combinations the framing rules forbid (spec 7.3.7 / 7.3.8.1) or
        // that would not fit the receive buffer, instead of feeding them to an unchecked
        // `add_len` / `get_mut_at_end` that would panic. In every case the in-progress frame is
        // unrecoverable, so drop it and wait for the next SV.
        let bail = if fresh && !sv {
            Some("DV=1 SV=0 on a fresh receive frame (data valid without start)")
        } else if fresh && ev && end <= start {
            Some("end-of-frame offset at or before start-of-frame offset")
        } else if !fresh && sv && !ev {
            Some("SV=1 EV=0 mid-frame (repeated frame start)")
        } else if !fresh && sv && ev && footer.ebo() >= footer.swo() * 4 {
            Some("frame concatenation with EBO >= 4*SWO (overlapping frames)")
        } else if !fresh && sv && spare < 64 {
            Some("this frame plus the next frame's start would overflow the receive buffer")
        } else if !fresh && ((ev && end > spare) || (!ev && spare < 64)) {
            Some("received frame larger than the receive buffer")
        } else {
            None
        };
        if let Some(why) = bail {
            warn!(
                "tc6: receive framing rejected ({}) - dropping in-progress frame",
                why
            );
            self.current_receive_buffer = None;
            self.extra_start_buffer = None;
            return Ok(());
        }

        let frame_done: bool;
        if fresh {
            // guarded above: SV is set, and when EV is set `start < end <= 64`.
            let receive_frame = self.current_receive_buffer.as_mut().unwrap();
            unsafe { receive_frame.set_start(start) };
            if ev {
                unsafe { receive_frame.add_len(end - start) };
                frame_done = true;
            } else {
                unsafe { receive_frame.add_len(64 - start) };
                frame_done = false;
            }
        } else {
            if sv {
                // SV && EV here (guarded): this chunk carries the end of the current frame and
                // the start of the next one. Stash the next frame's bytes so the following
                // receive buffer starts with them. `spare >= 64` was checked above.
                let receive_frame = self.current_receive_buffer.as_mut().unwrap();
                let next_start = heapless::Vec::<u8, 64>::from_slice(
                    &unsafe { receive_frame.get_mut_at_end(64) }[start..],
                )
                .unwrap();
                self.extra_start_buffer = Some(next_start);
            }
            let receive_frame = self.current_receive_buffer.as_mut().unwrap();
            if ev {
                unsafe { receive_frame.add_len(end) };
                frame_done = true;
            } else {
                // DV set, EV/SV clear: the whole 64-byte block is frame data.
                unsafe { receive_frame.add_len(64) };
                frame_done = false;
            }
        }

        if frame_done {
            let frame = self.current_receive_buffer.take().unwrap();
            // Frame Drop (spec Section 7.3.7): the MAC-PHY sets FD at end-of-frame when the frame
            // must be discarded (e.g. it overflowed the receive buffer). FD is only meaningful
            // with EV = 1, which is exactly the `frame_done` case. Free the buffer instead of
            // handing a truncated/corrupt frame up to smoltcp.
            if footer.fd() {
                warn!(
                    "tc6: receive footer FD=1 - dropping this frame ({} bytes)",
                    frame.as_slice().len()
                );
                // `frame` drops here, returning its block to the pool
            } else {
                trace!(
                    "tc6: received frame complete ({} bytes), handing to low-prio",
                    frame.as_slice().len()
                );
                self.rx_ready_producer.enqueue(frame).unwrap();
            }
        }
        Ok(())
    }

    pub(super) fn handle_idle(&mut self) -> Result<(), T::Error> {
        // A queued register read/write takes priority over data chunks: command mode is how the
        // host configures the chip in the first place (set CONFIG0.SYNC etc.), and while a
        // control transaction is pending we must not start a data transfer.
        if self.pending_control.is_some() {
            return self.start_control_transaction();
        }

        // Until `configure` has driven CONFIG0.SYNC = 1, the MAC-PHY ignores transmit data
        // chunks and returns DV = 0, so running the data path would just churn the pool and the
        // SPI bus for nothing. Stay idle; frames the low-prio side enqueues wait in the bounded
        // queue (and `transmit()` starts returning None once it fills).
        if !self.sync_established {
            return Ok(());
        }

        // do we have a receive buffer ready to fill? if not allocate one from the pool and queue it up for the tc6 chip to fill
        if self.current_receive_buffer.is_none() {
            // TODO make sure the compiler is smart and avoids a hugely expensive copy/move of the FrameBuffer struct here, since it is a large array. It should be optimized away by the compiler, but we should check.
            self.current_receive_buffer = RxP::alloc(FrameBuffer::new()).ok();
            match self.current_receive_buffer.as_mut() {
                Some(buf) => {
                    // A stashed frame start from the previous chunk's tail (see
                    // `handle_data_valid`) goes at the front of the fresh buffer.
                    if let Some(start) = self.extra_start_buffer.take() {
                        buf.push_slice(start.as_slice());
                    }
                }
                None => {
                    // Pool exhausted: this chunk goes out with NORX = 1 and inbound data is
                    // dropped. Keep `extra_start_buffer` (if any) so the stashed frame start is
                    // still prepended once a buffer frees up, rather than losing it here.
                    warn!(
                        "tc6: rx pool exhausted - sending NORX, dropping inbound data this chunk"
                    );
                }
            }
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
                // SAFETY: the 64-byte window may run past the frame's initialized bytes (a frame
                // shorter than one block, e.g. a 42-byte ARP request) into spare capacity, which
                // is why this is `get_mut_at_index` (bounded by M) rather than
                // `get_raw_mut_from_index` (bounded by the frame length, which panicked). The
                // chip ignores bytes past `ebo`.
                let data = unsafe {
                    DMAPtr::from_slice(
                        frame.get_mut_at_index(self.current_transmit_buffer_cursor, 64),
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
                trace!(
                    "tc6: tx first chunk (frame len={}, ev={}, ebo={}, norx={})",
                    len,
                    ev,
                    ebo,
                    self.current_receive_buffer.is_none()
                );
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
                trace!(
                    "tc6: tx continuation chunk at cursor {} (ev={})",
                    cursor, ev
                );
                self.interface.begin_send_joined_block(data, result)?;
                self.driver_state = TC6DriverState::WaitingForTransmitComplete;
            }
        } else if let Some(f) = self.current_receive_buffer.as_mut() {
            // irq fired, we have data to be read
            trace!("tc6: rx-only chunk (nothing queued to transmit)");
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
            // Nothing queued to transmit and the receive pool is exhausted (no buffer to point
            // the chip at). Don't churn the bus with NORX polls: the MAC-PHY keeps any pending
            // receive data and holds IRQn asserted, so we retry on the next interrupt or once the
            // low-priority side frees a buffer. Staying `Idle` lets `on_interrupt` break out.
            trace!("tc6: handle_idle - nothing to transmit and rx pool empty, deferring");
        }
        Ok(())
    }
}
