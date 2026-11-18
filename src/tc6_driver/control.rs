//! Control path: register reads and writes (TC6 spec Section 7.4), both the non-blocking
//! `begin_register_*` / `take_control_result` pair and the blocking `read_register` family.

use super::*;

/// Error from the register read/write entry points on [`TC6HighPrio`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControlError<E> {
    /// A control transaction is already pending or in flight; retry once it has completed
    /// (poll [`TC6HighPrio::take_control_result`]).
    Busy,
    /// The requested register count is zero or exceeds [`CONTROL_MAX_REGS`].
    UnsupportedCount,
    /// The underlying interface failed to start the transfer.
    Interface(E),
}

/// Error from the blocking register functions on [`TC6HighPrio`]
/// ([`read_register`](TC6HighPrio::read_register) and friends).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RegisterError<E> {
    /// The driver is mid-transfer: a control transaction is queued or in flight, or a data
    /// chunk is still in flight (a DMA interface mid-transfer, or
    /// [`on_interrupt`](TC6HighPrio::on_interrupt) stopped at its step cap). Call `on_interrupt`
    /// and retry.
    Busy,
    /// The requested register count is zero or exceeds [`CONTROL_MAX_REGS`].
    UnsupportedCount,
    /// The transfer never reported complete within [`Config::transfer_poll_limit`]. The state
    /// machine is returned to idle, but a genuinely stuck transfer may still be running on the
    /// interface.
    Timeout,
    /// The MAC-PHY rejected the control command header (HDRB set, or the echo failed its parity
    /// check). No register was accessed.
    HeaderRejected,
    /// A write's echoed data (TC6 spec 7.4.4) differs from what was sent, at register offset
    /// `index`: a bit error on MOSI or MISO. If it was MOSI, the chip wrote the corrupted value,
    /// so write it again or read it back.
    EchoMismatch { index: u8 },
    /// The underlying interface failed to run the transfer.
    Interface(E),
}

/// Default for [`Config::transfer_poll_limit`], also used by the blocking register functions
/// before [`TC6HighPrio::configure`] has run.
pub(super) const DEFAULT_TRANSFER_POLL_LIMIT: u32 = 100_000;

/// A register read/write request accepted but not yet put on the wire.
pub(super) struct PendingControl {
    /// `true` = write (WNR = 1), `false` = read.
    pub(super) write: bool,
    /// Memory Map Selector (TC6 spec Table 6).
    pub(super) mms: u8,
    /// Address of the first register within the selected memory map.
    pub(super) addr: u16,
    /// Number of consecutive registers, `1..=CONTROL_MAX_REGS`.
    pub(super) count: u8,
    /// Write data, `values[..count]` used for writes; ignored for reads.
    pub(super) values: [u32; CONTROL_MAX_REGS],
}

/// Bookkeeping for the control transaction currently on the wire.
pub(super) struct ControlInFlight {
    write: bool,
    count: u8,
}

/// Result of a completed control transaction, taken via [`TC6HighPrio::take_control_result`].
pub struct ControlResult {
    /// The control command header echoed back by the MAC-PHY (MISO bytes `[4..8)`).
    pub echoed_header: tc6_header::ControlHeader,
    /// `true` if the MAC-PHY flagged the command header as bad (HDRB set, or the echoed header
    /// failed its parity check - e.g. the `0xC0000001` error code from spec Section 7.5.1). No
    /// register access was performed; the host should retry the transaction.
    pub header_bad: bool,
    /// `true` if this was a write.
    pub write: bool,
    values: [u32; CONTROL_MAX_REGS],
    count: u8,
}

impl ControlResult {
    /// The register values.
    ///
    /// For a read these are the values read back. For a write these are the *echoed* write data
    /// words the MAC-PHY returned, which the host can compare against what it sent to spot bus
    /// errors (TC6 spec Section 7.4.4). Empty when [`header_bad`](Self::header_bad) is set.
    pub fn values(&self) -> &[u32] {
        if self.header_bad {
            &[]
        } else {
            &self.values[..self.count as usize]
        }
    }
}

impl<'s, T, const RX_N: usize, const TX_N: usize> TC6HighPrio<'s, T, RX_N, TX_N>
where
    T: Tc6Interface,
{
    // --- command mode: register read / write (TC6 spec Section 7.4) ---

    /// `true` if a control transaction is pending or in flight, i.e. the register read/write
    /// entry points would return [`ControlError::Busy`].
    pub fn control_busy(&self) -> bool {
        self.pending_control.is_some()
            || self.control_in_flight.is_some()
            || matches!(self.driver_state, TC6DriverState::WaitingForControlComplete)
    }

    /// Take the result of the last completed control transaction, if one is waiting.
    pub fn take_control_result(&mut self) -> Option<ControlResult> {
        self.last_control_result.take()
    }

    /// Queue a read of `count` consecutive registers starting at `addr` within memory map `mms`
    /// (see `crate::registers` for the standard map and its addresses). The address
    /// post-increments per register (TC6 `AID` = 0). Pump [`on_interrupt`](Self::on_interrupt)
    /// until [`take_control_result`](Self::take_control_result) returns the values.
    pub fn begin_register_read(
        &mut self,
        mms: u8,
        addr: u16,
        count: u8,
    ) -> Result<(), ControlError<T::Error>> {
        self.queue_control(PendingControl {
            write: false,
            mms,
            addr,
            count,
            values: [0; CONTROL_MAX_REGS],
        })
    }

    /// Queue a write of `values` to consecutive registers starting at `addr` within memory map
    /// `mms`. The address post-increments per register (TC6 `AID` = 0). At most
    /// [`CONTROL_MAX_REGS`] values.
    pub fn begin_register_write(
        &mut self,
        mms: u8,
        addr: u16,
        values: &[u32],
    ) -> Result<(), ControlError<T::Error>> {
        if values.is_empty() || values.len() > CONTROL_MAX_REGS {
            return Err(ControlError::UnsupportedCount);
        }
        let mut buf = [0u32; CONTROL_MAX_REGS];
        buf[..values.len()].copy_from_slice(values);
        self.queue_control(PendingControl {
            write: true,
            mms,
            addr,
            count: values.len() as u8,
            values: buf,
        })
    }

    // --- blocking register access ---
    //
    // The `begin_register_*` / `take_control_result` pair above suits interrupt-driven or async
    // code. These run one control transaction to completion instead: they spin on
    // `Tc6Interface::transfer_complete` (up to `Config::transfer_poll_limit` polls) and never
    // enter the data path, so no frame is sent or received as a side effect. They work before
    // `configure` too.

    /// Read one register, blocking until the control transaction completes. See
    /// [`read_registers`](Self::read_registers).
    pub fn read_register(&mut self, mms: u8, addr: u16) -> Result<u32, RegisterError<T::Error>> {
        let mut value = [0];
        self.read_registers(mms, addr, &mut value)?;
        Ok(value[0])
    }

    /// Read `out.len()` consecutive registers (`1..=`[`CONTROL_MAX_REGS`]) starting at `addr`
    /// within memory map `mms`, in one control transaction, blocking until it completes.
    ///
    /// Returns [`RegisterError::Busy`] instead of waiting if a control transaction or data chunk
    /// is already in flight.
    pub fn read_registers(
        &mut self,
        mms: u8,
        addr: u16,
        out: &mut [u32],
    ) -> Result<(), RegisterError<T::Error>> {
        let count = Self::register_count(out.len())?;
        let res = self.run_control_blocking(
            PendingControl {
                write: false,
                mms,
                addr,
                count,
                values: [0; CONTROL_MAX_REGS],
            },
            self.transfer_poll_limit,
        )?;
        if res.header_bad {
            return Err(RegisterError::HeaderRejected);
        }
        out.copy_from_slice(res.values());
        Ok(())
    }

    /// Write one register, blocking until the control transaction completes. See
    /// [`write_registers`](Self::write_registers).
    pub fn write_register(
        &mut self,
        mms: u8,
        addr: u16,
        value: u32,
    ) -> Result<(), RegisterError<T::Error>> {
        self.write_registers(mms, addr, &[value])
    }

    /// Write `values` (`1..=`[`CONTROL_MAX_REGS`]) to consecutive registers starting at `addr`
    /// within memory map `mms`, in one control transaction, blocking until it completes.
    ///
    /// The MAC-PHY echoes the write data it received (TC6 spec 7.4.4); any difference from
    /// `values` is reported as [`RegisterError::EchoMismatch`]. Returns
    /// [`RegisterError::Busy`] instead of waiting if a control transaction or data chunk is
    /// already in flight.
    pub fn write_registers(
        &mut self,
        mms: u8,
        addr: u16,
        values: &[u32],
    ) -> Result<(), RegisterError<T::Error>> {
        let count = Self::register_count(values.len())?;
        let mut buf = [0u32; CONTROL_MAX_REGS];
        buf[..values.len()].copy_from_slice(values);
        let res = self.run_control_blocking(
            PendingControl {
                write: true,
                mms,
                addr,
                count,
                values: buf,
            },
            self.transfer_poll_limit,
        )?;
        if res.header_bad {
            return Err(RegisterError::HeaderRejected);
        }
        if let Some(index) = res.values().iter().zip(values).position(|(e, v)| e != v) {
            warn!(
                "tc6: write to mms={} addr={:#06x}: echoed data differs at offset {}",
                mms, addr, index
            );
            return Err(RegisterError::EchoMismatch { index: index as u8 });
        }
        Ok(())
    }

    fn register_count(len: usize) -> Result<u8, RegisterError<T::Error>> {
        if len == 0 || len > CONTROL_MAX_REGS {
            warn!("tc6: register request with unsupported count {}", len);
            return Err(RegisterError::UnsupportedCount);
        }
        Ok(len as u8)
    }

    fn queue_control(&mut self, req: PendingControl) -> Result<(), ControlError<T::Error>> {
        if self.control_busy() {
            trace!("tc6: register request rejected - control path busy");
            return Err(ControlError::Busy);
        }
        if req.count == 0 || req.count as usize > CONTROL_MAX_REGS {
            warn!("tc6: register request with unsupported count {}", req.count);
            return Err(ControlError::UnsupportedCount);
        }
        debug!(
            "tc6: queued register {} mms={} addr={:#06x} count={}",
            if req.write { "write" } else { "read" },
            req.mms,
            req.addr,
            req.count
        );
        self.pending_control = Some(req);
        // If the state machine is otherwise idle, kick it now so callers that don't wire up an
        // interrupt source (e.g. a blocking init loop) still make progress. Otherwise it is
        // picked up by `handle_idle` once the in-flight transfer finishes.
        if matches!(self.driver_state, TC6DriverState::Idle) {
            self.handle_idle().map_err(ControlError::Interface)?;
        }
        Ok(())
    }

    /// Lay the pending request out in `control_tx_buffer` and hand it to the interface.
    pub(super) fn start_control_transaction(&mut self) -> Result<(), T::Error> {
        let req = self
            .pending_control
            .take()
            .expect("start_control_transaction called with no pending request");
        let n = req.count as usize;
        // header (4B) + n data words + trailing don't-care word (4B), see spec Figures 13/14.
        let total = 4 * (n + 2);

        let header = tc6_header::ControlHeader::new()
            .with_wnr(req.write)
            .with_mms(req.mms)
            .with_addr(req.addr)
            .with_len(req.count - 1) // LEN is register count minus one
            .with_correct_parity();

        self.control_tx_buffer[..total].fill(0);
        self.control_tx_buffer[0..4].copy_from_slice(&header.to_bytes());
        if req.write {
            for (i, word) in req.values[..n].iter().enumerate() {
                self.control_tx_buffer[4 + 4 * i..8 + 4 * i].copy_from_slice(&word.to_be_bytes());
            }
        }
        self.control_rx_buffer[..total].fill(0);

        let tx = DMAPtr::from_slice(&mut self.control_tx_buffer[..total]);
        let rx = DMAPtr::from_slice(&mut self.control_rx_buffer[..total]);
        self.interface.begin_control_transaction(tx, rx)?;

        self.control_in_flight = Some(ControlInFlight {
            write: req.write,
            count: req.count,
        });
        self.driver_state = TC6DriverState::WaitingForControlComplete;
        Ok(())
    }

    /// Parse the MISO stream of a finished control transaction into a [`ControlResult`].
    pub(super) fn handle_control_complete(&mut self) -> Result<(), T::Error> {
        let in_flight = self
            .control_in_flight
            .take()
            .expect("handle_control_complete called with no transaction in flight");
        let n = in_flight.count as usize;

        // MISO layout: [0..4) don't-care, [4..8) echoed control header, [8..) echoed/read words.
        let echoed_header =
            tc6_header::ControlHeader::from_bytes(self.control_rx_buffer[4..8].try_into().unwrap());
        // On a header parity error the MAC-PHY streams the 0xC0000001 error code where the
        // echoed header would be (spec Section 7.5.1); it decodes to HDRB = 1, so this catches
        // both that and an ordinary bad-header echo.
        let header_bad = echoed_header.hdrb() || !echoed_header.parity_ok();
        if header_bad {
            warn!("tc6: control transaction completed with a bad command header (HDRB/parity)");
        } else {
            debug!(
                "tc6: control transaction complete ({}, {} reg(s))",
                if in_flight.write { "write" } else { "read" },
                n
            );
        }

        let mut values = [0u32; CONTROL_MAX_REGS];
        if !header_bad {
            for (i, slot) in values[..n].iter_mut().enumerate() {
                let off = 8 + 4 * i;
                *slot =
                    u32::from_be_bytes(self.control_rx_buffer[off..off + 4].try_into().unwrap());
            }
        }

        self.last_control_result = Some(ControlResult {
            echoed_header,
            header_bad,
            write: in_flight.write,
            values,
            count: in_flight.count,
        });
        self.driver_state = TC6DriverState::Idle;
        Ok(())
    }

    /// Issue one control transaction and spin on [`Tc6Interface::transfer_complete`] until it
    /// finishes or `poll_limit` iterations elapse. Never enters the data path.
    ///
    /// Refuses with [`RegisterError::Busy`] unless the driver is fully idle: starting a control
    /// transaction while a data chunk is in flight would overwrite that chunk's state and lose
    /// its footer.
    pub(super) fn run_control_blocking(
        &mut self,
        req: PendingControl,
        poll_limit: u32,
    ) -> Result<ControlResult, RegisterError<T::Error>> {
        if self.control_busy() || !matches!(self.driver_state, TC6DriverState::Idle) {
            warn!("tc6: blocking control transaction refused - driver busy");
            return Err(RegisterError::Busy);
        }
        self.pending_control = Some(req);
        self.start_control_transaction()
            .map_err(RegisterError::Interface)?;

        let mut spins = 0u32;
        loop {
            if self
                .interface
                .transfer_complete()
                .map_err(RegisterError::Interface)?
            {
                self.handle_control_complete()
                    .map_err(RegisterError::Interface)?;
                return Ok(self
                    .last_control_result
                    .take()
                    .expect("handle_control_complete stores a result"));
            }
            spins += 1;
            if spins >= poll_limit {
                warn!(
                    "tc6: control transfer never completed within {} polls - giving up",
                    poll_limit
                );
                // don't leave the state machine wedged in WaitingForControlComplete
                self.control_in_flight = None;
                self.driver_state = TC6DriverState::Idle;
                return Err(RegisterError::Timeout);
            }
        }
    }
}
