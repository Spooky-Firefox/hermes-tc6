//! Bring-up: `Config`, the blocking `configure` sequence and extended-status servicing
//! (TC6 spec Sections 7.6 and 9.2).

use super::*;

/// `CONFIG0.TXCTHRESH` - minimum free transmit credits before the chip asserts IRQn.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum TxCreditThreshold {
    /// >= 1 credit (reset default).
    #[default]
    One,
    /// >= 4 credits.
    Four,
    /// >= 8 credits.
    Eight,
    /// >= 16 credits.
    Sixteen,
}

impl TxCreditThreshold {
    const fn bits(self) -> u8 {
        match self {
            TxCreditThreshold::One => 0b00,
            TxCreditThreshold::Four => 0b01,
            TxCreditThreshold::Eight => 0b10,
            TxCreditThreshold::Sixteen => 0b11,
        }
    }
}

/// Frame timestamp width, selected by `CONFIG0.FTSS` when timestamps are enabled.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TimestampSize {
    Bits32,
    Bits64,
}

/// PLCA (Physical Layer Collision Avoidance) node setup, written to the PHY's MMS 4 registers
/// during [`TC6HighPrio::configure`].
///
/// The MAC-PHY runs the PLCA state machine, but every node must be told its own ID - there is no
/// auto-assignment. Build one with [`PlcaConfig::follower`] or [`PlcaConfig::coordinator`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PlcaConfig {
    /// This node's PLCA ID (`0..=254`). Node `0` is the coordinator and sends the BEACON.
    pub node_id: u8,
    /// Total node count. Only used by the coordinator (`node_id == 0`); ignored otherwise.
    pub node_count: u8,
}

impl PlcaConfig {
    /// A follower node with the given ID. Node count is only needed on the coordinator.
    pub const fn follower(node_id: u8) -> Self {
        Self {
            node_id,
            node_count: 0,
        }
    }

    /// The coordinator (node 0), which sends the BEACON, for a network of `node_count` nodes.
    pub const fn coordinator(node_count: u8) -> Self {
        Self {
            node_id: 0,
            node_count,
        }
    }
}

/// Bring-up configuration applied by [`TC6HighPrio::configure`] / [`split_configured`].
///
/// [`Config::default`] is the minimal safe setup: unprotected control, >= 1 transmit credit
/// before IRQn, no cut-through, no timestamps, PLCA left untouched, every maskable interrupt
/// masked. The chunk payload size is always forced to 64 bytes - the data path assumes it.
//
// TODO: `CONFIG0.PROTE` (protected control transfers - each 32-bit word sent with its ones'
// complement, spec 7.4.4) is not offered here yet: the control path in `start_control_transaction`
// / `handle_control_complete` only does unprotected single-word transfers.
#[derive(Debug, Clone, Copy)]
pub struct Config {
    /// `CONFIG0.TXCTHRESH`.
    pub tx_credit_threshold: TxCreditThreshold,
    /// `CONFIG0.TXCTE` - cut-through from the SPI host to the network (requires `STDCAP.CTC`).
    ///
    /// **Not implemented yet:** the data path has no cut-through handling (transmit-buffer
    /// underflow recovery, partial-frame termination), so [`configure`](TC6HighPrio::configure)
    /// rejects `true` with [`ConfigError::NotImplemented`].
    pub cut_through_tx: bool,
    /// `CONFIG0.RXCTE` - cut-through from the network to the SPI host (requires `STDCAP.CTC`).
    ///
    /// **Not implemented yet** - see [`cut_through_tx`](Self::cut_through_tx).
    pub cut_through_rx: bool,
    /// `Some` would enable `CONFIG0.FTSE` and pick the width via `FTSS` (requires `STDCAP.FTSC`).
    ///
    /// **Not implemented yet:** an enabled receive timestamp is prepended to the frame data and
    /// `SWO` points at it, but the receive path ignores the footer's `RTSA` / `RTSP` bits and
    /// would treat those timestamp bytes as frame content. Until it strips them,
    /// [`configure`](TC6HighPrio::configure) rejects a `Some` here with
    /// [`ConfigError::NotImplemented`].
    pub timestamp: Option<TimestampSize>,
    /// `Some` sets this node's PLCA ID / node count and enables PLCA; `None` leaves the PHY's
    /// PLCA registers untouched (strap defaults / plain CSMA/CD).
    pub plca: Option<PlcaConfig>,
    /// Value written to `IMASK0`. Defaults to [`registers::IMask0::RESET_VALUE`].
    pub interrupt_mask: registers::IMask0,
    /// Assert `RESET.SWRESET` as the first step of [`configure`](TC6HighPrio::configure), before
    /// waiting for `STATUS0.RESETC`. Defaults to `true`.
    ///
    /// `SYNC` only clears on a reset, so after an MCU-only reboot (PHY still powered and already
    /// configured) a `configure` that skipped this would find `RESETC` already clear and fail
    /// with [`ConfigError::ResetNotComplete`]. Set `false` only if the caller guarantees the
    /// MAC-PHY was just hardware-reset / power-cycled and issues its own reset otherwise.
    pub software_reset: bool,
    /// Max busy-wait iterations for a single control transfer to report complete before
    /// [`configure`](TC6HighPrio::configure) gives up with [`ConfigError::Timeout`]. `configure`
    /// also keeps it for the blocking register functions
    /// ([`read_register`](TC6HighPrio::read_register) and friends).
    ///
    /// This is a count of [`Tc6Interface::transfer_complete`] polls, **not a time**: the driver
    /// has no clock. With a blocking interface such as `EmbeddedHalImplementation` the transfer
    /// has already finished when it is started, so the first poll succeeds and the limit is
    /// never reached. With a DMA/async interface each poll is one quick status check, so the
    /// timeout in seconds shrinks as the CPU gets faster. Size it for the slowest case: the
    /// longest control transaction (`4 * (CONTROL_MAX_REGS + 2)` = 40 bytes) at your SPI clock,
    /// divided by the time one poll takes on your CPU, with a wide margin.
    pub transfer_poll_limit: u32,
    /// Max `STATUS0` re-reads waiting for `RESETC` before [`ConfigError::ResetNotComplete`].
    pub reset_poll_limit: u32,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            tx_credit_threshold: TxCreditThreshold::One,
            cut_through_tx: false,
            cut_through_rx: false,
            timestamp: None,
            plca: None,
            interrupt_mask: registers::IMask0::RESET_VALUE,
            software_reset: true,
            transfer_poll_limit: DEFAULT_TRANSFER_POLL_LIMIT,
            reset_poll_limit: 64,
        }
    }
}

/// Error from [`TC6HighPrio::configure`] / [`split_configured`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConfigError<E> {
    /// A control transaction was already pending/in flight when `configure` was called.
    Busy,
    /// A control transfer never reported complete within `Config::transfer_poll_limit`.
    Timeout,
    /// `STATUS0.RESETC` never became set within `Config::reset_poll_limit` reads.
    ResetNotComplete,
    /// `IDVER` read back as all-zero or all-ones: MISO is stuck, there is no clock, or the SPI
    /// bus is misconfigured (wrong mode / bit order). Check the transport before the device -
    /// SPI **mode 0** (CPOL = 0, CPHA = 0), MSB first, SCK within the MAC-PHY's rated maximum.
    BusUnresponsive,
    /// `IDVER` read back a plausible value but `MAJVER` is 0 - the bus is talking but this is
    /// not a TC6 v1.x device (or, less likely, a subtle SPI sampling error).
    BadVersion { major: u8, minor: u8 },
    /// A requested feature is not advertised in `STDCAP` (the `&str` names it).
    Unsupported(&'static str),
    /// A requested feature is advertised by the chip but this driver does not implement it yet
    /// (the `&str` names it). Enabling it anyway would corrupt the data path, so `configure`
    /// refuses rather than half-wiring it.
    NotImplemented(&'static str),
    /// `CONFIG0` read-back did not show `SYNC` set with a 64-byte chunk payload.
    SyncFailed,
    /// The MAC-PHY rejected a control command header (parity error / HDRB). If this happens on
    /// the *first* command of `configure`, suspect the SPI mode / bit order rather than the
    /// header: a wrong sampling edge corrupts every header consistently.
    HeaderRejected,
    /// The underlying interface failed to run a transfer.
    Interface(E),
}

impl<T, RxP, TxP, const M: usize, const NQ: usize> TC6HighPrio<T, RxP, TxP, M, NQ>
where
    T: Tc6Interface,
    RxP: BoxPool<Data = FrameBuffer<M>> + 'static,
    TxP: BoxPool<Data = FrameBuffer<M>> + 'static,
{
    // --- blocking bring-up (register configuration, spec Sections 7.6 / 9.2) ---

    /// `true` once [`configure`](Self::configure) has driven `CONFIG0.SYNC = 1` and no receive
    /// footer has since reported `SYNC = 0`. While this is `false` the MAC-PHY ignores transmit
    /// data chunks and delivers no receive frames.
    pub fn is_configured(&self) -> bool {
        self.sync_established
    }

    /// Shared access to the underlying transport.
    pub fn interface(&self) -> &T {
        &self.interface
    }

    /// Exclusive access to the underlying transport, for diagnostics and hardware tests (e.g.
    /// changing the SPI clock, or injecting malformed transactions).
    ///
    /// Only use it while the driver is idle: no transfer in flight and no control transaction
    /// pending ([`control_busy`](Self::control_busy) is `false`). Anything sent through it
    /// bypasses the driver's state machine, and the driver will not know the chip's state changed.
    pub fn interface_mut(&mut self) -> &mut T {
        &mut self.interface
    }

    /// `true` if a data-chunk footer has reported `EXST = 1` (a `STATUS0` / `STATUS1` bit is set
    /// and unmasked, spec Section 7.3.7) and [`service_extended_status`](Self::service_extended_status)
    /// has not cleared it since.
    pub fn extended_status_pending(&self) -> bool {
        self.extended_status_pending
    }

    /// If a data-chunk footer has reported `EXST = 1` since the last call, do a blocking control
    /// read of `STATUS0`, log every error/event bit that is set, write those bits back to clear
    /// them (write-1-to-clear, spec Section 9.2.8), clear the pending latch, and return the
    /// `STATUS0` snapshot. Returns `Ok(None)` when nothing is pending.
    ///
    /// TC6 spec Section 7.7: on `EXST` the host must control-read the status registers and act on
    /// them. This handles `STATUS0`; a MAC-PHY that also raises `EXST` from `STATUS1` or vendor
    /// registers needs the application to read those as well. The returned bits let the caller
    /// react further (e.g. count `RXBOE` / `TXBOE`, or re-run [`configure`](Self::configure) if
    /// the chip has clearly desynced).
    ///
    /// Like [`configure`](Self::configure) this busy-polls each control transfer and never enters
    /// the data path, so it needs no interrupt source wired up - but it must be called with the
    /// control path idle (no register op pending or in flight), otherwise it returns
    /// [`ConfigError::Busy`].
    pub fn service_extended_status(
        &mut self,
        poll_limit: u32,
    ) -> Result<Option<registers::Status0>, ConfigError<T::Error>> {
        use registers::{Status0, addr, mms};

        if !self.extended_status_pending {
            return Ok(None);
        }
        if self.control_busy() {
            warn!("tc6: service_extended_status() called while the control path is busy");
            return Err(ConfigError::Busy);
        }

        let raw = self.read_reg_blocking(mms::STANDARD, addr::STATUS0, poll_limit)?;
        let s0 = Status0::from_raw(raw);
        warn!("tc6: servicing extended status - STATUS0 = {:#010x}", raw);
        if s0.txpe() {
            warn!("tc6:   TXPE  - transmit protocol error (bad tx data chunk sequence)");
        }
        if s0.txboe() {
            warn!("tc6:   TXBOE - transmit buffer overflow (frame dropped)");
        }
        if s0.txbue() {
            warn!("tc6:   TXBUE - transmit buffer underflow (cut-through)");
        }
        if s0.rxboe() {
            warn!("tc6:   RXBOE - receive buffer overflow (frame data lost)");
        }
        if s0.lofe() {
            warn!("tc6:   LOFE  - loss of framing (CSn deasserted early)");
        }
        if s0.hdre() {
            warn!("tc6:   HDRE  - header parity error");
        }
        if s0.txfcse() {
            warn!("tc6:   TXFCSE - transmit FCS validation error");
        }
        if s0.cdpe() {
            warn!("tc6:   CDPE  - control data protection error");
        }

        // Write-1-to-clear exactly the latched bits we just observed. PHYINT (read-only) and the
        // reserved bits are outside DEFINED_MASK; RESETC and the TTSCAx capture-available bits
        // clear too if they were set, which is fine here (timestamps are not enabled).
        let clear = raw & Status0::DEFINED_MASK;
        if clear != 0 {
            self.write_reg_blocking(mms::STANDARD, addr::STATUS0, clear, poll_limit)?;
        }

        self.extended_status_pending = false;
        Ok(Some(s0))
    }

    /// Run the mandatory bring-up sequence, busy-polling each control transfer to completion:
    ///
    /// 0. if `cfg.software_reset`, assert `RESET.SWRESET`,
    /// 1. wait for `STATUS0.RESETC` (reset complete),
    /// 2. sanity-check `IDVER` (all-zero / all-ones => [`ConfigError::BusUnresponsive`]), log `PHYID`,
    /// 3. gate requested features on `STDCAP`,
    /// 4. write `CONFIG0` with a 64-byte chunk payload and `SYNC = 1`,
    /// 5. read `CONFIG0` back and confirm `SYNC` stuck,
    /// 6. program `IMASK0`,
    /// 7. clear latched `STATUS0` bits (including `RESETC`),
    /// 8. if `cfg.plca` is set, write this node's PLCA ID / node count and enable PLCA.
    ///
    /// Call this from init (or after a footer reports `SYNC = 0`) *before* servicing data
    /// traffic. It only touches control registers - the frame queues are untouched - and it
    /// polls [`Tc6Interface::transfer_complete`] directly, so it does not need an interrupt
    /// source wired up yet. On [`ConfigError::Timeout`] the state machine is returned to idle,
    /// but a genuinely stuck transfer may still be running on the interface.
    pub fn configure(&mut self, cfg: &Config) -> Result<(), ConfigError<T::Error>> {
        use registers::{Config0, IdVer, PhyId, Reset, Status0, StdCap, addr, mms};

        info!("tc6: configure() starting bring-up");

        if self.control_busy() {
            warn!("tc6: configure() called while a control transaction is busy");
            return Err(ConfigError::Busy);
        }
        self.transfer_poll_limit = cfg.transfer_poll_limit;

        // 0. software reset. SYNC only clears on a reset, so without this an MCU-only reboot
        //    (PHY still configured, RESETC already clear) would never satisfy step 1. The reset
        //    takes effect once CSn deasserts at the end of this control write (spec 9.2.4);
        //    step 1 then polls RESETC back to 1.
        if cfg.software_reset {
            debug!("tc6: configure() asserting RESET.SWRESET");
            self.write_reg_blocking(
                mms::STANDARD,
                addr::RESET,
                Reset::SWRESET.raw(),
                cfg.transfer_poll_limit,
            )?;
        }

        // 1. reset complete?
        let mut tries = 0u32;
        loop {
            let s0 = Status0::from_raw(self.read_reg_blocking(
                mms::STANDARD,
                addr::STATUS0,
                cfg.transfer_poll_limit,
            )?);
            if s0.resetc() {
                break;
            }
            tries += 1;
            if tries >= cfg.reset_poll_limit {
                warn!(
                    "tc6: configure() gave up waiting for STATUS0.RESETC after {} reads",
                    tries
                );
                return Err(ConfigError::ResetNotComplete);
            }
        }
        debug!("tc6: reset complete after {} STATUS0 read(s)", tries + 1);

        // 2. is anyone home? An all-zero / all-ones IDVER means the SPI bus itself is not
        //    working (no clock, MISO stuck, wrong mode / bit order) rather than "wrong chip".
        let id_raw = self.read_reg_blocking(mms::STANDARD, addr::IDVER, cfg.transfer_poll_limit)?;
        if id_raw == 0x0000_0000 || id_raw == 0xFFFF_FFFF {
            error!(
                "tc6: IDVER read back {:#010x} - SPI bus unresponsive (check clock / mode 0 / MSB-first / wiring)",
                id_raw
            );
            return Err(ConfigError::BusUnresponsive);
        }
        let id = IdVer::from_raw(id_raw);
        if id.majver() == 0 {
            error!(
                "tc6: implausible IDVER (major={}, minor={}) - not a TC6 device or SPI sampling error",
                id.majver(),
                id.minver()
            );
            return Err(ConfigError::BadVersion {
                major: id.majver(),
                minor: id.minver(),
            });
        }
        // PHYID corroborates: a real integrated PHY reports a non-zero OUI-derived value. Log it
        // for bring-up (a 0 / all-ones here alongside a good IDVER hints at a marginal bus)
        // without failing on it - some parts legitimately gate PHYID behind PHY power-up.
        let phy_id = PhyId::from_raw(self.read_reg_blocking(
            mms::STANDARD,
            addr::PHYID,
            cfg.transfer_poll_limit,
        )?);
        debug!(
            "tc6: IDVER major={} minor={}, PHYID oui={:#08x} model={:#04x} rev={}",
            id.majver(),
            id.minver(),
            phy_id.oui(),
            phy_id.model(),
            phy_id.revision()
        );

        // 3. capability gating
        let cap = StdCap::from_raw(self.read_reg_blocking(
            mms::STANDARD,
            addr::STDCAP,
            cfg.transfer_poll_limit,
        )?);
        if cfg.cut_through_tx || cfg.cut_through_rx {
            if !cap.ctc() {
                warn!("tc6: cut-through requested but STDCAP.CTC is not set");
                return Err(ConfigError::Unsupported("cut-through"));
            }
            // The data path has no cut-through handling (no TXBUE recovery, no partial-frame
            // termination), so enabling CONFIG0.TXCTE/RXCTE would misbehave under load. Refuse
            // rather than half-wire it - same policy as `timestamp`.
            warn!("tc6: cut-through requested but the data path has no cut-through handling yet");
            return Err(ConfigError::NotImplemented("cut-through"));
        }
        if cfg.timestamp.is_some() {
            if !cap.ftsc() {
                warn!("tc6: frame timestamps requested but STDCAP.FTSC is not set");
                return Err(ConfigError::Unsupported("frame timestamp"));
            }
            // The RX path does not strip the prepended receive timestamp yet (footer RTSA/RTSP
            // are ignored), so enabling FTSE would feed timestamp bytes to smoltcp as frame
            // data. Refuse rather than corrupt every received frame.
            warn!("tc6: frame timestamps requested but the receive path does not handle RTSA yet");
            return Err(ConfigError::NotImplemented("frame timestamp"));
        }

        // 4. write CONFIG0 with the driver's required 64-byte chunk payload and SYNC = 1.
        //    TXCTE/RXCTE and FTSE/FTSS are intentionally left clear - `cfg.cut_through_*` and
        //    `cfg.timestamp` are rejected above until the data path supports them.
        let c0 = Config0::from_raw(0)
            .with_cps(6) // 2^6 = 64-byte data chunk payload
            .with_txcthresh(cfg.tx_credit_threshold.bits())
            .with_sync(true);
        self.write_reg_blocking(
            mms::STANDARD,
            addr::CONFIG0,
            c0.raw(),
            cfg.transfer_poll_limit,
        )?;

        // 5. confirm SYNC stuck (spec 7.6: once set it cannot be written back to 0)
        let read_back = Config0::from_raw(self.read_reg_blocking(
            mms::STANDARD,
            addr::CONFIG0,
            cfg.transfer_poll_limit,
        )?);
        if !read_back.sync() || read_back.cps() != 6 {
            error!(
                "tc6: CONFIG0 read-back bad (sync={}, cps={}) - SYNC did not stick",
                read_back.sync(),
                read_back.cps()
            );
            return Err(ConfigError::SyncFailed);
        }

        // 6. interrupt mask
        self.write_reg_blocking(
            mms::STANDARD,
            addr::IMASK0,
            cfg.interrupt_mask.raw(),
            cfg.transfer_poll_limit,
        )?;

        // 7. clear RESETC and any latched errors
        self.write_reg_blocking(
            mms::STANDARD,
            addr::STATUS0,
            Status0::clear_all().raw(),
            cfg.transfer_poll_limit,
        )?;

        // 8. optional PLCA node setup (PHY MMS 4): write ID + node count, then enable
        if let Some(p) = cfg.plca {
            debug!(
                "tc6: enabling PLCA (node_id={}, node_count={})",
                p.node_id, p.node_count
            );
            use registers::{PlcaCtrl0, PlcaCtrl1, plca};
            self.write_reg_blocking(
                mms::PHY_VS_PLCA,
                plca::CTRL1,
                PlcaCtrl1::from_raw(0)
                    .with_id(p.node_id)
                    .with_node_count(p.node_count)
                    .raw(),
                cfg.transfer_poll_limit,
            )?;
            self.write_reg_blocking(
                mms::PHY_VS_PLCA,
                plca::CTRL0,
                PlcaCtrl0::from_raw(0).with_enable(true).raw(),
                cfg.transfer_poll_limit,
            )?;
        }

        // The chip's transmit buffer is empty right after bring-up, so there is at least one
        // credit; seed optimistically so the first transmit chunk can go out before any footer
        // has been seen. `rx_chunks_available` stays 0 - the first receive poll of a burst is
        // driven by a fresh interrupt, not by this counter.
        self.rx_chunks_available = 0;
        self.tx_credits = 1;
        // Step 7 above cleared STATUS0, so any EXST latched from a footer seen before bring-up is
        // stale now.
        self.extended_status_pending = false;
        self.sync_established = true;
        info!("tc6: configure() complete - CONFIG0.SYNC established, data path live");
        Ok(())
    }

    fn read_reg_blocking(
        &mut self,
        mms: u8,
        addr: u16,
        poll_limit: u32,
    ) -> Result<u32, ConfigError<T::Error>> {
        let res = self
            .run_control_blocking(
                PendingControl {
                    write: false,
                    mms,
                    addr,
                    count: 1,
                    values: [0; CONTROL_MAX_REGS],
                },
                poll_limit,
            )
            .map_err(Self::config_err)?;
        if res.header_bad {
            warn!(
                "tc6: MAC-PHY rejected control header for read mms={} addr={:#06x}",
                mms, addr
            );
            return Err(ConfigError::HeaderRejected);
        }
        Ok(res.values()[0])
    }

    fn write_reg_blocking(
        &mut self,
        mms: u8,
        addr: u16,
        value: u32,
        poll_limit: u32,
    ) -> Result<(), ConfigError<T::Error>> {
        let mut values = [0; CONTROL_MAX_REGS];
        values[0] = value;
        let res = self
            .run_control_blocking(
                PendingControl {
                    write: true,
                    mms,
                    addr,
                    count: 1,
                    values,
                },
                poll_limit,
            )
            .map_err(Self::config_err)?;
        if res.header_bad {
            warn!(
                "tc6: MAC-PHY rejected control header for write mms={} addr={:#06x}",
                mms, addr
            );
            return Err(ConfigError::HeaderRejected);
        }
        Ok(())
    }

    /// Map a blocking-transaction error onto [`ConfigError`] for the bring-up helpers above,
    /// which only issue single-register transactions and don't check write echoes.
    fn config_err(e: RegisterError<T::Error>) -> ConfigError<T::Error> {
        match e {
            RegisterError::Busy => ConfigError::Busy,
            RegisterError::Timeout => ConfigError::Timeout,
            RegisterError::HeaderRejected => ConfigError::HeaderRejected,
            RegisterError::Interface(e) => ConfigError::Interface(e),
            RegisterError::UnsupportedCount | RegisterError::EchoMismatch { .. } => {
                unreachable!("bring-up helpers use count 1 and skip the echo check")
            }
        }
    }
}
