# hermes-tc6 — TODO

Status: protocol logic is unit-tested against `fake_tc6` + hand-rolled interface spies, and
the polled data path runs on silicon (Pico 2 + LAN8651: `single_board`, `two_board`, `frames`).
Open items are tracked as GitHub issues; the numbers are next to each item.

---

## Blocking real-hardware bring-up

- [ ] **Finish hardware validation** (#14): IRQ-driven servicing, a full smoltcp stack between
      two boards, and enough traffic to wrap the rings. `configure()` and polled frame TX/RX
      already pass on silicon.
- [ ] **Received frames include the 4-byte FCS** (#15), found on silicon; docs assume it is
      stripped.
- [ ] **Write a real non-blocking `Tc6Interface`** (#7) for the actual SPI + DMA peripheral. Only
      `EmbeddedHalImplementation` (fully blocking, `transfer_complete()` always `true`) exists
      today; `on_interrupt`'s yield / re-entry path is exercised only by `PollingFake`. This is
      the transport the split high/low-prio design is actually for.
      - [ ] Validate the `DMAPtr` raw-pointer buffer handoff and buffer liveness across a
            completion IRQ under the real peripheral.
- [ ] **Cache maintenance** (#8) in that interface impl: clean the TX chunk before the transfer,
      invalidate the RX `FrameBuffer` tail (the 68-byte landing zone) after. Nothing does this
      yet. The `Tc6Interface` trait should document it as an implementor responsibility.
- [ ] **`resync_timer` / CSn hold on data ↔ control switch** (#9) (spec §7.2, §7.5.2). The host must
      hold CSn deasserted ≥ the MAC-PHY's `resync_timer` when switching between a data
      transaction and a control transaction. Currently unenforced — `handle_idle` starts a
      control transaction mid data-path (`service_extended_status`, user `begin_register_read`,
      the blocking `read_register` / `write_register` family)
      with no delay.
      - [ ] Add a `mode_barrier()` hook (or a `mode` arg on the `begin_*` calls) to
            `Tc6Interface`, called by the driver only at a data↔control edge. Blocking impl:
            `delay_ns(resync_ns)`. Async impl: arm a timer / return "re-poll in N ns" using a
            monotonic `now()`, **not** a blocking delay in the state machine.
      - [ ] Consider stopping CSn deassertion *between data chunks* entirely (one `transaction()`
            spanning a burst of `Operation`s). Removes the per-chunk resync constraint and is a
            throughput win. Bigger refactor.
- [ ] **`Config::transfer_poll_limit` counts polls, not time.** (#10) The blocking control path
      (`configure`, `service_extended_status`, `read_register` / `write_register` and friends)
      spins on `transfer_complete()` up to N times. With the blocking `EmbeddedHalImplementation`
      the first poll always succeeds, so it doesn't matter yet. With a DMA interface the real
      timeout becomes N × (one poll on this CPU), so a fast CPU with a slow SPI clock can time
      out a transfer that is still running. Documented on the field for now. Decide when the
      DMA interface is written; it needs the same time source as the `resync_timer` item:
      - [ ] Option: a timeout / `now()` hook on `Tc6Interface`.
      - [ ] Option: the blocking functions take a deadline closure from the caller.
- [ ] **Verify CSn stays asserted across the two `Operation::Transfer`s** in
      `EmbeddedHalImplementation::begin_send_block` (#11) (header, then payload). The `SpiDevice`
      contract says it must; real HAL impls vary. A gap here = loss-of-framing.
- [ ] **Concurrency model** (#12): confirm `TC6HighPrio` / `TC6LowPrio` `Send`/`Sync` bounds, add an
      RTIC (or equivalent) integration example, and stress `on_interrupt` re-entrancy under real
      preemption.

## Correctness / robustness

- [ ] **`on_interrupt` ISR duration** (#13) with the blocking interface: up to
      `MAX_INTERRUPT_STEPS` (64) chunks drained synchronously (~2 ms at 15 MHz SPI). Fine for
      now; revisit if it bites a real app. Non-issue with a yielding DMA interface.

## Vendor / performance follow-ups

- [ ] **`lan865x` module** behind a feature flag (#20): `enable_mac()`, rev-B init, FCS strip,
      MAC address.
- [ ] **Measure TX chunk-padding overhead** on real traffic (#24) before considering an
      exact-length commit + scratch copy.
- [ ] **bbqueue never rewinds an empty ring** (#25): costs one worst-case record of slack per
      ring; possible upstream proposal.

## Features deferred (currently rejected by `configure`, not silently ignored)

- [ ] **Frame timestamp RX path** (#16): strip the `RTSA`/`RTSP` prepended receive timestamp before
      handing the frame to smoltcp. Until then `Config::timestamp` returns
      `ConfigError::NotImplemented("frame timestamp")`.
- [ ] **Cut-through** (#17) (`CONFIG0.TXCTE`/`RXCTE`): needs TXBUE recovery and partial-frame
      termination in the data path. `Config::cut_through_*` returns
      `ConfigError::NotImplemented("cut-through")`.
- [ ] **`CONFIG0.PROTE`** (#18) (protected control transfers — each 32-bit word sent with its ones'
      complement, spec §7.4.4). Control path does unprotected single-word transfers only. Add if
      control-bus integrity matters in the target environment.
- [ ] **SEQ bit / chunk retransmission** (#19) (spec §7.3.8.2, §7.5.1). Without it, a transmit-buffer
      overflow or a mid-frame header-parity error drops the frame with no resume. Acceptable for
      v1; needed for a reliability story.

## Portability

- [ ] **Cortex-M0 / thumbv6m** (#21): needs bbqueue's `CsCoord` (critical-section) instead of
      `AtomicCoord`; bbqueue 0.7's portable-atomic features don't help.
- [ ] **PLCA register map is a guess** (#22) (`registers.rs` says so). The 0xCA00 block roughly
      follows the OPEN Alliance PLCA Register Map across LAN865x / NCN26010 / ADIN110x, but not
      guaranteed (addresses, MMD unlock). Plan: a `PlcaRegs { ctrl0, ctrl1, ... }` field on
      `Config` with `PlcaRegs::open_alliance()` (default) / `::lan865x()` / `::adin1110()`
      constructors. Promote to a `PhyProfile` trait only if per-PHY knowledge spreads (link
      status, MMD unlock, PHY IRQ handling).

## Cleanup

Tracked together in #23.

- [ ] Resolve the clippy warnings: `is_multiple_of` in header/footer parity.
- [ ] `fmt.rs` / test module: unused-import warnings under `--features log`.
- [ ] `embedded_io_implementation` module name vs the `EmbeddedHalImplementation` type it
      exports — pick one.
- [ ] `Config` doc: default is "cut-through / timestamps off" **and** those knobs are currently
      *rejected*, not just defaulted — keep the doc honest as features land.

---

## Done

- TXC transmit-credit gating; RCA-driven `on_interrupt` drain loop with a step cap.
- `configure()`: optional `SWRESET`, `RESETC` wait, `IDVER`/`STDCAP` gating, `CONFIG0` +
  `SYNC`, `IMASK0`, `STATUS0` clear, optional PLCA.
- Corrupt data-chunk footer (parity fail / `HDRB` / `0xC0000001`): chunk treated as not
  accepted, transmit cursor held, block re-issued; borrowed continuation-header bytes restored.
- `EXST` latch + `service_extended_status()`: control-read `STATUS0`, log set bits,
  write-1-to-clear.
- `handle_data_valid` de-panicked: every footer-field `assert!`/`panic!` → logged frame drop;
  `FrameBuffer::spare_capacity()` guards the `add_len` calls.
- RX-pool-exhaustion no longer panics (`handle_idle` defers; NORX set when a TX frame is
  queued). Now a full RX ring, same behaviour.
- Packet storage is a caller-owned `HermesStorage` (two bbqueue framed rings) instead of
  `box_pool!` + `'static` SPSC queues (#3, #4, #5); TX chunk padding is initialised and zeroed,
  so the partial final block no longer exposes uninitialised memory (#6).
- Receive `FD` (Frame Drop) → buffer freed, frame not surfaced.
- Receive buffer sizing: `MIN_FRAME_BUFFER_CAPACITY` now counts the SWO start offset
  (`60 + 1521 + 68 = 1649`; the old 1600 panicked on a max-size frame at a high SWO).
  `handle_idle` drops frames that outgrow `M` instead of panicking in `get_mut_at_end`; a
  concatenated SV+EV chunk on a fresh buffer keeps the new frame.
- `SYNC = 0` in a footer → `sync_established` cleared (host must re-`configure`).
- `capabilities()`: MTU 1514, `max_burst_size` from the TX ring size, `const` assertions on
  the ring sizes in `split()`.
- SPI-mode contract documented on `Tc6Interface` / `EmbeddedHalImplementation`;
  `ConfigError::BusUnresponsive` for all-zero / all-ones `IDVER`; `PHYID` logged during
  bring-up; `HeaderRejected` doc points at SPI mode.
