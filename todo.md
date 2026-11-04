# hermes-tc6 — TODO

Status: protocol logic is implemented and unit-tested against `fake_tc6` + hand-rolled
interface spies. **Never run against real silicon.** PCBs are soldered, so hardware bring-up
is next.

---

## Blocking real-hardware bring-up

- [ ] **Test against a real MAC-PHY.** Everything so far is validated against `fake_tc6`, which
      is written from the same reading of the spec as the driver — a shared misinterpretation is
      invisible to the tests. First target: get `configure()` to return `Ok` against the
      physical chip, with a logic analyzer on the bus.
- [ ] **Write a real non-blocking `Tc6Interface`** for the actual SPI + DMA peripheral. Only
      `EmbeddedHalImplementation` (fully blocking, `transfer_complete()` always `true`) exists
      today; `on_interrupt`'s yield / re-entry path is exercised only by `PollingFake`. This is
      the transport the split high/low-prio design is actually for.
      - [ ] Validate the `DMAPtr` raw-pointer buffer handoff and buffer liveness across a
            completion IRQ under the real peripheral.
- [ ] **Cache maintenance** in that interface impl: clean the TX chunk before the transfer,
      invalidate the RX `FrameBuffer` tail (the 68-byte landing zone) after. Nothing does this
      yet. The `Tc6Interface` trait should document it as an implementor responsibility.
- [ ] **`resync_timer` / CSn hold on data ↔ control switch** (spec §7.2, §7.5.2). The host must
      hold CSn deasserted ≥ the MAC-PHY's `resync_timer` when switching between a data
      transaction and a control transaction. Currently unenforced — `handle_idle` starts a
      control transaction mid data-path (`service_extended_status`, user `begin_register_read`)
      with no delay.
      - [ ] Add a `mode_barrier()` hook (or a `mode` arg on the `begin_*` calls) to
            `Tc6Interface`, called by the driver only at a data↔control edge. Blocking impl:
            `delay_ns(resync_ns)`. Async impl: arm a timer / return "re-poll in N ns" using a
            monotonic `now()`, **not** a blocking delay in the state machine.
      - [ ] Consider stopping CSn deassertion *between data chunks* entirely (one `transaction()`
            spanning a burst of `Operation`s). Removes the per-chunk resync constraint and is a
            throughput win. Bigger refactor.
- [ ] **Verify CSn stays asserted across the two `Operation::Transfer`s** in
      `EmbeddedHalImplementation::begin_send_block` (header, then payload). The `SpiDevice`
      contract says it must; real HAL impls vary. A gap here = loss-of-framing.
- [ ] **Concurrency model**: confirm `TC6HighPrio` / `TC6LowPrio` `Send`/`Sync` bounds, add an
      RTIC (or equivalent) integration example, and stress `on_interrupt` re-entrancy under real
      preemption.

## Correctness / robustness

- [ ] **Guard `FrameBuffer::get_mut_at_end(68)`** in `handle_idle` / `handle_transmit_complete`.
      With `M >= MIN_FRAME_BUFFER_CAPACITY` (1600) it can't overflow for spec-compliant frames
      (≤ 1518), but a non-compliant oversized frame from the MAC would panic. Drop the frame
      instead. (`handle_data_valid`'s `add_len` calls are already guarded via `spare_capacity`.)
- [ ] **`MaybeUninit` UB on the partial final TX block**: the multi-block path forms a
      `&mut [u8]` over not-yet-written `FrameBuffer` capacity. Benign on real HW/DMA, technically
      UB. Fix: zero `[len, round_up(len, 64))` before the last send, or accept as
      non-miri-clean and document.
- [ ] **`on_interrupt` ISR duration** with the blocking interface: up to
      `MAX_INTERRUPT_STEPS` (64) chunks drained synchronously (~2 ms at 15 MHz SPI). Fine for
      now; revisit if it bites a real app. Non-issue with a yielding DMA interface.

## Features deferred (currently rejected by `configure`, not silently ignored)

- [ ] **Frame timestamp RX path**: strip the `RTSA`/`RTSP` prepended receive timestamp before
      handing the frame to smoltcp. Until then `Config::timestamp` returns
      `ConfigError::NotImplemented("frame timestamp")`.
- [ ] **Cut-through** (`CONFIG0.TXCTE`/`RXCTE`): needs TXBUE recovery and partial-frame
      termination in the data path. `Config::cut_through_*` returns
      `ConfigError::NotImplemented("cut-through")`.
- [ ] **`CONFIG0.PROTE`** (protected control transfers — each 32-bit word sent with its ones'
      complement, spec §7.4.4). Control path does unprotected single-word transfers only. Add if
      control-bus integrity matters in the target environment.
- [ ] **SEQ bit / chunk retransmission** (spec §7.3.8.2, §7.5.1). Without it, a transmit-buffer
      overflow or a mid-frame header-parity error drops the frame with no resume. Acceptable for
      v1; needed for a reliability story.

## Portability

- [ ] **PLCA register map is a guess** (`registers.rs` says so). The 0xCA00 block roughly
      follows the OPEN Alliance PLCA Register Map across LAN865x / NCN26010 / ADIN110x, but not
      guaranteed (addresses, MMD unlock). Plan: a `PlcaRegs { ctrl0, ctrl1, ... }` field on
      `Config` with `PlcaRegs::open_alliance()` (default) / `::lan865x()` / `::adin1110()`
      constructors. Promote to a `PhyProfile` trait only if per-PHY knowledge spreads (link
      status, MMD unlock, PHY IRQ handling).

## Cleanup

- [ ] Resolve the pre-existing clippy warnings: missing `# Safety` docs on `FrameBuffer` unsafe
      fns, `Default for FrameBuffer`, `is_multiple_of` in header/footer parity.
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
  queued).
- Receive `FD` (Frame Drop) → buffer freed, frame not surfaced.
- `SYNC = 0` in a footer → `sync_established` cleared (host must re-`configure`).
- `capabilities()`: MTU 1514, `max_burst_size = NQ - 1`, `const` assertions on `M` / `NQ` in
  `split()`.
- SPI-mode contract documented on `Tc6Interface` / `EmbeddedHalImplementation`;
  `ConfigError::BusUnresponsive` for all-zero / all-ones `IDVER`; `PHYID` logged during
  bring-up; `HeaderRejected` doc points at SPI mode.
