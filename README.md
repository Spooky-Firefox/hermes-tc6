# hermes-tc6

A `no_std` Rust driver for the **OPEN Alliance TC6 MAC-PHY serial interface** — the SPI
protocol used to talk to 10BASE-T1S single-pair Ethernet MAC-PHYs (LAN8650/1, NCN26010,
ADIN110x, …). It plugs into [smoltcp](https://github.com/smoltcp-rs/smoltcp) as a
`phy::Device` so the usual IP/UDP/TCP stack runs over a multidrop T1S bus.

> ⚠️ **AI-generated README, not yet reviewed.** This file was drafted by an AI assistant
> from the source and project notes. It has **not** been checked by a human — treat details
> (API names, constants, behaviour claims) as unverified until someone confirms them.
>
> ⚠️ **Only partly validated on real silicon.** Bring-up (`configure()`) and control-mode
> register access work against a real LAN8651 (see
> [Hardware bring-up](#hardware-bring-up-pico-2--lan8651)). The **data path (frame TX/RX)
> has only been tested against the `fake_tc6` host emulator** and hand-rolled interface spies.
> There are known open bugs and gaps (see [Status](#status)).

---

## Why this driver

It targets systems with hard real-time requirements — designed to survive a continuous
stream of minimum-size frames (on the order of ~1700 clock cycles per frame budget on a
50 MHz MCU). The design choices that follow from that:

- **Non-blocking SPI interface trait** (`Tc6Interface`) instead of the blocking
  `embedded-hal` SPI trait, so the driver can kick off a transfer and do other work while
  it runs. Buffer arguments implement the `embedded-dma` `ReadBuffer` / `WriteBuffer`
  traits, so a DMA transport drops straight in. A blocking `embedded-hal` wrapper
  (`EmbeddedHalImplementation`) is provided for non-critical use.
- **Split high-priority / low-priority halves.** `TC6HighPrio` does the fast, time-critical
  SPI servicing; `TC6LowPrio` does the heavy TCP/IP work and implements `smoltcp::phy::Device`.
  The two can live at different RTIC priorities or on different cores.
- **Lock-free zero-copy hand-off.** `heapless` SPSC queues + box pools move ~1500-byte
  frame buffers between the two halves with no copy. Separate RX and TX pools avoid
  deadlock. `MaybeUninit` frame buffers avoid the zeroing cost.

---

## Architecture

```text
  ┌───────────────────┐   rx_ready SPSC queue    ┌──────────────────┐
  │   TC6HighPrio     │ ───────(Box<RxP>)──────► │   TC6LowPrio     │
  │  (IRQ priority)   │                          │  (idle / lower)  │
  │                   │ ◄──────(Box<TxP>)─────── │                  │
  │  on_interrupt()   │   tx_ready SPSC queue    │  smoltcp Device  │
  │  on_transmit_...  │                          │  (receive/       │
  │  configure()      │        TransmitSignal    │   transmit)      │
  │  begin_register_* │ ◄────────(wake)───────── │                  │
  └────────┬──────────┘                          └──────────────────┘
           │ Tc6Interface  (begin_send_block / begin_control_transaction /
           ▼                transfer_complete)
   ┌────────────────┐
   │  SPI / DMA     │  SPI mode 0, MSB first
   │  transport     │
   └───────┬────────┘
           ▼
   10BASE-T1S MAC-PHY  ── single pair ──  multidrop bus
```

- `split()` / `split_configured()` consume a `Tc6Interface`, `'static` queue storage, and a
  `TransmitSignal`, and return the `(TC6HighPrio, TC6LowPrio)` pair.
- The high-priority side is driven by `on_interrupt()` (call it from the chip's IRQ line
  *and* from SPI/DMA transfer-completion) and `on_transmit_ready()` (call it when
  `TransmitSignal::signal()` fires, i.e. the low-prio side queued a frame).
- Register access goes through `begin_register_read()` / `begin_register_write()` on the
  high-prio side; pump `on_interrupt()` and collect the result with `take_control_result()`.

---

## Cargo features

| Feature | Default | Effect |
| --- | --- | --- |
| `embedded-hal` | ✅ | Builds `EmbeddedHalImplementation`, the blocking `Tc6Interface` on top of an `embedded-hal` `SpiDevice`. Turn off (`default-features = false`) if you only use your own transport. |
| `defmt` | | Route this crate's internal logging to `defmt`. |
| `log` | | Route this crate's internal logging to the `log` facade. |

Enable **at most one** of `defmt` / `log` — both together is a compile error. With neither,
the log macros expand to nothing. `smoltcp` has its own mutually-exclusive `log` / `defmt`
features — configure those in your application crate.

`smoltcp` is pulled in with `default-features = false` and only `medium-ethernet`,
`proto-ipv4` and `socket-udp` (`medium-ethernet` implies `socket`, which needs at least one
socket kind). Its defaults include `std`, which would break every `no_std` target. Enable
whatever else you need (TCP, DHCP, IPv6, …) on `smoltcp` in your application crate; Cargo
merges the features.

> `cargo test --features defmt` will not link (no `defmt` global logger in a host binary).
> Run the host suite with no logging or `--features log`.

---

## Usage sketch

The box pool types must name a concrete `FrameBuffer<M>`, so the application declares the
pools and the `'static` queue storage; the generic driver takes them as parameters.

```rust
use hermes_tc6::tc6_driver::{self, Config, TransmitSignal};
use hermes_tc6::embedded_io_implementation::EmbeddedHalImplementation;
use hermes_tc6::frame_buffer::FrameBuffer;
use heapless::{box_pool, spsc::Queue};

// M >= 1600 (MIN_FRAME_BUFFER_CAPACITY); NQ >= 2 (usable depth NQ - 1)
const M: usize = 1600;
const NQ: usize = 8;

box_pool!(RxP: FrameBuffer<M>);
box_pool!(TxP: FrameBuffer<M>);

// ... in #[init]: `manage` backing memory for RxP / TxP, get `&'static mut Queue`s ...

struct Signal; // pend the high-prio task / software IRQ
impl TransmitSignal for Signal {
    fn signal(&mut self) { /* e.g. rtic::pend(Interrupt::TC6_TX) */ }
}

let iface = EmbeddedHalImplementation::new(spi_device); // spi_device: embedded-hal SpiDevice, mode 0
let (mut hi, mut lo) = tc6_driver::split_configured::<_, RxP, TxP, _, M, NQ>(
    iface, rx_ready, tx_ready, Signal, &Config::default(),
)?;

// high-prio context:  hi.on_interrupt()?  /  hi.on_transmit_ready()?
// low-prio context:   run smoltcp against `lo` (impl smoltcp::phy::Device)
```

### SPI bus requirements

TC6 is **SPI mode 0** (CPOL = 0, CPHA = 0), **MSB first**, SCK up to the MAC-PHY's rated
maximum (commonly 15–25 MHz — check the datasheet). The `Tc6Interface` impl does **not**
configure the bus; set it up before constructing the implementor. A wrong mode/bit order
usually surfaces as `ConfigError::BusUnresponsive` or `ConfigError::HeaderRejected`.

Per spec §7.2 the host must hold CSn deasserted for at least the MAC-PHY's `resync_timer`
when switching between a data transaction and a control transaction. This is **currently
the implementor's responsibility** and is not enforced by the driver or the blocking
wrapper (see `todo.md`).

---

## `Config` and bring-up

`Config::default()` is the minimal safe setup: unprotected control transfers, ≥ 1 transmit
credit before IRQn, no cut-through, no timestamps, PLCA untouched, every maskable interrupt
masked, 64-byte chunk payload (the data path assumes this). `configure()`:

1. optional `RESET.SWRESET` (`Config::software_reset`, default `true`),
2. wait for `STATUS0.RESETC`,
3. sanity-check `IDVER` (all-zero / all-ones ⇒ `BusUnresponsive`), gate on `MAJVER`,
4. `STDCAP` capability gating,
5. write `CONFIG0` + verify `SYNC`,
6. `IMASK0`, clear `STATUS0`,
7. optional PLCA (`PlcaConfig::follower` / `::coordinator`).

Features that are advertised by the chip but **not implemented yet** are rejected by
`configure()` rather than half-wired: `NotImplemented("cut-through")`,
`NotImplemented("frame timestamp")`. See `ConfigError` for the full set.

---

## Status

**Protocol logic is implemented and unit-tested against `fake_tc6` + interface spies.**
Hardware bring-up has started: on 2026-11-03 `configure()` returned `Ok` against a real LAN8651
on a Raspberry Pi Pico 2 (see [Hardware bring-up](#hardware-bring-up-pico-2--lan8651)), and an
18-test on-target suite (control path, reset handling, PLCA registers, footer layout, error
injection, SPI up to 25 MHz) passes. Frame TX/RX has not been exercised on hardware yet.

`todo.md` tracks the full list. Highlights:

- **No real non-blocking `Tc6Interface`** exists yet — only the fully blocking
  `EmbeddedHalImplementation`. The async `on_interrupt` yield/re-entry path is exercised
  only by a polling test fake.
- **Cache maintenance**, `resync_timer` / CSn hold, and CSn-across-`Operation` guarantees
  are unimplemented / unverified.
- **PLCA register map**: `IDVER` / `CTRL0` / `CTRL1` / `STATUS` are confirmed on the LAN8651
  (map ID `0x0A10`). `TOTMR` / `BURST` are still unverified.
- Deferred protocol features currently rejected by `configure()`: frame-timestamp RX path,
  cut-through, `CONFIG0.PROTE` protected control transfers, SEQ-bit chunk retransmission.
- Pre-existing clippy warnings (missing `# Safety` docs, etc.) are unresolved.
- `fake_tc6` is written from the same reading of the spec as the driver, so a shared
  misinterpretation is invisible to the tests. The first real-hardware milestone (`configure()`
  returning `Ok` on a physical chip) is done; the data path is next.
- **No vendor-specific init for the LAN8650/1.** The driver only does the generic TC6
  bring-up. Microchip specifies extra configuration writes for these parts (Microchip's Linux
  PHY driver applies a rev-B init sequence). Until that is added, don't trust electrical
  behaviour on the wire. Check the datasheet / app note.

See also the project memory note on known-broken paths pinned by `driver_tests`.

---

## Hardware bring-up (Pico 2 + LAN8651)

`examples/pico2-bringup/` is a standalone firmware crate (its own `Cargo.toml`, `memory.x`,
`.cargo/config.toml`), so the library's host test suite never sees its embedded-only
dependencies. It has three parts:

- `src/lib.rs`: board support shared by everything else. Pins/SPI setup, RESET_N / IRQ_N
  helpers, the `'static` pools and queues (`take_driver()`), pumped register read/write helpers,
  and a `raw` module that talks SPI directly, bypassing the driver, for wire-format checks and
  error injection.
- `src/main.rs` (`cargo run`): hardware-resets the MAC-PHY, configures it, dumps a few
  registers, then loops, servicing IRQ_N and logging `STATUS0` once a second.
- `tests/single_board.rs` (`cargo test --test single_board`): the on-target test suite for chip
  A, see below. Chip B is held in reset so it stays silent on the cable.
- `tests/two_board.rs` (`cargo test --test two_board`): chip B identity, crossed-line checks, SPI
  sweep on chip B, and a PLCA BEACON check across the cable in both directions. Needs both chips connected.
- `tests/wiring_probe.rs` (`cargo test --test wiring_probe`): GPIO-only line checks for when a
  chip doesn't answer. It always passes; read the log.

| LAN8651 | Chip A (SPI1) | Chip B (SPI0) | Notes |
| --- | --- | --- | --- |
| SCLK | GPIO10 | GPIO2 | SPI SCK |
| SDI | GPIO11 | GPIO3 | SPI TX (MOSI) |
| SDO | GPIO12 | GPIO4 | SPI RX (MISO) |
| CS_N | GPIO13 | GPIO5 | plain GPIO, driven by `embedded-hal-bus` `ExclusiveDevice` |
| IRQ_N | GPIO14 | GPIO6 | input, pull-up, polled |
| RESET_N | GPIO15 | GPIO7 | 10 ms low pulse at boot |

The two chips share power and are joined by a twisted pair, terminated at both ends.

Status (2026-11-04): `two_board` passes 6/6 with both chips on the cable. Each chip reports PLCA
inactive as a follower until the other starts as coordinator, then active, in both directions.
Frames have not been sent between the chips yet.

```sh
cd examples/pico2-bringup
cargo run                      # demo: flash via a Raspberry Pi Debug Probe, stream defmt logs
cargo test --test single_board # on-target test suite for chip A (~6 s)
cargo test -- spi_clock_sweep  # one test by name
```

Needs the `thumbv8m.main-none-eabihf` target, **probe-rs ≥ 0.32** and `flip-link`. With probe-rs
0.31, every test after the first fails with "The control block has been corrupted: The channel
mode flags are invalid". 0.31 keeps its RTT attachment across the per-test reset, and the RP2350
reset leaves SRAM scrambled. SPI runs at 1 MHz (`SPI_HZ`) by default for logic-analyzer-friendly
bring-up.

### On-target test suite

`tests/single_board.rs` uses [embedded-test](https://github.com/probe-rs/embedded-test).
probe-rs resets the MCU before every test, and the suite's `#[init]` pulses RESET_N, so every
test starts with a fresh MCU *and* a freshly reset MAC-PHY. This is why each test can call
`take_driver()`: the `'static` pools/queues can only be handed out once per boot. Logs go over
defmt-rtt. Semihosting is only used for test selection and pass/fail.

Results of the first run (2026-11-03, all 18 pass):

| Area | What was checked | Observed |
| --- | --- | --- |
| Wire format | Hand-built control read of `IDVER`, without the driver | header echoed unchanged, `0x11` |
| Wire format | Empty data chunk before `configure()` | footer `0x8000003F`: `EXST` set, `SYNC` = 0 |
| Wire format | Empty data chunk after `configure()` | footer `0x2000003F`: `SYNC` = 1, `TXC` 31 (BUFSTS 48, saturated to 5 bits) |
| Bring-up | `configure()` after hardware reset, with and without SWRESET | OK; `CONFIG0` `SYNC` + 64-byte chunks, `STATUS0` = 0 |
| Bring-up | 5× re-`configure()` without hardware reset | OK |
| Bring-up | re-`configure()` with `software_reset = false` | `ResetNotComplete`, as expected |
| IRQ | IRQ_N after reset / after configure / after first `on_interrupt()` | low / low / high |
| Reset | RESET_N pulsed under a running driver | `is_configured()` → `false`, re-`configure()` recovers |
| Control | `IMASK0` write/readback, 4 patterns | echo and readback match |
| Control | Burst reads of 2, 3, 8 registers vs single reads | identical |
| SPI | 1000 `IDVER` reads at 1 / 5 / 9.375 / 15 / 18.75 / 25 MHz | 0 errors at every clock |
| PLCA | MMS 4 `0xCA00` | `0x0A10`, the OPEN Alliance map |
| PLCA | `coordinator(8)` → `CTRL0` / `CTRL1` / `STATUS` | `0x8000` / `0x0800` / `PST` = 1 (lone coordinator beacons) |
| PLCA | `follower(3)` → `CTRL0` / `CTRL1` / `STATUS` | `0x8000` / `0x0003` / `PST` = 0 (no beacon) |
| Errors | Control header with flipped parity | echo `0xC0000001` (spec 7.5.1), `STATUS0.HDRE` |
| Errors | CS_N released after 34 of 68 bytes | `STATUS0.LOFE` |
| Errors | Data chunk with `EV` but no `SV` | `STATUS0.TXPE` |

The SPI sweep requires error-free reads up to `SWEEP_MUST_PASS_UP_TO_HZ` (25 MHz, the LAN8651's
rated maximum). Lower it if longer leads start failing.

First run (2026-11-03, T1S cable unconnected, no other node):

| Register | Value | Meaning |
| --- | --- | --- |
| `IDVER` | `0x00000011` | TC6 v1.1 |
| `PHYID` | `0x0007C1B4` | Microchip OUI, model `0x1B` (LAN8650/1), rev 4 |
| `STDCAP` | `0x000005E5` | capability bits |
| `CONFIG0` | `0x00008006` | `SYNC` set, 64-byte chunk payload |
| `STATUS0` | `0x00000000` | no errors latched |
| `BUFSTS` | `0x00003000` | 48 TX credits, 0 RX chunks |

IRQ_N was asserted after reset (`RESETC`). It deasserted after the first `on_interrupt()`
and stayed high.

### What can be tested with a single board

No peer node is needed for any of these. Ticked items are covered by `tests/single_board.rs`.

**Control path and SPI bus**

- [x] Hardware reset → `configure()` → `SYNC` established; register dump.
- [x] **SPI clock sweep**: read `IDVER` 1000 times each at 1, 5, 10, 15, 20
      and 25 MHz, and count mismatches. Gives the reliable max clock for this wiring.
- [x] **Write/readback**: write patterns to R/W registers and read them back. Check the echoed
      write data in `ControlResult::values()`. Check that multi-register reads (`count` 2..=8)
      match single reads, which exercises address post-increment.
- [x] **Reconfigure without a hardware reset** (MCU-only reboot): run `configure()` repeatedly
      with `software_reset = true`. With `software_reset = false` and no reset, expect
      `ResetNotComplete`.
- [x] **Reset under the driver**: pulse RESET_N while running and check that the next footer
      reports `SYNC = 0`, `is_configured()` goes `false`, and a re-`configure()` recovers.
- [x] **PLCA register map**: read MMS 4 `0xCA00` (OPEN Alliance PLCA
      `IDVER`, expected `0x0A1x`). Configure as `PlcaConfig::coordinator(n)` and read back
      `CTRL0`/`CTRL1`/`STATUS`. A lone coordinator should still send BEACONs, so PLCA status
      should look active even with no other nodes.

**Error injection** (drive `Tc6Interface` directly, bypassing the driver)

- [x] Control header with bad parity → `HDRB` / `header_bad`, `STATUS0.HDRE`.
- [x] Deassert CS_N mid-chunk → `STATUS0.LOFE`. (Not yet: footer `EXST` with `LOFEM`
      unmasked, and `service_extended_status()` clearing it.)
- [x] Bad `DV`/`SV`/`EV` sequence on data chunks → `STATUS0.TXPE`.
- [ ] Unmask bits via `Config::interrupt_mask` and check that IRQ_N / `EXST` follow.

**Data path without a peer**

- [ ] **Loopback** (the big one): enable the PHY's IEEE 802.3 loopback or the MAC's local
      loopback (check the LAN8650/1 datasheet for which the part supports). Then send frames of
      60, 64, 65, 127, 128 and 1514 bytes, plus back-to-back bursts, and check length and
      payload on the receive side. This covers the single- and multi-block TX/RX paths, footer
      `TXC`/`RCA` accounting and pool-exhaustion recovery on real silicon, which `fake_tc6`
      can't confirm independently.
- [ ] **TX into an open line** (no loopback): frames should leave the MAC (CSMA/CD, no PLCA).
      Check that TX credits recover, there is no `TXBOE`, and the MAC statistics counters
      (MMS 1) move. Collision counters on the unterminated line are informative too.
- [ ] **smoltcp on top**: hook `TC6LowPrio` into a smoltcp `Interface` and send periodic
      UDP broadcasts / ARP. This exercises `phy::Device`, `TransmitSignal` and
      `on_transmit_ready()`.

**Timing and robustness** (logic analyzer on SCLK/CS_N/SDI/SDO/IRQ_N)

- [ ] Confirm CS_N stays asserted across the two `Operation::Transfer`s in
      `EmbeddedHalImplementation::begin_send_block` (header then payload).
- [ ] Measure the CS_N-high gap at data ↔ control switches against `resync_timer`
      (spec §7.2).
- [ ] Measure `on_interrupt()` / `configure()` duration (GPIO toggle or the timer).
- [ ] Soak test: hours of the service loop with interleaved register reads and TX traffic,
      watching `STATUS0`.

**Needs a second node**: real RX from the wire, multi-node PLCA, collisions under load,
the minimum-size-frame throughput target, and interop with other stacks (e.g. Linux
`lan865x`).

---

## Repository layout

| Path | What |
| --- | --- |
| `src/tc6_driver.rs` | The split driver: `TC6HighPrio`, `TC6LowPrio`, `split*`, `Config`, `smoltcp::phy::Device` impl, `mod driver_tests`. |
| `src/tc6_interface.rs` | The `Tc6Interface` transport trait. |
| `src/embedded_io_implementation.rs` | `EmbeddedHalImplementation` — blocking `Tc6Interface` over `embedded-hal` `SpiDevice`. |
| `src/tc6_header.rs`, `src/tc6_footer.rs` | TC6 data-chunk header / footer bitfields + parity. |
| `src/registers.rs` | Register map (standard MMS + a best-effort PLCA block). |
| `src/frame_buffer.rs` | `FrameBuffer<M>` — the `MaybeUninit`-backed frame storage. |
| `src/bitfield.rs`, `src/fmt.rs` | Bitfield helper macro; logging shim. |
| `fake_tc6/` | Host-side TC6 device emulator, used as a dev-dependency in tests. |
| `examples/pico2-bringup/` | Standalone RP2350 firmware: board support (`src/lib.rs`), bring-up demo (`src/main.rs`), on-target test suite (`tests/single_board.rs`). |
| `OPEN_Alliance_10BASET1x_MAC-PHY_Serial_Interface_V1.1.pdf` | The spec this driver implements. |

---

## Building and testing

```sh
cargo build                      # default (embedded-hal wrapper on)
cargo build --no-default-features # bring your own Tc6Interface
cargo test                       # host suite, no logging backend
cargo test --features log        # host suite with `log`
cargo build --target thumbv8m.main-none-eabihf   # check the lib still builds no_std
(cd examples/pico2-bringup && cargo run)         # flash + run on hardware
(cd examples/pico2-bringup && cargo test)        # on-target test suite (probe-rs >= 0.32)
```

---

## Context

Part of a 10BASE-T1S project: T1S PCBs, this driver, and the `fake_tc6` emulator, groundwork
for a transport-agnostic secure bootloader over a multidrop bus. Built on `smoltcp`,
`heapless`, `embedded-hal`, `embedded-dma`.

## License

Not yet specified.
