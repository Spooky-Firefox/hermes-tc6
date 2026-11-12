//! Minimal hardware bring-up for hermes-tc6 on a Raspberry Pi Pico 2 (RP2350) wired to a
//! LAN8651 10BASE-T1S MAC-PHY. Wiring: see `src/lib.rs`.
//!
//! Hardware-reset the MAC-PHY, build the `TC6HighPrio` / `TC6LowPrio` pair, run `configure()`
//! with `Config::default()`, dump a few registers, then loop servicing IRQ_N and printing
//! STATUS0 once a second. No smoltcp stack yet. With no T1S cable and no other node, nothing
//! should be received.
//!
//! `cargo run` flashes and runs this; `cargo test` runs the on-target suite in `tests/`.

#![no_std]
#![no_main]

use defmt::{Debug2Format, error, info, warn};
use defmt_rtt as _;
use panic_probe as _;

use pico2_bringup as board;
use pico2_bringup::hermes_tc6::registers::{addr, mms};
use pico2_bringup::hermes_tc6::tc6_driver::Config;
use rp235x_hal as hal;

#[hal::entry]
fn main() -> ! {
    let (iface, mut ctl) = board::init();
    info!("hermes-tc6 pico2 bring-up");

    ctl.hw_reset();
    info!("RESET_N released, IRQ_N = {}", level(ctl.irq_asserted()));

    let cfg = Config::default();
    let (mut hi, _lo) = board::take_driver(iface);
    if let Err(e) = hi.configure(&cfg) {
        error!("configure() failed: {}", Debug2Format(&e));
        loop {
            cortex_m::asm::wfi();
        }
    }
    info!("configure() OK");

    for (name, m, a) in [
        ("IDVER", mms::STANDARD, addr::IDVER),
        ("PHYID", mms::STANDARD, addr::PHYID),
        ("STDCAP", mms::STANDARD, addr::STDCAP),
        ("CONFIG0", mms::STANDARD, addr::CONFIG0),
        ("STATUS0", mms::STANDARD, addr::STATUS0),
        ("STATUS1", mms::STANDARD, addr::STATUS1),
        ("BUFSTS", mms::STANDARD, addr::BUFSTS),
    ] {
        match hi.read_register(m, a) {
            Ok(v) => info!("{=str} = {=u32:#010x}", name, v),
            Err(e) => warn!("{=str} read failed: {}", name, Debug2Format(&e)),
        }
    }
    info!("is_configured = {}", hi.is_configured());

    let mut last_report = ctl.timer.get_counter();
    let mut irq_count: u32 = 0;
    loop {
        if ctl.irq_asserted() {
            irq_count = irq_count.wrapping_add(1);
            if let Err(e) = hi.on_interrupt() {
                error!("on_interrupt: {}", Debug2Format(&e));
            }
        }
        if board::take_tx_pending() {
            if let Err(e) = hi.on_transmit_ready() {
                error!("on_transmit_ready: {}", Debug2Format(&e));
            }
        }
        if hi.extended_status_pending() {
            match hi.service_extended_status(cfg.transfer_poll_limit) {
                Ok(Some(s0)) => info!("extended status: STATUS0 = {=u32:#010x}", s0.raw()),
                Ok(None) => {}
                Err(e) => error!("service_extended_status: {}", Debug2Format(&e)),
            }
        }

        let now = ctl.timer.get_counter();
        if (now - last_report).to_millis() >= 1000 {
            last_report = now;
            let s0 = hi.read_register(mms::STANDARD, addr::STATUS0);
            info!(
                "alive: IRQ_N={} irq_loops={} configured={} STATUS0={}",
                level(ctl.irq_asserted()),
                irq_count,
                hi.is_configured(),
                Debug2Format(&s0),
            );
        }
    }
}

fn level(low: bool) -> &'static str {
    if low { "low (asserted)" } else { "high" }
}
