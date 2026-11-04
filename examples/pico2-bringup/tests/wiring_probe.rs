//! Wiring check that needs no SPI: every line of both chips is read as a plain input, once with
//! the Pico's pull-down and once with its pull-up, printed as down/up. 0/1 just follows the pull
//! (an undriven chip input, or an open wire); 0/0 or 1/1 means something holds the line. Chip A
//! is the known-good baseline. Run with `cargo test --test wiring_probe`, then read the log.

#![no_std]
#![no_main]

use defmt_rtt as _;

#[embedded_test::setup]
fn setup() {
    // SAFETY: first code after reset, single core.
    unsafe { pico2_bringup::early_init() };
}

#[cfg(test)]
#[embedded_test::tests]
mod tests {
    use defmt::info;
    use embedded_hal::digital::OutputPin;
    use rp235x_hal::{self as hal, pac};

    const SETTLE: u32 = 20_000; // CPU cycles; ~1.5 ms on the boot clock

    /// (level with pull-down, level with pull-up). Pull-down goes first, from a driven-low start:
    /// RP2350 erratum E9 latches a pad at ~2.2 V once it has been high, which the weak pull-down
    /// can't undo, so the reverse order reads 1 on undriven lines.
    fn probe(n: usize) -> (u32, u32) {
        // SAFETY: single-threaded test; only this pin's pad and SIO bits are touched.
        let pads = unsafe { &*pac::PADS_BANK0::ptr() };
        let sio = unsafe { &*pac::SIO::ptr() };
        let bit = 1u32 << n;

        pads.gpio(n)
            .modify(|_, w| w.pue().clear_bit().pde().set_bit());
        sio.gpio_out_clr().write(|w| unsafe { w.bits(bit) });
        sio.gpio_oe_set().write(|w| unsafe { w.bits(bit) });
        cortex_m::asm::delay(200);
        sio.gpio_oe_clr().write(|w| unsafe { w.bits(bit) });
        cortex_m::asm::delay(SETTLE);
        let down = (sio.gpio_in().read().bits() >> n) & 1;

        pads.gpio(n)
            .modify(|_, w| w.pue().set_bit().pde().clear_bit());
        cortex_m::asm::delay(SETTLE);
        let up = (sio.gpio_in().read().bits() >> n) & 1;

        pads.gpio(n)
            .modify(|_, w| w.pue().clear_bit().pde().clear_bit());
        (down, up)
    }

    /// Drive `n` high: keeps CS_N deasserted while the other lines are probed.
    fn hold_high(n: usize) {
        // SAFETY: as in `probe`.
        let sio = unsafe { &*pac::SIO::ptr() };
        sio.gpio_out_set().write(|w| unsafe { w.bits(1 << n) });
        sio.gpio_oe_set().write(|w| unsafe { w.bits(1 << n) });
    }

    fn dump(phase: &str) {
        info!("--- {=str}: line  A(gpio down/up)  B(gpio down/up)", phase);
        hold_high(13);
        hold_high(5);
        for (name, a, b) in [
            ("SCLK", 10, 2),
            ("SDI", 11, 3),
            ("SDO", 12, 4),
            ("IRQ_N", 14, 6),
            ("CS_N", 13, 5),
        ] {
            let (ad, au) = probe(a);
            let (bd, bu) = probe(b);
            info!(
                "{=str}: A gpio{=usize} {=u32}/{=u32}   B gpio{=usize} {=u32}/{=u32}",
                name, a, ad, au, b, bd, bu
            );
        }
    }

    #[test]
    #[timeout(10)]
    fn dump_line_levels() {
        let mut p = pac::Peripherals::take().unwrap();
        let sio = hal::Sio::new(p.SIO);
        let pins = hal::gpio::Pins::new(p.IO_BANK0, p.PADS_BANK0, sio.gpio_bank0, &mut p.RESETS);

        let _ = pins.gpio10.into_floating_input();
        let _ = pins.gpio11.into_floating_input();
        let _ = pins.gpio12.into_floating_input();
        let _ = pins.gpio13.into_floating_input();
        let _ = pins.gpio14.into_floating_input();
        let _ = pins.gpio2.into_floating_input();
        let _ = pins.gpio3.into_floating_input();
        let _ = pins.gpio4.into_floating_input();
        let _ = pins.gpio5.into_floating_input();
        let _ = pins.gpio6.into_floating_input();

        let reset_a = pins.gpio15.into_floating_input();
        let reset_b = pins.gpio7.into_floating_input();
        let (ad, au) = probe(15);
        let (bd, bu) = probe(7);
        info!(
            "RESET_N undriven: A gpio15 {=u32}/{=u32}   B gpio7 {=u32}/{=u32}",
            ad, au, bd, bu
        );

        let mut reset_a = reset_a.into_push_pull_output();
        let mut reset_b = reset_b.into_push_pull_output();

        reset_a.set_low().unwrap();
        reset_b.set_low().unwrap();
        cortex_m::asm::delay(200_000);
        dump("RESET_N low");

        reset_a.set_high().unwrap();
        reset_b.set_high().unwrap();
        cortex_m::asm::delay(600_000);
        dump("RESET_N high");
    }

    fn pull_up(n: usize) {
        // SAFETY: as in `probe`.
        let pads = unsafe { &*pac::PADS_BANK0::ptr() };
        let sio = unsafe { &*pac::SIO::ptr() };
        sio.gpio_oe_clr().write(|w| unsafe { w.bits(1 << n) });
        pads.gpio(n)
            .modify(|_, w| w.pue().set_bit().pde().clear_bit());
    }

    fn drive_low(n: usize) {
        // SAFETY: as in `probe`.
        let sio = unsafe { &*pac::SIO::ptr() };
        sio.gpio_out_clr().write(|w| unsafe { w.bits(1 << n) });
        sio.gpio_oe_set().write(|w| unsafe { w.bits(1 << n) });
    }

    fn input_level(n: usize) -> u32 {
        // SAFETY: as in `probe`.
        let sio = unsafe { &*pac::SIO::ptr() };
        (sio.gpio_in().read().bits() >> n) & 1
    }

    /// Pulse each line low in turn as if it were RESET_N, with every line on a pull-up. A line
    /// reading 0 afterwards is being pulled low by the chip (IRQ_N after reset).
    fn scan_reset(name: &str, lines: [usize; 6]) {
        for r in lines {
            for n in lines {
                pull_up(n);
            }
            cortex_m::asm::delay(SETTLE);
            drive_low(r);
            cortex_m::asm::delay(50_000);
            pull_up(r);
            cortex_m::asm::delay(600_000);
            let mut low = 0u32;
            for n in lines {
                if input_level(n) == 0 {
                    low |= 1 << n;
                }
            }
            info!(
                "{=str}: pulsed gpio{=usize} as RESET_N -> lines held low {=u32:#b}",
                name, r, low
            );
        }
    }

    /// With the real RESET_N line high, assert each line as CS_N and report which others stop
    /// following the pulls, i.e. are driven by the chip (SDO while selected).
    fn scan_cs(name: &str, lines: [usize; 6], reset: usize) {
        for c in lines {
            if c == reset {
                continue;
            }
            for n in lines {
                pull_up(n);
            }
            cortex_m::asm::delay(SETTLE);
            drive_low(c);
            cortex_m::asm::delay(SETTLE);
            let mut driven = 0u32;
            for n in lines {
                if n != c && n != reset {
                    let (d, u) = probe(n);
                    pull_up(n);
                    if d == u {
                        driven |= 1 << n;
                    }
                }
            }
            pull_up(c);
            info!(
                "{=str}: gpio{=usize} low as CS_N -> driven lines {=u32:#b}",
                name, c, driven
            );
        }
    }

    #[test]
    #[timeout(20)]
    fn scan_for_swapped_lines() {
        let mut p = pac::Peripherals::take().unwrap();
        let sio = hal::Sio::new(p.SIO);
        let pins = hal::gpio::Pins::new(p.IO_BANK0, p.PADS_BANK0, sio.gpio_bank0, &mut p.RESETS);
        let _ = pins.gpio2.into_floating_input();
        let _ = pins.gpio3.into_floating_input();
        let _ = pins.gpio4.into_floating_input();
        let _ = pins.gpio5.into_floating_input();
        let _ = pins.gpio6.into_floating_input();
        let _ = pins.gpio7.into_floating_input();
        let _ = pins.gpio10.into_floating_input();
        let _ = pins.gpio11.into_floating_input();
        let _ = pins.gpio12.into_floating_input();
        let _ = pins.gpio13.into_floating_input();
        let _ = pins.gpio14.into_floating_input();
        let _ = pins.gpio15.into_floating_input();

        const A: [usize; 6] = [10, 11, 12, 13, 14, 15];
        const B: [usize; 6] = [2, 3, 4, 5, 6, 7];
        scan_reset("A", A);
        scan_reset("B", B);
        for n in A.into_iter().chain(B) {
            pull_up(n);
        }
        cortex_m::asm::delay(600_000);
        scan_cs("A", A, 15);
        scan_cs("B", B, 7);
    }
}
