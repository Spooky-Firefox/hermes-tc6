//! Frame transfer between the two LAN8651s over the twisted pair: the first hardware run of the
//! driver's data path (transmit chunking, receive reassembly, footer handling). Wiring: see
//! `src/lib.rs`. Run with `cargo test --test frames`.
//!
//! Each test sends frames through the smoltcp `phy::Device` API on one chip and checks what the
//! other chip's `Device::receive` hands back: same bytes, same order, each frame exactly once.
//! Both drivers are polled in a loop rather than on IRQ_N; whether the receiver raised IRQ_N is
//! logged, as a data point for the interrupt-driven code later.
//!
//! Frames are broadcast with the IEEE local experimental EtherType, and both MACs run
//! promiscuous, so address filtering can't hide a frame.

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
    use defmt::{Debug2Format, assert_eq, info, warn};
    use embedded_hal::digital::{InputPin, OutputPin};
    use pico2_bringup::hermes_tc6::registers::{addr, mms};
    use pico2_bringup::hermes_tc6::tc6_driver::Config;
    use pico2_bringup::hermes_tc6::tc6_interface::Tc6Interface;
    use pico2_bringup::{self as board, Ctl, CtlB, Driver, Hi, HiB, Lo, Timer, lan865x};
    use smoltcp::phy::{Device, RxToken, TxToken};
    use smoltcp::time::Instant;

    /// IEEE 802 local experimental EtherType.
    const ETHERTYPE: u16 = 0x88B5;
    const MAC_A: [u8; 6] = [0x02, 0, 0, 0, 0, 0x0A];
    const MAC_B: [u8; 6] = [0x02, 0, 0, 0, 0, 0x0B];
    /// Minimum Ethernet frame without FCS (the MAC appends it).
    const MIN_LEN: usize = 60;
    /// Maximum Ethernet frame without FCS.
    const MAX_LEN: usize = 1514;
    /// How long each test keeps polling. Long enough to see a duplicate if a frame is re-sent.
    const RUN_MS: u64 = 200;

    struct Link {
        hi_a: Hi,
        lo_a: Lo,
        ctl_a: Ctl,
        hi_b: HiB,
        lo_b: Lo,
        ctl_b: CtlB,
    }

    #[init]
    fn init() -> Link {
        let mut b = board::init_both();
        b.ctl_a.hw_reset();
        b.ctl_b.hw_reset();
        let (mut hi_a, lo_a) = board::take_driver(b.a);
        let (mut hi_b, lo_b) = board::take_driver_b(b.b);
        bring_up(&mut hi_a, "A");
        bring_up(&mut hi_b, "B");
        Link {
            hi_a,
            lo_a,
            ctl_a: b.ctl_a,
            hi_b,
            lo_b,
            ctl_b: b.ctl_b,
        }
    }

    /// `configure()`, then enable the MAC (vendor registers, not done by the driver yet), then
    /// clear the post-reset IRQ.
    fn bring_up<T: Tc6Interface<Error: core::fmt::Debug>>(hi: &mut Driver<T>, name: &str) {
        if let Err(e) = hi.configure(&Config::default()) {
            defmt::panic!("{=str}: configure() failed: {}", name, Debug2Format(&e));
        }
        let (ncr, ncfg) = board::enable_mac(hi, true).unwrap();
        let ncr_now = hi.read_register(mms::MAC, lan865x::MAC_NCR).unwrap();
        let ncfg_now = hi.read_register(mms::MAC, lan865x::MAC_NCFG).unwrap();
        info!(
            "{=str}: MAC_NCR {=u32:#x} -> {=u32:#x}, MAC_NCFG {=u32:#x} -> {=u32:#x}",
            name, ncr, ncr_now, ncfg, ncfg_now
        );
        let both = lan865x::NCR_TXEN | lan865x::NCR_RXEN;
        assert_eq!(ncr_now & both, both, "MAC TX/RX enable did not stick");
        hi.on_interrupt().unwrap();
    }

    /// Fill `buf` with a broadcast test frame from `src`; the payload depends on `seq`.
    fn build(buf: &mut [u8], src: [u8; 6], seq: u8) {
        buf[0..6].fill(0xFF);
        buf[6..12].copy_from_slice(&src);
        buf[12..14].copy_from_slice(&ETHERTYPE.to_be_bytes());
        for (i, b) in buf[14..].iter_mut().enumerate() {
            *b = (i as u8).wrapping_mul(7).wrapping_add(seq);
        }
    }

    fn send(lo: &mut Lo, len: usize, src: [u8; 6], seq: u8) {
        let token = lo
            .transmit(Instant::ZERO)
            .expect("transmit() returned None");
        token.consume(len, |buf| build(buf, src, seq));
    }

    /// Does `got` start with the frame `build(len, src, seq)` would make? The receiving MAC passes
    /// the 4-byte FCS up (seen on silicon 2026-11-11), and a frame under 60 bytes may also be
    /// padded to 60 by the sender, so trailing bytes up to `max(len, 60) + 4` are allowed.
    fn matches(got: &[u8], len: usize, src: [u8; 6], seq: u8) -> bool {
        let mut want = [0u8; MAX_LEN];
        build(&mut want[..len], src, seq);
        if got.len() < len || got.len() > len.max(MIN_LEN) + 4 {
            return false;
        }
        match got[..len]
            .iter()
            .zip(&want[..len])
            .position(|(g, w)| g != w)
        {
            None => true,
            Some(i) => {
                warn!(
                    "first mismatch at byte {=usize}: got {=u8:#04x}, want {=u8:#04x}",
                    i, got[i], want[i]
                );
                false
            }
        }
    }

    /// Poll both drivers for `RUN_MS`, handing every frame the receiver delivers to `on_frame`.
    /// Returns whether the receiver's IRQ_N was ever seen asserted.
    fn run<TS, TR, R, I>(
        tx_hi: &mut Driver<TS>,
        tx_pending: fn() -> bool,
        rx_hi: &mut Driver<TR>,
        rx_lo: &mut Lo,
        rx_ctl: &mut Ctl<R, I>,
        timer: Timer,
        mut on_frame: impl FnMut(&[u8]),
    ) -> bool
    where
        TS: Tc6Interface<Error: core::fmt::Debug>,
        TR: Tc6Interface<Error: core::fmt::Debug>,
        R: OutputPin,
        I: InputPin,
    {
        let start = timer.get_counter();
        let mut irq_seen = false;
        while (timer.get_counter() - start).to_millis() < RUN_MS {
            if tx_pending() {
                tx_hi.on_transmit_ready().unwrap();
            }
            tx_hi.on_interrupt().unwrap();
            irq_seen |= rx_ctl.irq_asserted();
            rx_hi.on_interrupt().unwrap();
            while let Some((rx, _tx)) = rx_lo.receive(Instant::ZERO) {
                rx.consume(|f| on_frame(f));
            }
        }
        irq_seen
    }

    fn log_status<T: Tc6Interface<Error: core::fmt::Debug>>(hi: &mut Driver<T>, name: &str) {
        let s0 = hi.read_register(mms::STANDARD, addr::STATUS0).unwrap();
        let buf = hi.read_register(mms::STANDARD, addr::BUFSTS).unwrap();
        info!(
            "{=str}: STATUS0 {=u32:#010x} BUFSTS {=u32:#010x} configured={}",
            name,
            s0,
            buf,
            hi.is_configured()
        );
    }

    /// Send `lens` (one frame each, seq 1, 2, ...) from A to B and check B got each exactly
    /// once, in order.
    fn a_to_b(mut l: Link, lens: &[usize]) {
        for (i, &len) in lens.iter().enumerate() {
            send(&mut l.lo_a, len, MAC_A, i as u8 + 1);
        }
        let mut got = 0usize;
        let mut good = 0usize;
        let timer = l.ctl_b.timer;
        let irq = run(
            &mut l.hi_a,
            board::take_tx_pending,
            &mut l.hi_b,
            &mut l.lo_b,
            &mut l.ctl_b,
            timer,
            |f| {
                info!("B received {=usize} bytes", f.len());
                if let Some(&len) = lens.get(got)
                    && matches(f, len, MAC_A, got as u8 + 1)
                {
                    if f.len() != len {
                        info!(
                            "  ({=usize} trailing bytes: FCS, plus padding if under 60)",
                            f.len() - len
                        );
                    }
                    good += 1;
                }
                got += 1;
            },
        );
        info!("B IRQ_N seen asserted: {}", irq);
        log_status(&mut l.hi_a, "A");
        log_status(&mut l.hi_b, "B");
        assert_eq!(got, lens.len(), "B received the wrong number of frames");
        assert_eq!(
            good,
            lens.len(),
            "B received frames with the wrong contents or order"
        );
    }

    /// [`a_to_b`] in the other direction.
    fn b_to_a(mut l: Link, lens: &[usize]) {
        for (i, &len) in lens.iter().enumerate() {
            send(&mut l.lo_b, len, MAC_B, i as u8 + 1);
        }
        let mut got = 0usize;
        let mut good = 0usize;
        let timer = l.ctl_a.timer;
        let irq = run(
            &mut l.hi_b,
            board::take_tx_pending_b,
            &mut l.hi_a,
            &mut l.lo_a,
            &mut l.ctl_a,
            timer,
            |f| {
                info!("A received {=usize} bytes", f.len());
                if let Some(&len) = lens.get(got)
                    && matches(f, len, MAC_B, got as u8 + 1)
                {
                    if f.len() != len {
                        info!(
                            "  ({=usize} trailing bytes: FCS, plus padding if under 60)",
                            f.len() - len
                        );
                    }
                    good += 1;
                }
                got += 1;
            },
        );
        info!("A IRQ_N seen asserted: {}", irq);
        log_status(&mut l.hi_a, "A");
        log_status(&mut l.hi_b, "B");
        assert_eq!(got, lens.len(), "A received the wrong number of frames");
        assert_eq!(
            good,
            lens.len(),
            "A received frames with the wrong contents or order"
        );
    }

    /// Nothing sent, nothing received: the baseline for the tests below.
    #[test]
    #[timeout(10)]
    fn idle_link_receives_nothing(l: Link) {
        a_to_b(l, &[]);
    }

    /// ARP-request size (42 bytes, below the Ethernet minimum): less than one chunk on the
    /// transmit side. Panicked in the driver before 2026-11-11.
    #[test]
    #[timeout(10)]
    fn arp_size_frame_a_to_b(l: Link) {
        a_to_b(l, &[42]);
    }

    /// Three frames queued before the driver runs (the queue holds `NQ - 1` = 3): each must
    /// arrive once, in order, with no re-sends.
    #[test]
    #[timeout(10)]
    fn three_queued_frames_a_to_b_in_order(l: Link) {
        a_to_b(l, &[MIN_LEN, 300, MAX_LEN]);
    }

    /// Minimum-size frame: one 64-byte chunk on both sides.
    #[test]
    #[timeout(10)]
    fn min_frame_a_to_b(l: Link) {
        a_to_b(l, &[MIN_LEN]);
    }

    #[test]
    #[timeout(10)]
    fn min_frame_b_to_a(l: Link) {
        b_to_a(l, &[MIN_LEN]);
    }

    /// Exactly two chunks, then a partial third: chunk boundaries and EBO on both sides.
    #[test]
    #[timeout(10)]
    fn chunk_boundary_frames_a_to_b(l: Link) {
        a_to_b(l, &[128, 129]);
    }

    /// Full-size frame: 24 chunks, the multi-block transmit and receive paths.
    #[test]
    #[timeout(10)]
    fn max_frame_a_to_b(l: Link) {
        a_to_b(l, &[MAX_LEN]);
    }

    #[test]
    #[timeout(10)]
    fn max_frame_b_to_a(l: Link) {
        b_to_a(l, &[MAX_LEN]);
    }
}
