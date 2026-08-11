use crate::bitfield::{get_bit, get_bits, set_bit, set_bits};

/// Receive data chunk footer (sent MAC-PHY -> host on MISO), see TC6 spec Section 7.3.7.
///
/// Stored as a single `u32` matching the wire format exactly, rather than one field per bit-range
/// like the fake_tc6 reference, since we're targeting embedded and don't want the extra size/copy
/// cost of a many-field struct.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ReceiveFooter(u32);

impl ReceiveFooter {
    pub const fn new() -> Self {
        Self(0)
    }

    pub const fn from_raw(raw: u32) -> Self {
        Self(raw)
    }

    pub const fn raw(self) -> u32 {
        self.0
    }

    pub fn from_bytes(bytes: [u8; 4]) -> Self {
        Self(u32::from_be_bytes(bytes))
    }

    pub fn to_bytes(self) -> [u8; 4] {
        self.0.to_be_bytes()
    }

    // Extended Status (bit 31): a STATUS0/STATUS1 bit is set and not masked.
    pub const fn exst(self) -> bool {
        get_bit(self.0, 31)
    }
    pub const fn with_exst(self, val: bool) -> Self {
        Self(set_bit(self.0, 31, val))
    }

    // Received Header Bad (bit 30): the last header received had a parity error.
    pub const fn hdrb(self) -> bool {
        get_bit(self.0, 30)
    }
    pub const fn with_hdrb(self, val: bool) -> Self {
        Self(set_bit(self.0, 30, val))
    }

    // Configuration Synchronized flag (bit 29), mirrors CONFIG0.SYNC.
    pub const fn sync(self) -> bool {
        get_bit(self.0, 29)
    }
    pub const fn with_sync(self, val: bool) -> Self {
        Self(set_bit(self.0, 29, val))
    }

    // Receive Chunks Available (bits 28-24).
    pub const fn rca(self) -> u8 {
        get_bits(self.0, 24, 0b1_1111) as u8
    }
    pub const fn with_rca(self, val: u8) -> Self {
        Self(set_bits(self.0, 24, 0b1_1111, val as u32))
    }

    // Data Valid flag (bit 21): payload contains valid receive frame data.
    pub const fn dv(self) -> bool {
        get_bit(self.0, 21)
    }
    pub const fn with_dv(self, val: bool) -> Self {
        Self(set_bit(self.0, 21, val))
    }

    // Start Valid flag (bit 20): payload contains the start of a frame.
    pub const fn sv(self) -> bool {
        get_bit(self.0, 20)
    }
    pub const fn with_sv(self, val: bool) -> Self {
        Self(set_bit(self.0, 20, val))
    }

    // Start Word Offset (bits 19-16): 32-bit word offset of the first frame byte.
    pub const fn swo(self) -> u8 {
        get_bits(self.0, 16, 0b1111) as u8
    }
    pub const fn with_swo(self, val: u8) -> Self {
        Self(set_bits(self.0, 16, 0b1111, val as u32))
    }

    // Frame Drop (bit 15): only valid when DV = 1 and EV = 1.
    pub const fn fd(self) -> bool {
        get_bit(self.0, 15)
    }
    pub const fn with_fd(self, val: bool) -> Self {
        Self(set_bit(self.0, 15, val))
    }

    // End Valid flag (bit 14): payload contains the end of a frame.
    pub const fn ev(self) -> bool {
        get_bit(self.0, 14)
    }
    pub const fn with_ev(self, val: bool) -> Self {
        Self(set_bit(self.0, 14, val))
    }

    // End Byte Offset (bits 13-8): byte offset of the last frame byte within the payload.
    pub const fn ebo(self) -> u8 {
        get_bits(self.0, 8, 0b11_1111) as u8
    }
    pub const fn with_ebo(self, val: u8) -> Self {
        Self(set_bits(self.0, 8, 0b11_1111, val as u32))
    }

    // Receive Timestamp Added (bit 7), zero when SV = 0.
    pub const fn rtsa(self) -> bool {
        get_bit(self.0, 7)
    }
    pub const fn with_rtsa(self, val: bool) -> Self {
        Self(set_bit(self.0, 7, val))
    }

    // Receive Timestamp Parity (bit 6), zero when RTSA = 0.
    pub const fn rtsp(self) -> bool {
        get_bit(self.0, 6)
    }
    pub const fn with_rtsp(self, val: bool) -> Self {
        Self(set_bit(self.0, 6, val))
    }

    // Transmit Credits (bits 5-1): free tx buffer space, in chunks.
    pub const fn txc(self) -> u8 {
        get_bits(self.0, 1, 0b1_1111) as u8
    }
    pub const fn with_txc(self, val: u8) -> Self {
        Self(set_bits(self.0, 1, 0b1_1111, val as u32))
    }

    // Parity bit (bit 0), odd parity over the rest of the footer.
    pub const fn parity(self) -> bool {
        get_bit(self.0, 0)
    }
    pub const fn with_parity(self, val: bool) -> Self {
        Self(set_bit(self.0, 0, val))
    }

    /// What the parity bit needs to be for the footer (minus the parity bit itself) to have odd parity.
    pub const fn calc_parity(self) -> bool {
        (self.0 & !1).count_ones() % 2 == 0
    }

    /// Set the parity bit to the correct value for the rest of the footer's current contents.
    pub const fn with_correct_parity(self) -> Self {
        let p = self.calc_parity();
        Self(set_bit(self.0, 0, p))
    }

    pub const fn parity_ok(self) -> bool {
        self.parity() == self.calc_parity()
    }

    pub const fn valid(self) -> bool {
        if !self.dv() {
            // if data is not valid then all bits describing the contained data should be 0
            if self.sv() || self.swo() != 0 || self.ev() || self.ebo() != 0 {
                return false;
            }
        } else {
            // if sv is false swo should be zero
            if !self.sv() && self.swo() != 0 {
                return false;
            }
            // if end is not valid ebo should be zero
            if !self.ev() && self.ebo() != 0 {
                return false;
            }
        }

        // FD is only valid when EV = 1 and DV = 1
        if self.fd() && (!self.ev() || !self.dv()) {
            return false;
        }

        // RTSA should be 0 when SV = 0
        if !self.sv() && self.rtsa() {
            return false;
        }

        // RTSP should be 0 when RTSA = 0
        if !self.rtsa() && self.rtsp() {
            return false;
        }

        self.parity_ok()
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn parity_test() {
        let footer = ReceiveFooter::new();
        // all zero footer needs parity = true for odd parity
        assert!(footer.calc_parity());

        let footer = footer.with_exst(true);
        assert!(!footer.calc_parity());

        let footer = footer.with_rca(0b11);
        // two more bits set, parity requirement unchanged
        assert!(!footer.calc_parity());
    }

    #[test]
    fn valid_test() {
        let footer = ReceiveFooter::new();
        // a footer with all zeros is not valid due to parity
        assert!(!footer.valid());

        let footer = footer.with_correct_parity();
        assert!(footer.valid());

        let footer = footer.with_swo(0b11).with_correct_parity();
        // footer not valid since swo is something and SV is not set
        assert!(!footer.valid());

        let footer = footer.with_sv(true).with_correct_parity();
        // not valid since dv is not set
        assert!(!footer.valid());

        let footer = footer.with_dv(true).with_correct_parity();
        assert!(footer.valid());

        let footer = footer
            .with_swo(0)
            .with_ev(true)
            .with_ebo(0b11)
            .with_correct_parity();
        // setting an end byte offset after swo is fine, no ordering constraint here
        assert!(footer.valid());
    }

    #[test]
    fn roundtrip_bytes() {
        let footer = ReceiveFooter::new()
            .with_dv(true)
            .with_ev(true)
            .with_ebo(0b10101)
            .with_txc(0b1101)
            .with_correct_parity();

        let bytes = footer.to_bytes();
        let restored = ReceiveFooter::from_bytes(bytes);
        assert_eq!(footer, restored);
    }
}
