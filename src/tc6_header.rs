use crate::bitfield::{get_bit, get_bits, set_bit, set_bits};

/// Transmit data chunk header (sent host -> MAC-PHY on MOSI), see TC6 spec Section 7.3.6.
///
/// Stored as a single `u32` matching the wire format exactly, rather than one field per bit-range
/// like the fake_tc6 reference, since we're targeting embedded and don't want the extra size/copy
/// cost of a many-field struct.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct TransmitHeader(u32);

impl TransmitHeader {
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

    // Data-Not-Control flag (bit 31): 0 = control command, 1 = data chunk.
    pub const fn dnc(self) -> bool {
        get_bit(self.0, 31)
    }
    pub const fn with_dnc(self, val: bool) -> Self {
        Self(set_bit(self.0, 31, val))
    }

    // Data Chunk Sequence bit (bit 30), toggled by the host per chunk when SEQE is enabled.
    pub const fn seq(self) -> bool {
        get_bit(self.0, 30)
    }
    pub const fn with_seq(self, val: bool) -> Self {
        Self(set_bit(self.0, 30, val))
    }

    // No Receive flag (bit 29): host asks the MAC-PHY to ignore/retain the current rx chunk.
    pub const fn norx(self) -> bool {
        get_bit(self.0, 29)
    }
    pub const fn with_norx(self, val: bool) -> Self {
        Self(set_bit(self.0, 29, val))
    }

    // Data Valid flag (bit 21): payload contains valid transmit frame data.
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

    // Timestamp Capture (bits 7-6), only meaningful when SV = 1.
    pub const fn tsc(self) -> u8 {
        get_bits(self.0, 6, 0b11) as u8
    }
    pub const fn with_tsc(self, val: u8) -> Self {
        Self(set_bits(self.0, 6, 0b11, val as u32))
    }

    // Parity bit (bit 0), odd parity over the rest of the header.
    pub const fn parity(self) -> bool {
        get_bit(self.0, 0)
    }
    pub const fn with_parity(self, val: bool) -> Self {
        Self(set_bit(self.0, 0, val))
    }

    /// What the parity bit needs to be for the header (minus the parity bit itself) to have odd parity.
    pub const fn calc_parity(self) -> bool {
        (self.0 & !1).count_ones() % 2 == 0
    }

    /// Set the parity bit to the correct value for the rest of the header's current contents.
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

        self.parity_ok()
    }
}

/// Control command header (sent host -> MAC-PHY on MOSI), see TC6 spec Section 7.4.
///
/// Stored as a single `u32`, same rationale as [`TransmitHeader`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct ControlHeader(u32);

impl ControlHeader {
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

    // Data-Not-Control flag (bit 31): shall be 0 for control commands.
    pub const fn dnc(self) -> bool {
        get_bit(self.0, 31)
    }
    pub const fn with_dnc(self, val: bool) -> Self {
        Self(set_bit(self.0, 31, val))
    }

    // Received Header Bad (bit 30): host shall always clear this bit.
    pub const fn hdrb(self) -> bool {
        get_bit(self.0, 30)
    }
    pub const fn with_hdrb(self, val: bool) -> Self {
        Self(set_bit(self.0, 30, val))
    }

    // Write-Not-Read (bit 29): 0 = read, 1 = write.
    pub const fn wnr(self) -> bool {
        get_bit(self.0, 29)
    }
    pub const fn with_wnr(self, val: bool) -> Self {
        Self(set_bit(self.0, 29, val))
    }

    // Address Increment Disable (bit 28).
    pub const fn aid(self) -> bool {
        get_bit(self.0, 28)
    }
    pub const fn with_aid(self, val: bool) -> Self {
        Self(set_bit(self.0, 28, val))
    }

    // Memory Map Selector (bits 27-24).
    pub const fn mms(self) -> u8 {
        get_bits(self.0, 24, 0b1111) as u8
    }
    pub const fn with_mms(self, val: u8) -> Self {
        Self(set_bits(self.0, 24, 0b1111, val as u32))
    }

    // Address (bits 23-8): first register address within the selected memory map.
    pub const fn addr(self) -> u16 {
        get_bits(self.0, 8, 0xffff) as u16
    }
    pub const fn with_addr(self, val: u16) -> Self {
        Self(set_bits(self.0, 8, 0xffff, val as u32))
    }

    // Length (bits 7-1): number of registers to read/write, minus 1.
    pub const fn len(self) -> u8 {
        get_bits(self.0, 1, 0b111_1111) as u8
    }
    pub const fn with_len(self, val: u8) -> Self {
        Self(set_bits(self.0, 1, 0b111_1111, val as u32))
    }

    // Parity bit (bit 0), odd parity over the rest of the header.
    pub const fn parity(self) -> bool {
        get_bit(self.0, 0)
    }
    pub const fn with_parity(self, val: bool) -> Self {
        Self(set_bit(self.0, 0, val))
    }

    /// What the parity bit needs to be for the header (minus the parity bit itself) to have odd parity.
    pub const fn calc_parity(self) -> bool {
        (self.0 & !1).count_ones() % 2 == 0
    }

    /// Set the parity bit to the correct value for the rest of the header's current contents.
    pub const fn with_correct_parity(self) -> Self {
        let p = self.calc_parity();
        Self(set_bit(self.0, 0, p))
    }

    pub const fn parity_ok(self) -> bool {
        self.parity() == self.calc_parity()
    }

    pub const fn valid(self) -> bool {
        // DNC must be 0 for a control header
        if self.dnc() {
            return false;
        }

        self.parity_ok()
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn transmit_header_parity() {
        let header = TransmitHeader::new();
        // all zero header needs parity = true for odd parity
        assert!(header.calc_parity());

        let header = header.with_dnc(true);
        assert!(!header.calc_parity());

        let header = header.with_swo(0b11);
        // two more bits set, parity requirement unchanged
        assert!(!header.calc_parity());
    }

    #[test]
    fn transmit_header_valid() {
        let header = TransmitHeader::new();
        // all zero is invalid, parity bit is wrong
        assert!(!header.valid());

        let header = header.with_correct_parity();
        assert!(header.valid());

        let header = header.with_swo(0b11).with_correct_parity();
        // swo set without sv is invalid
        assert!(!header.valid());

        let header = header.with_sv(true).with_correct_parity();
        // sv set but dv is not
        assert!(!header.valid());

        let header = header.with_dv(true).with_correct_parity();
        assert!(header.valid());
    }

    #[test]
    fn transmit_header_roundtrip_bytes() {
        let header = TransmitHeader::new()
            .with_dv(true)
            .with_sv(true)
            .with_swo(0b101)
            .with_correct_parity();

        let bytes = header.to_bytes();
        let restored = TransmitHeader::from_bytes(bytes);
        assert_eq!(header, restored);
    }

    #[test]
    fn control_header_parity() {
        let header = ControlHeader::new();
        assert!(header.calc_parity());

        let header = header.with_wnr(true);
        assert!(!header.calc_parity());
    }

    #[test]
    fn control_header_valid() {
        let header = ControlHeader::new();
        assert!(!header.valid());

        let header = header.with_correct_parity();
        assert!(header.valid());

        let header = header
            .with_addr(0x1234)
            .with_len(0x7f)
            .with_correct_parity();
        assert!(header.valid());

        let header = header.with_dnc(true).with_correct_parity();
        assert!(!header.valid());
    }
}
