use crate::utils::Bit;

#[derive(Debug, Clone, Copy)]
pub struct ControlHeader {
    // Data-Not-Control flag (bit 31)
    // Specifies the type of SPI transaction. For control commands, this bit shall be '0'.
    // 0 = Control command
    // 1 = Data chunk
    pub dnc: bool,

    // Received Header Bad (bit 30)
    // When set by the MAC-PHY, indicates that a header was received with a parity error.
    // The SPI host should always clear this bit. The MAC-PHY ignores the HDRB value sent
    // by the SPI host on MOSI.
    pub hdrb: bool,

    // Write-Not-Read (bit 29)
    // Indicates if data is to be written to registers (when set) or read from registers (when clear).
    // 0 = Read from registers
    // 1 = Write to registers
    pub wnr: bool,

    // Address Increment Disable (bit 28)
    // When clear, the address will be automatically post-incremented by one following each register
    // read or write. When set, address auto increment is disabled allowing successive reads and writes
    // to occur at the same register address. If this feature is not supported, the MAC-PHY ignores
    // this field and the SPI host shall always clear it.
    pub aid: bool,

    // Memory Map Selector (bits 27-24)
    // Selects the specific register memory map to access. Allows accessing different memory regions.
    pub mms: u8,

    // Address (bits 23-8)
    // Address of the first register within the selected memory map to access.
    pub addr: u16,

    // Length (bits 7-1)
    // Specifies the number of registers to read/write. Interpreted as the number of registers minus 1,
    // allowing for up to 128 consecutive registers read or written starting at the address specified in
    // ADDR. A length of zero shall read or write a single register.
    pub len: u8,

    // Parity bit (bit 0)
    // Parity bit calculated over the control command header using odd parity.
    pub parity: bool,
}

impl Default for ControlHeader {
    fn default() -> Self {
        Self::new()
    }
}

impl ControlHeader {
    pub fn new() -> Self {
        Self {
            dnc: false,
            hdrb: false,
            wnr: false,
            aid: false,
            mms: 0,
            addr: 0,
            len: 0,
            parity: false,
        }
    }

    pub fn valid(&self) -> bool {
        // For control commands, DNC should be 0 (though this struct assumes control context)
        if self.dnc {
            return false;
        }

        // MMS should fit in 4 bits
        if self.mms > 0b1111 {
            return false;
        }

        // LEN should fit in 7 bits
        if self.len > 0b111_1111 {
            return false;
        }

        // make sure its odd parity
        if self.parity != self.calc_parity_bit() {
            return false;
        }

        true
    }

    fn calc_parity_bit(&self) -> bool {
        self.dnc as u32
            + self.hdrb as u32
            + self.wnr as u32
            + self.aid as u32
            + self.mms.count_ones()
            + ((self.addr >> 8) as u8).count_ones()
            + (self.addr as u8).count_ones()
            + self.len.count_ones()
            % 2
            == 0
    }

    // will create a ControlHeader from a slice, if the slice is not 4 bytes it returns none
    pub fn from_slice(bytes: &[u8]) -> Option<Self> {
        use crate::utils::Bit;

        if bytes.len() != 4 {
            return None;
        }
        let ary = [bytes[0], bytes[1], bytes[2], bytes[3]];
        let mut header = ControlHeader::new();
        let header_u32 = u32::from_be_bytes(ary);
        header.dnc = header_u32.get_bit(31);
        header.hdrb = header_u32.get_bit(30);
        header.wnr = header_u32.get_bit(29);
        header.aid = header_u32.get_bit(28);
        header.mms = ((header_u32 >> 24) & 0b1111) as u8;
        header.addr = ((header_u32 >> 8) & 0xffff) as u16;
        header.len = ((header_u32 >> 1) & 0b111_1111) as u8;
        header.parity = header_u32.get_bit(0);
        Some(header)
    }

    pub fn to_bytes(&self) -> [u8; 4] {
        let mut header_u32 = 0u32;
        header_u32.set_bit(31, self.dnc);
        header_u32.set_bit(30, self.hdrb);
        header_u32.set_bit(29, self.wnr);
        header_u32.set_bit(28, self.aid);
        header_u32 |= (self.mms as u32 & 0b1111) << 24;
        header_u32 |= (self.addr as u32 & 0xffff) << 8;
        header_u32 |= (self.len as u32 & 0b111_1111) << 1;
        header_u32.set_bit(0, self.parity);
        header_u32.to_be_bytes()
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn parity_test() {
        let mut header = ControlHeader::default();
        // bit should be true, so there is an odd number of bits
        assert!(header.calc_parity_bit());

        header.wnr = true;
        // bit should be false so there is an odd number, since wnr is set
        assert!(!header.calc_parity_bit());
        header.mms = 0b11;

        // added two bits should be same
        assert!(!header.calc_parity_bit());
    }

    #[test]
    fn valid_test() {
        let mut header = ControlHeader::default();
        // a header with all zeros is not valid due to parity
        assert!(!header.valid());

        header.parity = true;
        assert!(header.valid());

        header.wnr = true;
        // header not valid since parity needs to change with wnr set
        assert!(!header.valid());

        header.parity = false;
        assert!(header.valid());

        // Test with address and length values
        header.addr = 0x1234;
        header.len = 0x7f; // max 7 bits
        header.parity = false;
        assert!(header.valid());

        // Test invalid field sizes
        header.mms = 0b1_0000; // too large for 4 bits
        header.parity = false;
        assert!(!header.valid());

        header.mms = 0b1111; // reset to valid
        header.len = 0b1_0000_000; // too large for 7 bits
        header.parity = false;
        assert!(!header.valid());
    }

    #[test]
    fn serialization_test() {
        let mut header = ControlHeader::default();
        header.wnr = true;
        header.addr = 0x1234;
        header.len = 0x42;
        header.parity = true;

        let bytes = header.to_bytes();
        let parsed = ControlHeader::from_slice(&bytes).unwrap();

        assert_eq!(parsed.wnr, header.wnr);
        assert_eq!(parsed.addr, header.addr);
        assert_eq!(parsed.len, header.len);
        assert_eq!(parsed.parity, header.parity);
    }
}
