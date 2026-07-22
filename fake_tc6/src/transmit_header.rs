use crate::utils::Bit;

pub struct TransmitHeader {
    // Data Not Control 31
    pub dnc: bool,

    // Data chunk sequence 30
    pub seq: bool,

    // no receive 29
    pub norx: bool,

    // data valid
    pub dv: bool,

    // start valid 21
    pub sv: bool,

    // Start word offset, 28..24
    pub swo: u8,

    // End valid
    pub ev: bool,

    // end byte offset
    pub ebo: u8,

    // transmit frame timestamp capture 7..6
    pub tsc: u8,

    pub parity: bool,
}

impl Default for TransmitHeader {
    fn default() -> Self {
        Self::new()
    }
}

impl TransmitHeader {
    pub fn new() -> Self {
        Self {
            dnc: false,
            seq: false,
            norx: false,
            dv: false,
            sv: false,
            swo: 0,
            ev: false,
            ebo: 0,
            tsc: 0,
            parity: false,
        }
    }

    pub fn valid(&self) -> bool {
        if !self.dv {
            // if data is not valid then all bits describing the contained data should be 0
            if self.sv {
                return false;
            }
            if self.swo != 0 {
                return false;
            }
            if self.ev {
                return false;
            }
            if self.ebo != 0 {
                return false;
            }
        } else {
            // if there is valid data

            // if sv is false swo should be zero
            if !self.sv && self.swo != 0 {
                return false;
            }

            // if end is not valid ebo should be zero
            if !self.ev && self.ebo != 0 {
                return false;
            }

            // if SWO is larger than 4 bits
            // conversion to its 32 bit "actual" representation would fail
            if self.swo > 0b1111 {
                return false;
            }
            // if EBO is larger than 6 bits
            // conversion to its 32 bit "actual" representation would fail
            if self.ebo > 0b11_1111 {
                return false;
            }
        }

        // make sure its odd parity
        if self.parity != self.calc_parity_bit() {
            return false;
        }

        true
    }

    fn calc_parity_bit(&self) -> bool {
        self.dnc as u32
            + self.seq as u32
            + self.norx as u32
            + self.dv as u32
            + self.sv as u32
            + self.swo.count_ones()
            + self.ev as u32
            + self.ebo.count_ones()
            + self.tsc.count_ones() % 2
            == 0
    }

    // will create a Transmit header from a slice, if the slice is not 4 bytes it returns none
    pub fn from_slice(bytes: &[u8]) -> Option<Self> {
        use crate::utils::Bit;

        if bytes.len() != 4 {
            return None;
        }
        let ary = [bytes[0], bytes[1], bytes[2], bytes[3]];
        let mut header = TransmitHeader::new();
        let header_u32 = u32::from_be_bytes(ary);
        header.dnc = header_u32.get_bit(31);
        header.seq = header_u32.get_bit(30);
        header.norx = header_u32.get_bit(29);
        header.dv = header_u32.get_bit(21);
        header.sv = header_u32.get_bit(20);
        header.swo = ((header_u32 >> 16) & 0b1111) as u8;
        header.ev = header_u32.get_bit(14);
        header.ebo = ((header_u32 >> 8) & 0b111111) as u8;
        header.tsc = ((header_u32 >> 6) & 0b11) as u8;
        header.parity = header_u32.get_bit(0);
        Some(header)
    }

    pub fn to_bytes(&self) -> [u8; 4] {
        let mut header_u32 = 0u32;
        header_u32.set_bit(31, self.dnc);
        header_u32.set_bit(30, self.seq);
        header_u32.set_bit(29, self.norx);
        header_u32.set_bit(21, self.dv);
        header_u32.set_bit(20, self.sv);
        header_u32 |= self.swo as u32 & 0b1111 << 16;
        header_u32.set_bit(14, self.ev);
        header_u32 |= self.ebo as u32 & 0b111111 << 8;
        header_u32 |= self.tsc as u32 & 0b11 << 6;
        header_u32.set_bit(0, self.parity);
        header_u32.to_be_bytes()
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn parity_test() {
        let mut header = TransmitHeader::default();
        // bit should be true, so there is a odd number of bits
        assert!(header.calc_parity_bit());

        header.dnc = true;
        // bit should be false so there is an odd number, since dnc is set
        assert!(!header.calc_parity_bit());
        header.swo = 0b11;

        // added two bits should be same
        assert!(!header.calc_parity_bit());
    }

    #[test]
    fn valid_test() {
        let mut header = TransmitHeader::default();
        // a header with all zeros is not valid due to parity
        assert!(!header.valid());

        header.parity = true;
        assert!(header.valid());

        header.swo = 0b11;
        // header not valid since swo is something and SV is not set, party is the same since we added 2 bits
        assert!(!header.valid());

        header.swo = 0b111;
        header.sv = true;
        // not valid since dw is not set
        assert!(!header.valid());

        header.dv = true;
        header.parity = false;
        assert!(header.valid());

        header.swo = 0;
        header.ev = true;
        header.ebo = 0b11;

        // setting an end byte offset to after swo
        assert!(header.valid())
    }
}
