use crate::utils::Bit;

#[derive(Debug, Clone, Copy)]
pub struct TransmitHeader {
    // Data Not Control flag (bit 31)
    // Specifies the type of SPI transaction. For TX data chunks, this bit shall be '1'.
    // 0 = Control command
    // 1 = Data chunk
    pub dnc: bool,

    // Data Chunk Sequence bit (bit 30)
    // When SEQE is enabled in CONFIG0 register, this acts as an even/odd bit (LSB of transmit data chunk counter).
    // The SPI host shall toggle this bit for each new transmit data chunk. When the MAC-PHY receives a chunk
    // with SEQ unchanged from the previous chunk, it assumes a resend and does not accept the current chunk.
    // When disabled or unsupported, the MAC-PHY ignores this bit and the host should write SEQ = 0.
    pub seq: bool,

    // No Receive flag (bit 29)
    // Used by the SPI host for flow control to the MAC-PHY. When NORX = 1, the host will ignore the current
    // receive data chunk payload, and the MAC-PHY will set DV = 0 in the footer of the current receive data chunk.
    // Any available receive frame data is retained and the MAC-PHY will attempt to resend it on the next data chunk.
    // When NORX = 0, the host indicates it will receive and process the current receive data chunk payload.
    pub norx: bool,

    // Data Valid flag (bit 21)
    // Indicates to the MAC-PHY whether the current transmit data chunk payload contains valid transmit
    // Ethernet frame data. When DV = 1, the SV and EV flags should be set accordingly to locate frame
    // data boundaries. It is possible that DV = 1 with both SV = 0 and EV = 0 during long frame transfers,
    // or SV = 1 and EV = 1 for a complete frame within a single chunk. When DV = 0, the MAC-PHY ignores
    // the transmit data chunk payload.
    pub dv: bool,

    // Start Valid flag (bit 20)
    // Indicates whether the transmit data chunk payload contains a valid beginning of an Ethernet frame.
    // When SV = 1, the SWO field shall be set to locate the beginning of the frame. When SV = 0, the
    // SPI host shall write SWO and EBO as all zero.
    pub sv: bool,

    // Start Word Offset (bits 19-16)
    // When SV = 1, this field is set to the offset (expressed in 32-bit words) of the first data byte
    // within the payload of the chunk. The first byte of the Ethernet frame is always the most significant
    // byte of the 32-bit word, ensuring alignment to a 32-bit boundary. Range: 0 to 15. When SV = 0,
    // the host shall write this field as zero.
    pub swo: u8,

    // End Valid flag (bit 14)
    // Indicates whether the transmit data chunk payload contains the end of an Ethernet frame.
    // When EV = 1, the EBO field shall be set accordingly to point to the position of the last byte
    // of the frame within the transmit data chunk payload.
    pub ev: bool,

    // End Byte Offset (bits 13-8)
    // When EV = 1, this field is set to the offset of the last byte of the Ethernet frame within the
    // chunk payload. The first byte of the transmit data chunk payload is located at an offset of zero.
    // When EV = 0, the host shall write this field as zero.
    pub ebo: u8,

    // Timestamp Capture (bits 7-6)
    // Used by the SPI host to request the capture of a timestamp when the frame is transmitted onto the network.
    // This field is only valid when SV = 1 and shall be ignored when SV = 0 or the timestamp feature is not
    // supported. The host shall set TSC = 00 at all other times.
    // 00 = Do not capture a timestamp
    // 01 = Capture timestamp into timestamp capture register A
    // 10 = Capture timestamp into timestamp capture register B
    // 11 = Capture timestamp into timestamp capture register C
    pub tsc: u8,

    // Parity bit (bit 0)
    // Odd parity bit for transmit data header protection. Provides error detection as described in Section 8.5.2.
    // When a header is received with a parity error, the MAC-PHY handles it as described in Section 7.5.
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
        header_u32 |= (self.swo as u32 & 0b1111) << 16;
        header_u32.set_bit(14, self.ev);
        header_u32 |= (self.ebo as u32 & 0b111111) << 8;
        header_u32 |= (self.tsc as u32 & 0b11) << 6;
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
