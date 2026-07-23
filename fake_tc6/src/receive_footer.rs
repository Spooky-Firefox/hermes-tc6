use crate::utils::Bit;

#[derive(Debug, Clone, Copy)]
pub struct ReceiveFooter {
    // Extended Status (bit 31)
    // This bit is set when any bit in the STATUS0 or STATUS1 registers are set and not masked.
    // The SPI host can use this bit to determine and schedule appropriate control commands to read the
    // status registers and maintain proper operation of the MAC-PHY.
    pub exst: bool,

    // Received Header Bad (bit 30)
    // When set, indicates that the MAC-PHY received a control or data header with a parity error.
    // When a header is received from the SPI host with a parity error, the MAC-PHY sets this bit.
    pub hdrb: bool,

    // Configuration Synchronized flag (bit 29)
    // Reflects the state of the SYNC bit in the CONFIG0 configuration register. A zero indicates that the
    // MAC-PHY configuration may not be as expected by the SPI host. Following configuration, the SPI host
    // sets the corresponding bit in the configuration register which is reflected in this field.
    pub sync: bool,

    // Receive Chunks Available (bits 28-24)
    // Indicates to the SPI host the minimum number of additional receive data chunks of frame data that
    // are available for reading beyond the current receive data chunk. This field is zero when there is no
    // receive frame data pending in the MAC-PHY's buffer for reading. When RCA > 0, the SPI host can
    // immediately perform additional data chunk transactions to read the available receive frame data.
    pub rca: u8,

    // Data Valid flag (bit 21)
    // The MAC-PHY uses this bit to indicate whether the current receive data chunk contains valid receive
    // Ethernet frame data (DV = 1) or not (DV = 0). When DV = 1, the SV and EV flags are set accordingly
    // to locate frame data boundaries. It is possible that DV = 1 with both SV = 0 and EV = 0 during long
    // frame transfers, or SV = 1 and EV = 1 for a complete frame within a single chunk. When DV = 0,
    // the SPI host shall ignore the chunk payload.
    pub dv: bool,

    // Start Valid flag (bit 20)
    // The MAC-PHY sets this bit when the current chunk payload contains the start of an Ethernet frame.
    // Otherwise, this bit is zero. When SV = 1, the SWO field is set accordingly to locate the beginning
    // of the frame. When SV = 0, the MAC-PHY writes SWO as all zero and it shall be ignored by the SPI host.
    pub sv: bool,

    // Start Word Offset (bits 19-16)
    // When SV = 1, this field contains the 32-bit word offset into the receive data chunk payload containing
    // the first byte of a new received Ethernet frame. When a receive timestamp has been added to the beginning
    // of the received Ethernet frame (RTSA = 1) then SWO points to the most significant byte of the timestamp.
    // The offset of the first data byte of the frame is always aligned to a 32-bit boundary within the receive
    // data chunk payload such that the first byte of the frame is always the most significant byte of the 32-bit word.
    // When SV = 0, this field will be zero.
    pub swo: u8,

    // Frame Drop (bit 15)
    // When set, this bit indicates that the MAC has detected a condition for which the SPI host should drop
    // the received Ethernet frame. Some MACs implement the ability to transfer bad received frames to the
    // station entity for debugging purposes. This bit is only valid at the end of a received Ethernet frame
    // (EV = 1) with DV = 1, and shall be zero at all other times.
    pub fd: bool,

    // End Valid flag (bit 14)
    // The MAC-PHY sets this bit when the end of a received Ethernet frame is present in this receive data
    // chunk payload. If EV = 1, then EBO points to the position of the last byte of the frame within the
    // receive data chunk payload.
    pub ev: bool,

    // End Byte Offset (bits 13-8)
    // When EV = 1, this field contains the byte offset into the receive data chunk payload that locates the
    // last byte of the received Ethernet frame. The first byte of the receive data chunk payload is located
    // at an offset of zero. When EV = 0, this field is set to zero.
    pub ebo: u8,

    // Receive Timestamp Added (bit 7)
    // This bit is set when a 32-bit or 64-bit timestamp has been added to the beginning of the received
    // Ethernet frame. The MAC-PHY shall set this bit to zero when SV = 0.
    pub rtsa: bool,

    // Receive Timestamp Parity (bit 6)
    // Parity bit calculated over the 32-bit/64-bit timestamp added to the beginning of the received
    // Ethernet frame. Method used is odd parity as described in Section 8.5.2.
    // The MAC-PHY shall set this bit to zero when RTSA = 0.
    pub rtsp: bool,

    // Transmit Credits (bits 5-1)
    // This field contains the minimum number of transmit data chunks of frame data (DV = 1) that the SPI host
    // can write in a single transaction without incurring a transmit buffer overflow error. The TXC field
    // essentially denotes the minimum amount of free buffer space (in chunks) that the MAC-PHY has available
    // for accepting frame data written by the SPI host.
    pub txc: u8,

    // Parity bit (bit 0)
    // Parity bit calculated over the receive data footer using odd parity.
    pub parity: bool,
}

impl Default for ReceiveFooter {
    fn default() -> Self {
        Self::new()
    }
}

impl ReceiveFooter {
    pub fn new() -> Self {
        Self {
            exst: false,
            hdrb: false,
            sync: false,
            rca: 0,
            dv: false,
            sv: false,
            swo: 0,
            fd: false,
            ev: false,
            ebo: 0,
            rtsa: false,
            rtsp: false,
            txc: 0,
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
            if self.swo > 0b1111 {
                return false;
            }
            // if EBO is larger than 6 bits
            if self.ebo > 0b11_1111 {
                return false;
            }
        }

        // FD is only valid when EV = 1 and DV = 1
        if self.fd && (!self.ev || !self.dv) {
            return false;
        }

        // RTSA should be 0 when SV = 0
        if !self.sv && self.rtsa {
            return false;
        }

        // RTSP should be 0 when RTSA = 0
        if !self.rtsa && self.rtsp {
            return false;
        }

        // RCA should not be larger than 5 bits
        if self.rca > 0b1_1111 {
            return false;
        }

        // TXC should not be larger than 5 bits
        if self.txc > 0b1_1111 {
            return false;
        }

        // make sure its odd parity
        if self.parity != self.calc_parity_bit() {
            return false;
        }

        true
    }

    fn calc_parity_bit(&self) -> bool {
        self.exst as u32
            + self.hdrb as u32
            + self.sync as u32
            + self.rca.count_ones()
            + self.dv as u32
            + self.sv as u32
            + self.swo.count_ones()
            + self.fd as u32
            + self.ev as u32
            + self.ebo.count_ones()
            + self.rtsa as u32
            + self.rtsp as u32
            + self.txc.count_ones() % 2
            == 0
    }

    // will create a ReceiveFooter from a slice, if the slice is not 4 bytes it returns none
    pub fn from_slice(bytes: &[u8]) -> Option<Self> {
        use crate::utils::Bit;

        if bytes.len() != 4 {
            return None;
        }
        let ary = [bytes[0], bytes[1], bytes[2], bytes[3]];
        let mut footer = ReceiveFooter::new();
        let footer_u32 = u32::from_be_bytes(ary);
        footer.exst = footer_u32.get_bit(31);
        footer.hdrb = footer_u32.get_bit(30);
        footer.sync = footer_u32.get_bit(29);
        footer.rca = ((footer_u32 >> 24) & 0b1_1111) as u8;
        footer.dv = footer_u32.get_bit(21);
        footer.sv = footer_u32.get_bit(20);
        footer.swo = ((footer_u32 >> 16) & 0b1111) as u8;
        footer.fd = footer_u32.get_bit(15);
        footer.ev = footer_u32.get_bit(14);
        footer.ebo = ((footer_u32 >> 8) & 0b111111) as u8;
        footer.rtsa = footer_u32.get_bit(7);
        footer.rtsp = footer_u32.get_bit(6);
        footer.txc = ((footer_u32 >> 1) & 0b1_1111) as u8;
        footer.parity = footer_u32.get_bit(0);
        Some(footer)
    }

    pub fn to_bytes(&self) -> [u8; 4] {
        let mut footer_u32 = 0u32;
        footer_u32.set_bit(31, self.exst);
        footer_u32.set_bit(30, self.hdrb);
        footer_u32.set_bit(29, self.sync);
        footer_u32 |= (self.rca as u32 & 0b1_1111) << 24;
        footer_u32.set_bit(21, self.dv);
        footer_u32.set_bit(20, self.sv);
        footer_u32 |= (self.swo as u32 & 0b1111) << 16;
        footer_u32.set_bit(15, self.fd);
        footer_u32.set_bit(14, self.ev);
        footer_u32 |= (self.ebo as u32 & 0b111111) << 8;
        footer_u32.set_bit(7, self.rtsa);
        footer_u32.set_bit(6, self.rtsp);
        footer_u32 |= (self.txc as u32 & 0b1_1111) << 1;
        footer_u32.set_bit(0, self.parity);
        footer_u32.to_be_bytes()
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn parity_test() {
        let mut footer = ReceiveFooter::default();
        // bit should be true, so there is an odd number of bits
        assert!(footer.calc_parity_bit());

        footer.exst = true;
        // bit should be false so there is an odd number, since exst is set
        assert!(!footer.calc_parity_bit());
        footer.rca = 0b11;

        // added two bits should be same
        assert!(!footer.calc_parity_bit());
    }

    #[test]
    fn valid_test() {
        let mut footer = ReceiveFooter::default();
        // a footer with all zeros is not valid due to parity
        assert!(!footer.valid());

        footer.parity = true;
        assert!(footer.valid());

        footer.swo = 0b11;
        // footer not valid since swo is something and SV is not set, parity is the same since we added 2 bits
        assert!(!footer.valid());

        footer.swo = 0b111;
        footer.sv = true;
        // not valid since dv is not set
        assert!(!footer.valid());

        footer.dv = true;
        footer.parity = false;
        assert!(footer.valid());

        footer.swo = 0;
        footer.ev = true;
        footer.ebo = 0b11;

        // setting an end byte offset to after swo
        assert!(footer.valid())
    }
}
