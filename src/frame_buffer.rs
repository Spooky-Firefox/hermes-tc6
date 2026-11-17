//! A frame laid out inside one ring-buffer record.
//!
//! Every record the driver writes into a `bbqueue` framed ring has this body layout (bbqueue's
//! own `u16` length prefix sits in front of it and is not visible here):
//!
//! ```text
//! [ meta: start u16 LE, len u16 LE ][ frame region: capacity() bytes ............ ]
//!                                     ^ frame bytes at region[start .. start + len]
//! ```
//!
//! `start` is where the frame begins inside the region. On receive it is the first chunk's
//! start word offset (`SWO * 4`); the bytes before it are whatever the MAC-PHY sent ahead of
//! the frame. On transmit it is always 0. Keeping it as a field, rather than moving the frame
//! down to offset 0, means the receive path needs no extra copy and no special case for
//! `SWO != 0`.
//!
//! The region may extend past `start + len`: a receive grant is sized for the worst case, and
//! a transmit record is padded to whole 64-byte chunks so the SPI windows stay inside it.

use core::ops::DerefMut;

/// Bytes of driver metadata at the front of every record body.
pub const META_LEN: usize = 4;

#[derive(Debug)]
pub struct FrameBuffer<B: DerefMut<Target = [u8]>> {
    grant: B,
    start: usize,
    len: usize,
}

impl<B: DerefMut<Target = [u8]>> FrameBuffer<B> {
    /// An empty frame over a fresh write grant. Nothing is read from the grant, so stale
    /// bytes from earlier records in the same ring are harmless.
    pub fn fresh(grant: B) -> Self {
        assert!(grant.len() >= META_LEN);
        Self {
            grant,
            start: 0,
            len: 0,
        }
    }

    /// Re-open a committed record (a read grant) using the meta header written by
    /// [`write_meta`](Self::write_meta).
    pub fn from_record(grant: B) -> Self {
        assert!(grant.len() >= META_LEN);
        let start = u16::from_le_bytes([grant[0], grant[1]]) as usize;
        let len = u16::from_le_bytes([grant[2], grant[3]]) as usize;
        // Only this driver writes records, and `write_meta` always runs before a commit that
        // covers `META_LEN + start + len` bytes or more. A mismatch means memory corruption.
        assert!(
            start + len <= grant.len() - META_LEN,
            "frame record meta header out of range"
        );
        Self { grant, start, len }
    }

    /// Store `start`/`len` in the record's meta header. Call this before committing.
    pub fn write_meta(&mut self) {
        let start = u16::try_from(self.start).expect("frame start exceeds u16");
        let len = u16::try_from(self.len).expect("frame length exceeds u16");
        self.grant[0..2].copy_from_slice(&start.to_le_bytes());
        self.grant[2..4].copy_from_slice(&len.to_le_bytes());
    }

    /// Body bytes this frame needs committed: meta header + region up to the end of the frame.
    pub fn record_len(&self) -> usize {
        META_LEN + self.start + self.len
    }

    /// Hand back the underlying grant, to commit or release it.
    pub fn into_grant(self) -> B {
        self.grant
    }

    /// Size of the frame region.
    pub fn capacity(&self) -> usize {
        self.grant.len() - META_LEN
    }

    fn region(&mut self) -> &mut [u8] {
        &mut self.grant[META_LEN..]
    }

    /// Mutable slice of `len` bytes right after the frame so far. The caller writes into it and
    /// then calls [`add_len`](Self::add_len) for however much of it became frame data.
    pub fn get_mut_at_end(&mut self, len: usize) -> &mut [u8] {
        let end = self.start + self.len;
        assert!(end + len <= self.capacity());
        &mut self.region()[end..end + len]
    }

    /// Mutable view of `len` bytes starting at absolute region index `index`.
    ///
    /// The range may run past the end of the frame into spare capacity, so it is bounded only
    /// by [`capacity`](Self::capacity). Used by the multi-block transmit path, which hands the
    /// interface a window that starts 4 bytes *before* the send cursor (room for the chunk
    /// header) and runs one 64-byte block past it. On the final, partial block the window's
    /// tail sits in the record's padding.
    pub fn get_mut_at_index(&mut self, index: usize, len: usize) -> &mut [u8] {
        assert!(index + len <= self.capacity());
        &mut self.region()[index..index + len]
    }

    pub fn add_len(&mut self, len: usize) {
        let end = self.start + self.len;
        assert!(end + len <= self.capacity());
        self.len += len;
    }

    /// Bytes of spare capacity after the frame (`capacity - start - len`). This is the
    /// largest `len` a following [`add_len`](Self::add_len) / [`get_mut_at_end`](Self::get_mut_at_end)
    /// call can take without tripping their bounds assertions - callers handling untrusted
    /// on-wire framing should check against this instead of relying on the assert to fire.
    pub fn spare_capacity(&self) -> usize {
        self.capacity() - self.start - self.len
    }

    pub fn set_start(&mut self, start: usize) {
        assert!(start + self.len <= self.capacity());
        self.start = start;
    }

    /// The frame bytes, `region[start .. start + len]`.
    pub fn as_slice(&self) -> &[u8] {
        &self.grant[META_LEN + self.start..META_LEN + self.start + self.len]
    }

    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        let (s, l) = (self.start, self.len);
        &mut self.region()[s..s + l]
    }

    pub fn push_slice(&mut self, slice: &[u8]) {
        self.get_mut_at_end(slice.len()).copy_from_slice(slice);
        self.len += slice.len();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec;
    use std::vec::Vec;

    #[test]
    fn meta_round_trips_through_a_record() {
        let mut fb = FrameBuffer::fresh(vec![0xAAu8; META_LEN + 128]);
        fb.set_start(8);
        fb.push_slice(&[1, 2, 3, 4, 5]);
        fb.write_meta();
        assert_eq!(fb.record_len(), META_LEN + 8 + 5);

        let used = fb.record_len();
        let mut grant: Vec<u8> = fb.into_grant();
        grant.truncate(used); // what a read grant of the committed length looks like
        let fb = FrameBuffer::from_record(grant);
        assert_eq!(fb.as_slice(), &[1, 2, 3, 4, 5]);
    }

    #[test]
    #[should_panic(expected = "meta header out of range")]
    fn corrupt_meta_is_rejected() {
        let mut grant = vec![0u8; META_LEN + 10];
        grant[2..4].copy_from_slice(&20u16.to_le_bytes());
        let _ = FrameBuffer::from_record(grant);
    }
}
