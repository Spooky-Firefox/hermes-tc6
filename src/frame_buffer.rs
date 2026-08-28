use core::mem::MaybeUninit;
#[derive(Debug)]
pub struct FrameBuffer<const M: usize> {
    start: usize,
    len: usize,
    buffer: [MaybeUninit<u8>; M],
}

impl<const M: usize> FrameBuffer<M> {
    pub const fn new() -> Self {
        Self {
            start: 0,
            len: 0,
            buffer: [MaybeUninit::uninit(); M],
        }
    }

    /// this function gets a mutable slice to the uninitialized portion of the buffer, starting at the end of the initialized portion. The caller is responsible for writing to this slice and then calling `add_len` to update the length of the initialized portion.
    pub unsafe fn get_mut_at_end(&mut self, len: usize) -> &mut [u8] {
        let end = self.start + self.len;
        assert!(end + len <= M);
        let ptr = unsafe { self.buffer.as_mut_ptr().add(end) } as *mut u8;
        unsafe { core::slice::from_raw_parts_mut(ptr, len) }
    }

    /// Mutable view of `len` bytes starting at absolute buffer index `index`.
    ///
    /// Like [`get_mut_at_end`](Self::get_mut_at_end), the range is allowed to run past the
    /// initialized region (`start + len`) into spare capacity, so it is bounded only by the
    /// total capacity `M`. Bytes beyond `start + len` must not be read as initialized.
    ///
    /// Used by the multi-block transmit path, which hands the interface a window that starts
    /// 4 bytes *before* the send cursor (room for the chunk header) and runs one 64-byte
    /// block past it - the tail of that window may sit in not-yet-written frame bytes on the
    /// final, partial block.
    pub unsafe fn get_mut_at_index(&mut self, index: usize, len: usize) -> &mut [u8] {
        assert!(index + len <= M);
        let ptr = unsafe { self.buffer.as_mut_ptr().add(index) } as *mut u8;
        unsafe { core::slice::from_raw_parts_mut(ptr, len) }
    }

    pub unsafe fn add_len(&mut self, len: usize) {
        let end = self.start + self.len;
        assert!(end + len <= M);
        self.len += len;
    }

    pub unsafe fn set_start(&mut self, start: usize) {
        assert!(start <= M);
        self.start = start;
    }

    pub unsafe fn get_raw_from_index(&self, index: usize, len: usize) -> &[u8] {
        let end = self.start + self.len;
        assert!(index + len <= end);
        let ptr = unsafe { self.buffer.as_ptr().add(index) } as *const u8;
        unsafe { core::slice::from_raw_parts(ptr, len) }
    }

    pub unsafe fn get_raw_mut_from_index(&mut self, index: usize, len: usize) -> &mut [u8] {
        let end = self.start + self.len;
        assert!(index + len <= end);
        let ptr = unsafe { self.buffer.as_mut_ptr().add(index) } as *mut u8;
        unsafe { core::slice::from_raw_parts_mut(ptr, len) }
    }

    /// View the initialized portion as a normal &[u8].
    pub fn as_slice(&self) -> &[u8] {
        // SAFETY: bytes in [start, start+len) are guaranteed initialized
        // by the get_mut_at_end/add_len invariant.
        unsafe {
            let ptr = self.buffer.as_ptr().add(self.start) as *const u8;
            core::slice::from_raw_parts(ptr, self.len)
        }
    }

    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        // SAFETY: bytes in [start, start+len) are guaranteed initialized
        // by the get_mut_at_end/add_len invariant.
        unsafe {
            let ptr = self.buffer.as_mut_ptr().add(self.start) as *mut u8;
            core::slice::from_raw_parts_mut(ptr, self.len)
        }
    }

    pub fn push_slice(&mut self, slice: &[u8]) {
        let len = slice.len();
        let end = self.start + self.len;
        assert!(end + len <= M);
        unsafe {
            self.get_mut_at_end(len).copy_from_slice(slice);
        }
        self.len += len;
    }
}
