// small bit-twiddling helpers shared by the header/footer wire-format types, kept as free
// functions operating on plain u32s so those types can stay a single 4-byte word (no per-field
// storage) which matters on embedded targets.

pub(crate) const fn get_bit(word: u32, bit: u32) -> bool {
    (word >> bit) & 1 != 0
}

pub(crate) const fn set_bit(word: u32, bit: u32, val: bool) -> u32 {
    if val {
        word | (1 << bit)
    } else {
        word & !(1 << bit)
    }
}

pub(crate) const fn get_bits(word: u32, shift: u32, mask: u32) -> u32 {
    (word >> shift) & mask
}

pub(crate) const fn set_bits(word: u32, shift: u32, mask: u32, val: u32) -> u32 {
    (word & !(mask << shift)) | ((val & mask) << shift)
}
