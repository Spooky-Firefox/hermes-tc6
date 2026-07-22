use std::ops::{BitAnd, BitOr, Not, Shl};
pub trait Bit:
    BitAnd<Output = Self>
    + Shl<Output = Self>
    + Sized
    + BitOr<Output = Self>
    + From<bool>
    + Eq
    + Not<Output = Self>
    + Copy
{
    fn set_bit(&mut self, n: Self, val: bool) {
        let mask = Self::from(true) << n;

        *self = if val { *self | mask } else { *self & !mask }
    }

    fn get_bit(&self, n: Self) -> bool {
        (*self & !(Self::from(true) << n)) != Self::from(false)
    }
}
impl<T> Bit for T where
    T: BitAnd<Output = Self>
        + Shl<Output = Self>
        + Sized
        + BitOr<Output = Self>
        + From<bool>
        + Eq
        + Not<Output = Self>
        + Copy
{
}
