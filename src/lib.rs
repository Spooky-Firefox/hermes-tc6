#![no_std]

#[cfg(test)]
extern crate std;

#[macro_use]
mod fmt;

/// Ready-made [`tc6_interface::Tc6Interface`] on top of a blocking `embedded-hal` SPI device.
/// Gated behind the `embedded-hal` feature (on by default).
#[cfg(feature = "embedded-hal")]
pub mod embedded_io_implementation;

pub mod tc6_interface;

pub mod tc6_driver;

mod bitfield;
pub mod frame_buffer;
pub mod registers;
pub mod tc6_footer;
pub mod tc6_header;
#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        let result = 2 + 2;
        assert_eq!(result, 4);
    }
}
