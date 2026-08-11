#![no_std]

#[cfg(test)]
extern crate std;

/// TODO add feature for this and probably change module/file name to something better
pub mod embeded_io_implementation;

pub mod tc6_interface;

pub mod tc6_driver;

mod bitfield;
pub mod tc6_footer;
pub mod tc6_header;
#[cfg(test)]
mod tests {
    use std::vec;

    use super::*;

    #[test]
    fn it_works() {
        let result = 2 + 2;
        assert_eq!(result, 4);
    }
}
