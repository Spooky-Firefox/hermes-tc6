pub mod control_header;
pub mod fake_tc6_device;
pub mod receive_footer;
pub mod transmit_header;
mod utils;
use fake_tc6_device::FakeTc6SpiDevice;

#[cfg(test)]
mod tests {

    use super::*;

    #[test]
    fn it_works() {
        let device = FakeTc6SpiDevice::new();
        // Add assertions to test the device's behavior
    }
}
