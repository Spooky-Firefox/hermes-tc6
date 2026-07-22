pub mod fake_tc6_device;
pub mod transmit_header;
mod utils;
pub use fake_tc6_device::FakeTc6Device;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let device = FakeTc6Device::new();
        // Add assertions to test the device's behavior
    }
}
