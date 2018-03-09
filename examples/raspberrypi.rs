extern crate linux_embedded_hal as hal;
extern crate mcp3425;

use hal::{Delay, I2cdev};
use mcp3425::{MCP3425, Config, Resolution, Gain};

fn main() {
    println!("Hello, MCP3425!");

    let dev = I2cdev::new("/dev/i2c-1").unwrap();
    let address = 0x68;
    let mut adc = MCP3425::new(dev, address, Delay);
    let config = Config::new(Resolution::SPS240Bits12, Gain::Gain1);

    println!("Temperature 12 bit / 1x gain: {:?}", adc.oneshot(&config).map_err(|e| format!("{:?}", e)));
    let config = config.with_resolution(Resolution::SPS60Bits14);
    println!("Temperature 14 bit / 1x gain: {:?}", adc.oneshot(&config).map_err(|e| format!("{:?}", e)));
    let config = config.with_resolution(Resolution::SPS15Bits16);
    println!("Temperature 16 bit / 1x gain: {:?}", adc.oneshot(&config).map_err(|e| format!("{:?}", e)));
    let config = config.with_gain(Gain::Gain2);
    println!("Temperature 16 bit / 2x gain: {:?}", adc.oneshot(&config).map_err(|e| format!("{:?}", e)));
}
