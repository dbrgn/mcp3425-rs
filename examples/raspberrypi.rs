extern crate linux_embedded_hal as hal;
extern crate mcp3425;

use hal::{Delay, I2cdev};
use mcp3425::{MCP3425, SampleRate, Gain};

fn main() {
    println!("Hello, MCP3425!");

    let dev = I2cdev::new("/dev/i2c-1").unwrap();
    let address = 0x68;
    let mut adc = MCP3425::new(dev, address, Delay).with_sample_rate(SampleRate::SPS240Bits12);

    println!("Temperature 12 bit / 1x gain: {:?}", adc.oneshot().map_err(|e| format!("{:?}", e)));
    adc.set_sample_rate(SampleRate::SPS60Bits14);
    println!("Temperature 14 bit / 1x gain: {:?}", adc.oneshot().map_err(|e| format!("{:?}", e)));
    adc.set_sample_rate(SampleRate::SPS15Bits16);
    println!("Temperature 16 bit / 1x gain: {:?}", adc.oneshot().map_err(|e| format!("{:?}", e)));
    adc.set_gain(Gain::Gain2);
    println!("Temperature 16 bit / 2x gain: {:?}", adc.oneshot().map_err(|e| format!("{:?}", e)));
}
