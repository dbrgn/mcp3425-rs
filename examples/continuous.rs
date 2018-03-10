extern crate embedded_hal as hal;
extern crate linux_embedded_hal as linux_hal;
extern crate mcp3425;

use hal::blocking::delay::DelayMs;
use linux_hal::{Delay, I2cdev};
use mcp3425::{MCP3425, Config, Resolution, Gain};

fn main() {
    println!("Hello, MCP3425!");

    let dev = I2cdev::new("/dev/i2c-1").unwrap();
    let address = 0x68;
    let mut adc = MCP3425::continuous(dev, address, Delay);
    let config = Config::new(Resolution::Bits16Sps15, Gain::Gain1);

    println!("Writing configuration to device: {:?}", &config);
    adc.set_config(&config).unwrap();
    Delay.delay_ms(150u8);
    println!("Reading measurement: {:?}", &adc.read_measurement().unwrap());
    println!("Sleeping 150ms");
    Delay.delay_ms(150u8);
    println!("Reading measurement: {:?}", &adc.read_measurement().unwrap());
    println!("Reading measurement: {:?}", &adc.read_measurement().unwrap());
}
