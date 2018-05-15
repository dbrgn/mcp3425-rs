extern crate embedded_hal as hal;
extern crate linux_embedded_hal as linux_hal;
extern crate mcp3425;

use hal::blocking::delay::DelayMs;
use linux_hal::{Delay, I2cdev};
use mcp3425::{MCP3425, Config, Resolution, Gain};

fn main() {
    println!("Hello, MCP3425!");
    println!();
    println!("------");
    println!();
    println!("This example will write the config to the device.");
    println!("It will then immediately read data from the device, which will fail");
    println!("because no measurement was awaited.");
    println!("Then the program will sleep for 150ms and read the measurement twice");
    println!("in a row. The first measurement should succeed, the second one should");
    println!("fail because the ADC is being polled too quickly.");
    println!();
    println!("------");
    println!();

    let dev = I2cdev::new("/dev/i2c-1").unwrap();
    let address = 0x68;
    let mut adc = MCP3425::continuous(dev, address, Delay);
    let config = Config::new(Resolution::Bits16Sps15, Gain::Gain1);

    println!("Writing configuration to device: {:?}", &config);
    adc.set_config(&config).unwrap();
    println!("Reading measurement: {:?}", &adc.read_measurement());
    println!("Sleeping 150ms");
    Delay.delay_ms(150u8);
    println!("Reading measurement: {:?}", &adc.read_measurement());
    println!("Reading measurement: {:?}", &adc.read_measurement());
}
