use linux_embedded_hal::{Delay, I2cdev};
use mcp3425::{Config, Gain, Resolution, MCP3425};

fn main() {
    println!("Hello, MCP3425!");

    let dev = I2cdev::new("/dev/i2c-1").unwrap();
    let address = 0x68;
    let mut adc = MCP3425::oneshot(dev, address, Delay);
    let config = Config::default();

    println!(
        "Temperature 12 bit / 1x gain: {:?}",
        adc.measure(&config).map_err(|e| format!("{:?}", e))
    );
    let config = config.with_resolution(Resolution::Bits14Sps60);
    println!(
        "Temperature 14 bit / 1x gain: {:?}",
        adc.measure(&config).map_err(|e| format!("{:?}", e))
    );
    let config = config.with_resolution(Resolution::Bits16Sps15);
    println!(
        "Temperature 16 bit / 1x gain: {:?}",
        adc.measure(&config).map_err(|e| format!("{:?}", e))
    );
    let config = config.with_gain(Gain::Gain2);
    println!(
        "Temperature 16 bit / 2x gain: {:?}",
        adc.measure(&config).map_err(|e| format!("{:?}", e))
    );
}
