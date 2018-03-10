//! A platform agnostic Rust driver for the MCP3425, based on the
//! [`embedded-hal`](https://github.com/japaric/embedded-hal) traits.
//!
//! ## The Device
//!
//! The Microchip MCP3425 is a low-current 16-bit analog-to-digital converter.
//!
//! The device has an I²C interface and an on-board ±2048mV reference.
//!
//! - [Details and datasheet](http://www.microchip.com/wwwproducts/en/en533561)
//!
//! ## Usage
//!
//! ### Instantiating
//!
//! Import this crate and an `embedded_hal` implementation:
//!
//! ```
//! extern crate linux_embedded_hal as hal;
//! extern crate mcp3425;
//! ```
//!
//! Then instantiate the device in either
//! [`ContinuousMode`](struct.ContinuousMode.html) or
//! [`OneShotMode`](struct.OneShotMode.html):
//!
//! ```no_run
//! # extern crate linux_embedded_hal as hal;
//! # extern crate mcp3425;
//! use hal::{Delay, I2cdev};
//! use mcp3425::{MCP3425, Config, Resolution, Gain, Error, OneShotMode};
//!
//! # fn main() {
//! let dev = I2cdev::new("/dev/i2c-1").unwrap();
//! let address = 0x68;
//! let mut adc = MCP3425::new(dev, address, Delay, OneShotMode);
//! # }
//! ```
//!
//! (You can also use the shortcut functions
//! [`oneshot`](struct.MCP3425.html#method.oneshot) or
//! [`continuous`](struct.MCP3425.html#method.continuous) to create instances
//! of the [`MCP3425`](struct.MCP3425.html) type without having to specify the
//! type as parameter.)
//!
//! ### Configuration
//!
//! You can choose the conversion resolution / sample rate and the PGA gain
//! with a [`Config`](struct.Config.html) object.
//!
//! Use the methods starting with `with_` to create a (side-effect free) new
//! instance of the configuration where the specified setting has been
//! replaced.
//!
//! ```no_run
//! # extern crate mcp3425;
//! # use mcp3425::{Config, Resolution, Gain};
//! # fn main() {
//! let config = Config::new(Resolution::Bits12Sps240, Gain::Gain1);
//! let high_res = config.with_resolution(Resolution::Bits16Sps15);
//! let high_gain = high_res.with_gain(Gain::Gain8);
//! # }
//! ```
//!
//! ### Measurements
//!
//! You can trigger a one-shot measurement:
//!
//! ```no_run
//! # extern crate linux_embedded_hal as hal;
//! # extern crate mcp3425;
//! # use hal::{Delay, I2cdev};
//! # use mcp3425::{MCP3425, Config, Resolution, Gain, Error};
//! # fn main() {
//! # let dev = I2cdev::new("/dev/i2c-1").unwrap();
//! # let address = 0x68;
//! let mut adc = MCP3425::oneshot(dev, address, Delay);
//! let config = Config::new(Resolution::Bits12Sps240, Gain::Gain1);
//! match adc.measure(&config) {
//!     Ok(mv) => println!("ADC measured {} mV", mv),
//!     Err(Error::I2c(e)) => println!("An I2C error happened: {}", e),
//!     Err(Error::VoltageTooHigh) => println!("Voltage is too high to measure"),
//!     Err(Error::VoltageTooLow) => println!("Voltage is too low to measure"),
//! }
//! # }
//! ```
//!
//! As you can see, the saturation values are automatically converted to
//! proper errors.

#![no_std]
#![deny(missing_docs)]

extern crate byteorder;
extern crate embedded_hal as hal;

use byteorder::{BigEndian, ByteOrder};
use hal::blocking::delay::DelayMs;
use hal::blocking::i2c::{Read, Write, WriteRead};


/// All possible errors in this crate
#[derive(Debug)]
pub enum Error<E> {
    /// I2C bus error
    I2c(E),
    /// Voltage is too high to be measured
    VoltageTooHigh,
    /// Voltage is too low to be measured
    VoltageTooLow,
}


/// ADC reference voltage: +-2048mV
const REF_MILLIVOLTS: i16 = 2048;


/// User register value to start a conversion.
///
/// This sets the "Not Ready" bit to 1.
const START_CONVERSION: u8 = 0b10000000;


/// The two conversion mode structs implement this trait.
///
/// This allows the `MCP3425` instance to be generic over the conversion mode.
pub trait ConversionMode {
    /// Return the bitmask for this conversion mode
    fn val(&self) -> u8;
}

/// Use the MCP3425 in One-Shot mode.
pub struct OneShotMode;

impl ConversionMode for OneShotMode {
    fn val(&self) -> u8 {
        0b00000000
    }
}

/// Use the MCP3425 in Continuous Conversion mode.
pub struct ContinuousMode;

impl ConversionMode for ContinuousMode {
    fn val(&self) -> u8 {
        0b00010000
    }
}


/// Conversion bit resolution and sample rate
///
/// * 15 SPS -> 16 bits
/// * 60 SPS -> 14 bits
/// * 240 SPS -> 12 bits
///
/// Defaults to 12 bits / 240 SPS (`Bits12Sps240`),
/// matching the power-on defaults of the device.
#[allow(dead_code)]
#[derive(Debug, Copy, Clone)]
pub enum Resolution {
    /// 16 bits / 15 SPS. This allows you to measure voltage in 62.5 µV steps.
    Bits16Sps15 = 0b00001000,
    /// 14 bits / 60 SPS. This allows you to measure voltage in 250 µV steps.
    Bits14Sps60 = 0b00000100,
    /// 12 bits / 240 SPS. This allows you to measure voltage in 1 mV steps.
    Bits12Sps240 = 0b00000000,
}

impl Resolution {
    /// Return the bitmask for this sample rate.
    pub fn val(&self) -> u8 {
        *self as u8
    }

    /// Return the number of bits of accuracy this sample rate gives you.
    pub fn bits(&self) -> u8 {
        match *self {
            Resolution::Bits16Sps15 => 16,
            Resolution::Bits14Sps60 => 14,
            Resolution::Bits12Sps240 => 12,
        }
    }

    /// Return the maximum output code.
    pub fn max(&self) -> i16 {
        match *self {
            Resolution::Bits16Sps15 => 32767,
            Resolution::Bits14Sps60 => 8191,
            Resolution::Bits12Sps240 => 2047,
        }
    }

    /// Return the minimum output code.
    pub fn min(&self) -> i16 {
        match *self {
            Resolution::Bits16Sps15 => -32768,
            Resolution::Bits14Sps60 => -8192,
            Resolution::Bits12Sps240 => -2048,
        }
    }
}

impl Default for Resolution {
    /// Default implementation matching the power-on defaults of the device.
    fn default() -> Self {
        Resolution::Bits12Sps240
    }
}


/// Programmable gain amplifier (PGA)
///
/// Defaults to no amplification (`Gain1`),
/// matching the power-on defaults of the device.
#[allow(dead_code)]
#[derive(Debug, Copy, Clone)]
pub enum Gain {
    /// Amplification factor 1.
    Gain1 = 0b00000000,
    /// Amplification factor 2.
    Gain2 = 0b00000001,
    /// Amplification factor 4.
    Gain4 = 0b00000010,
    /// Amplification factor 8.
    Gain8 = 0b00000011,
}

impl Gain {
    /// Return the bitmask for this gain configuration.
    pub fn val(&self) -> u8 {
        *self as u8
    }
}

impl Default for Gain {
    /// Default implementation matching the power-on defaults of the device.
    fn default() -> Self {
        Gain::Gain1
    }
}


/// Device configuration: Resolution and gain
#[derive(Debug, Default, Copy, Clone)]
pub struct Config {
    /// Conversion bit resolution and sample rate
    pub resolution: Resolution,
    /// Programmable gain amplifier (PGA)
    pub gain: Gain,
}

impl Config {
    /// Create a new device configuration with the specified resolution /
    /// sample rate and gain.
    ///
    /// Note that creating and changing this instance does not have an
    /// immediate effect on the device. It is only written when a measurement
    /// is triggered (TODO: Or when writing config explicitly).
    pub fn new(resolution: Resolution, gain: Gain) -> Self {
        Config { resolution, gain }
    }

    /// Create a new configuration where the resolution has been replaced
    /// with the specified value.
    pub fn with_resolution(&self, resolution: Resolution) -> Self {
        Config {
            resolution,
            gain: self.gain,
        }
    }

    /// Create a new configuration where the gain has been replaced
    /// with the specified value.
    pub fn with_gain(&self, gain: Gain) -> Self {
        Config {
            resolution: self.resolution,
            gain,
        }
    }

    /// Return the bitmask for the combined configuration values.
    fn val(&self) -> u8 {
        self.resolution.val() | self.gain.val()
    }
}


/// Driver for the MCP3425 ADC
#[derive(Debug, Default)]
pub struct MCP3425<I2C, D, M> {
    i2c: I2C,
    address: u8,
    delay: D,
    mode: M,
}

impl<I2C, D, E, M> MCP3425<I2C, D, M>
where
    I2C: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>,
    D: DelayMs<u8>,
    M: ConversionMode,
{
    /// Initialize the MCP3425 driver.
    pub fn new(i2c: I2C, address: u8, delay: D, mode: M) -> Self {
        MCP3425 {
            i2c,
            address,
            delay,
            mode,
        }
    }

    /// Read an i16 from the device.
    fn read_i16(&mut self) -> Result<i16, Error<E>> {
        let mut buf = [0, 0];
        self.i2c.read(self.address, &mut buf).map_err(Error::I2c)?;
        Ok(BigEndian::read_i16(&buf))
    }
}

impl<I2C, D, E> MCP3425<I2C, D, OneShotMode>
where
    I2C: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>,
    D: DelayMs<u8>,
{
    /// Initialize the MCP3425 driver in One-Shot mode.
    pub fn oneshot(i2c: I2C, address: u8, delay: D) -> Self {
        MCP3425 {
            i2c,
            address,
            delay,
            mode: OneShotMode,
        }
    }

    /// Do a one-shot voltage measurement.
    ///
    /// Return the result in millivolts.
    ///
    /// TODO: Newtype for return value.
    pub fn measure(&mut self, config: &Config) -> Result<i16, Error<E>> {
        let command = START_CONVERSION
                    | self.mode.val()
                    | config.val();

        // Send command
        self.i2c
            .write(self.address, &[command])
            .map_err(Error::I2c)?;

        // Wait for conversion to finish
        self.delay.delay_ms(150);

        // Read result
        let val = self.read_i16()?;

        // Check against min/max codes
        if val == config.resolution.max() {
            Err(Error::VoltageTooHigh)
        } else if val == config.resolution.min() {
            Err(Error::VoltageTooLow)
        } else {
            Ok(calculate_voltage(val, &config.resolution))
        }
    }
}


impl<I2C, D, E> MCP3425<I2C, D, ContinuousMode>
where
    I2C: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>,
    D: DelayMs<u8>,
{
    /// Initialize the MCP3425 driver in Continuous Measurement mode.
    pub fn continuous(i2c: I2C, address: u8, delay: D) -> Self {
        MCP3425 {
            i2c,
            address,
            delay,
            mode: ContinuousMode,
        }
    }

}


/// Calculate the voltage for the measurement result at the specified sample rate.
fn calculate_voltage(measurement: i16, resolution: &Resolution) -> i16 {
    let converted = measurement as i32
        * (REF_MILLIVOLTS * 2) as i32
        / (1 << resolution.bits()) as i32;
    converted as i16
}
