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
//! Then instantiate the device:
//!
//! ```no_run
//! # extern crate linux_embedded_hal as hal;
//! # extern crate mcp3425;
//! use hal::{Delay, I2cdev};
//! use mcp3425::{MCP3425, Resolution, Gain, Error};
//!
//! # fn main() {
//! let dev = I2cdev::new("/dev/i2c-1").unwrap();
//! let address = 0x68;
//! let mut adc = MCP3425::new(dev, address, Delay);
//! # }
//! ```
//!
//! ### Configuration
//!
//! You can use the methods starting with `with_` to configure the device in a
//! chained fashion:
//!
//! ```no_run
//! # extern crate linux_embedded_hal as hal;
//! # extern crate mcp3425;
//! # use hal::{Delay, I2cdev};
//! # use mcp3425::{MCP3425, Resolution, Gain};
//! # fn main() {
//! # let dev = I2cdev::new("/dev/i2c-1").unwrap();
//! # let address = 0x68;
//! let mut adc = MCP3425::new(dev, address, Delay)
//!     .with_resolution(Resolution::SPS240Bits12)
//!     .with_gain(Gain::Gain1);
//! # }
//! ```
//!
//! Or you can use the methods starting with `set_` to mutate the instance in-place.
//!
//! ```no_run
//! # extern crate linux_embedded_hal as hal;
//! # extern crate mcp3425;
//! # use hal::{Delay, I2cdev};
//! # use mcp3425::{MCP3425, Resolution, Gain};
//! # fn main() {
//! # let dev = I2cdev::new("/dev/i2c-1").unwrap();
//! # let address = 0x68;
//! let mut adc = MCP3425::new(dev, address, Delay);
//! adc.set_resolution(Resolution::SPS240Bits12);
//! adc.set_gain(Gain::Gain1);
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
//! # use mcp3425::{MCP3425, Error};
//! # fn main() {
//! # let dev = I2cdev::new("/dev/i2c-1").unwrap();
//! # let address = 0x68;
//! # let mut adc = MCP3425::new(dev, address, Delay);
//! match adc.oneshot() {
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


/// Conversion mode.
///
/// Defaults to single conversion (`OneShot`).
#[allow(dead_code)]
#[derive(Debug, Copy, Clone)]
enum ConversionMode {
    OneShot = 0b00000000,
    Continuous = 0b00010000,
}

impl ConversionMode {
    /// Return the bitmask for this conversion mode.
    pub fn val(&self) -> u8 {
        *self as u8
    }
}

impl Default for ConversionMode {
    fn default() -> Self {
        ConversionMode::OneShot
    }
}


/// Conversion bit resolution and sample rate
///
/// * 15 SPS -> 16 bits
/// * 60 SPS -> 14 bits
/// * 240 SPS -> 12 bits
///
/// Defaults to 15 SP / 16 bits (`SPS15Bits16`).
#[allow(dead_code)]
#[derive(Debug, Copy, Clone)]
pub enum Resolution {
    /// 15 SPS / 16 bits. This allows you to measure voltage in 62.5 µV steps.
    SPS15Bits16 = 0b00001000,
    /// 60 SPS / 14 bits. This allows you to measure voltage in 250 µV steps.
    SPS60Bits14 = 0b00000100,
    /// 240 SPS / 12 bits. This allows you to measure voltage in 1 mV steps.
    SPS240Bits12 = 0b00000000,
}

impl Resolution {
    /// Return the bitmask for this sample rate.
    pub fn val(&self) -> u8 {
        *self as u8
    }

    /// Return the number of bits of accuracy this sample rate gives you.
    pub fn bits(&self) -> u8 {
        match *self {
            Resolution::SPS15Bits16 => 16,
            Resolution::SPS60Bits14 => 14,
            Resolution::SPS240Bits12 => 12,
        }
    }

    /// Return the maximum output code.
    pub fn max(&self) -> i16 {
        match *self {
            Resolution::SPS15Bits16 => 32767,
            Resolution::SPS60Bits14 => 8191,
            Resolution::SPS240Bits12 => 2047,
        }
    }

    /// Return the minimum output code.
    pub fn min(&self) -> i16 {
        match *self {
            Resolution::SPS15Bits16 => -32768,
            Resolution::SPS60Bits14 => -8192,
            Resolution::SPS240Bits12 => -2048,
        }
    }
}

impl Default for Resolution {
    fn default() -> Self {
        Resolution::SPS15Bits16
    }
}


/// Programmable gain amplifier (PGA)
///
/// Defaults to no amplification (`Gain1`).
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
    fn default() -> Self {
        Gain::Gain1
    }
}


/// Driver for the MCP3425 ADC
#[derive(Debug, Default)]
pub struct MCP3425<I2C, D> {
    i2c: I2C,
    address: u8,
    delay: D,
    resolution: Resolution,
    gain: Gain,
}

impl<I2C, D, E> MCP3425<I2C, D>
where
    I2C: Read<Error = E> + Write<Error = E> + WriteRead<Error = E>,
    D: DelayMs<u8>,
{
    /// Initialize the MCP3425 driver.
    pub fn new(i2c: I2C, address: u8, delay: D) -> Self {
        MCP3425 {
            i2c,
            address,
            delay,
            resolution: Default::default(),
            gain: Default::default(),
        }
    }

    /// Set the sample rate (chained variant).
    pub fn with_resolution(mut self, resolution: Resolution) -> Self {
        self.resolution = resolution;
        self
    }

    /// Set the sample rate (mutating variant).
    pub fn set_resolution(&mut self, resolution: Resolution) {
        self.resolution = resolution;
    }

    /// Set the gain (chained variant).
    pub fn with_gain(mut self, gain: Gain) -> Self {
        // TODO: In continuous mode, this will not update the actual config
        // register. Should it?
        self.gain = gain;
        self
    }

    /// Set the gain (mutating variant).
    pub fn set_gain(&mut self, gain: Gain) {
        // TODO: In continuous mode, this will not update the actual config
        // register. Should it?
        self.gain = gain;
    }

    /// Do a one-shot voltage measurement.
    ///
    /// Return the result in millivolts.
    ///
    /// TODO: Newtype for return value.
    pub fn oneshot(&mut self) -> Result<i16, Error<E>> {
        let command = START_CONVERSION
                    | ConversionMode::OneShot.val()
                    | self.resolution.val()
                    | self.gain.val();

        // Send command
        self.i2c
            .write(self.address, &[command])
            .map_err(Error::I2c)?;

        // Wait for conversion to finish
        self.delay.delay_ms(150);

        // Read result
        let val = self.read_i16()?;

        // Check against min/max codes
        if val == self.resolution.max() {
            Err(Error::VoltageTooHigh)
        } else if val == self.resolution.min() {
            Err(Error::VoltageTooLow)
        } else {
            Ok(calculate_voltage(val, &self.resolution))
        }
    }

    /// Read an i16 from the device.
    fn read_i16(&mut self) -> Result<i16, Error<E>> {
        let mut buf = [0, 0];
        self.i2c.read(self.address, &mut buf).map_err(Error::I2c)?;
        Ok(BigEndian::read_i16(&buf))
    }
}

/// Calculate the voltage for the measurement result at the specified sample rate.
fn calculate_voltage(measurement: i16, resolution: &Resolution) -> i16 {
    let converted = measurement as i32
        * (REF_MILLIVOLTS * 2) as i32
        / (1 << resolution.bits()) as i32;
    converted as i16
}
