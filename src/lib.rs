//! Platform agnostic driver for the Microchip MCP3425 16-bit ADC.

#![no_std]

extern crate byteorder;
extern crate embedded_hal as hal;

use byteorder::{BigEndian, ByteOrder};
use hal::blocking::delay::DelayMs;
use hal::blocking::i2c::{Read, Write, WriteRead};


/// All possible errors.
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
/// This sets the "Ready" bit to 1.
const START_CONVERSION: u8 = 0b10000000;


/// Conversion mode.
///
/// Defaults to `OneShot`.
#[allow(dead_code)]
#[derive(Debug, Copy, Clone)]
enum ConversionMode {
    OneShot = 0b00000000,
    Continuous = 0b00010000,
}

impl ConversionMode {
    pub fn val(&self) -> u8 {
        *self as u8
    }
}

impl Default for ConversionMode {
    fn default() -> Self {
        ConversionMode::OneShot
    }
}


/// Sample rate / accuracy.
///
/// * 15 SPS -> 16 bits
/// * 60 SPS -> 14 bits
/// * 240 SPS -> 12 bits
///
/// Defaults to 15 SP / 16 bits.
#[allow(dead_code)]
#[derive(Debug, Copy, Clone)]
pub enum SampleRate {
    /// 15 SPS / 16 bits
    SPS15Bits16 = 0b00001000,
    /// 60 SPS / 14 bits
    SPS60Bits14 = 0b00000100,
    /// 240 SPS / 12 bits
    SPS240Bits12 = 0b00000000,
}

impl SampleRate {
    pub fn val(&self) -> u8 {
        *self as u8
    }

    pub fn bits(&self) -> u8 {
        match *self {
            SampleRate::SPS15Bits16 => 16,
            SampleRate::SPS60Bits14 => 14,
            SampleRate::SPS240Bits12 => 12,
        }
    }

    /// Return the maximum output code.
    pub fn max(&self) -> i16 {
        match *self {
            SampleRate::SPS15Bits16 => 32767,
            SampleRate::SPS60Bits14 => 8191,
            SampleRate::SPS240Bits12 => 2047,
        }
    }

    /// Return the minimum output code.
    pub fn min(&self) -> i16 {
        match *self {
            SampleRate::SPS15Bits16 => -32768,
            SampleRate::SPS60Bits14 => -8192,
            SampleRate::SPS240Bits12 => -2048,
        }
    }
}

impl Default for SampleRate {
    fn default() -> Self {
        SampleRate::SPS15Bits16
    }
}


/// Programmable gain amplifier (PGA).
///
/// Defaults to no amplification (`Gain1`).
#[allow(dead_code)]
#[derive(Debug, Copy, Clone)]
pub enum Gain {
    Gain1 = 0b00000000,
    Gain2 = 0b00000001,
    Gain4 = 0b00000010,
    Gain8 = 0b00000011,
}

impl Gain {
    pub fn val(&self) -> u8 {
        *self as u8
    }
}

impl Default for Gain {
    fn default() -> Self {
        Gain::Gain1
    }
}


/// MCP3425 driver
#[derive(Debug, Default)]
pub struct MCP3425<I2C, D> {
    i2c: I2C,
    address: u8,
    delay: D,
    sample_rate: SampleRate,
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
            sample_rate: Default::default(),
            gain: Default::default(),
        }
    }

    /// Set the sample rate (chained variant).
    pub fn with_sample_rate(mut self, sample_rate: SampleRate) -> Self {
        self.sample_rate = sample_rate;
        self
    }

    /// Set the sample rate (mutating variant).
    pub fn set_sample_rate(&mut self, sample_rate: SampleRate) {
        self.sample_rate = sample_rate;
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
                    | self.sample_rate.val()
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
        if val == self.sample_rate.max() {
            Err(Error::VoltageTooHigh)
        } else if val == self.sample_rate.min() {
            Err(Error::VoltageTooLow)
        } else {
            Ok(calculate_voltage(val, &self.sample_rate))
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
fn calculate_voltage(measurement: i16, sample_rate: &SampleRate) -> i16 {
    let converted = measurement as i32
        * (REF_MILLIVOLTS * 2) as i32
        / (1 << sample_rate.bits()) as i32;
    converted as i16
}
