//!
//! A platform-agnostic driver for the ATM90E32 energy metering device. Built using embedded-hal.
//!
//! Initialization logic was heavily inspired by ESPHome's [implementation for the ATM90E32 sensor](https://github.com/esphome/esphome/tree/dev/esphome/components/atm90e32).
//! Main inspiration for writing this driver was to make an alternative firmware for the amazing
//! [CircuitSetup Expandable 6 Channel ESP32 Energy Meter](https://circuitsetup.us/product/expandable-6-channel-esp32-energy-meter/)
//!

#![cfg_attr(not(feature = "std"), no_std)]

pub mod driver;
pub mod register;

pub use driver::*;
pub use register::Register;
