# Rust ATM90E32 Driver
A platform-agnostic driver for the ATM90E32 energy monitor.
Implemented with [embedded-hal](https://docs.rs/embedded-hal/latest/embedded_hal/index.html) traits.
It contains an opinionated initialization of the driver for basic energy monitoring but allows full manual
control if desired.  Initialization code was heavily inspired by the [ESPHome](https://esphome.io/) implementation of
[ATM90E32 sensor](https://github.com/esphome/esphome/tree/dev/esphome/components/atm90e32).

## Example
An example is included in `examples/` for the ESP32.  It isn't officially included in the `Cargo.toml` because I
couldn't figure out a good way to have it compile with the esp rust toolchain without interfering with this library.