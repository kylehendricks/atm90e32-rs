use atm90e32::driver::{ATM90E32Driver, ATM90E32InitConfig, Phase};
use embedded_hal::spi::MODE_3;
use esp_idf_hal::delay::Ets;
use esp_idf_hal::gpio::Level;
use esp_idf_hal::spi::*;
use esp_idf_hal::units::FromValueType;
use std::error::Error;
use std::sync::Arc;

const PRE_CS_DELAY_US: u32 = 10;

fn main() -> Result<(), Box<dyn Error>> {
    esp_idf_sys::link_patches();

    let peripherals = Peripherals::take().unwrap();
    let pins = peripherals.pins;

    let spi_driver = SpiDriver::new(
        peripherals.spi2,
        pins.gpio18.into(),
        pins.gpio23.into(),
        Some(pins.gpio19.into()),
        Dma::Disabled,
    )?;

    let config = config::Config::new()
        .baudrate(200.kHz().into())
        .data_mode(MODE_3);

    let shared_driver = Arc::new(SpiSharedDeviceDriver::new(spi_driver, &config)?);

    let mut soft_cs_driver =
        SpiSoftCsDeviceDriver::new(shared_driver.clone(), pins.gpio5.into(), Level::High)?;
    soft_cs_driver.cs_pre_delay_us(PRE_CS_DELAY_US);

    let chip_config = &config.chip_config;
    let driver = ATM90E32Driver::new(
        soft_cs_driver,
        Ets,
        Some(ATM90E32InitConfig {
            frequency: chip_config.line_frequency,
            pga_gain: chip_config.pga_gain,
            phase_a_vol_gain: chip_config.phase_a.voltage_gain,
            phase_b_vol_gain: chip_config.phase_b.voltage_gain,
            phase_c_vol_gain: chip_config.phase_c.voltage_gain,
            phase_a_cur_gain: chip_config.phase_a.current_gain,
            phase_b_cur_gain: chip_config.phase_b.current_gain,
            phase_c_cur_gain: chip_config.phase_c.current_gain,
        }),
    );

    let voltage = driver.get_line_voltage(Phase::A)?;
    let active_power = driver.get_active_power(Phase::A)?;
    let forward_energy = driver.get_forward_active_energy(Phase::A)?;
}
