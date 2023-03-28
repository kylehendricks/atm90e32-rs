use crate::register::Register;
use embedded_hal::delay::DelayUs;
use embedded_hal::spi::{SpiBus, SpiBusRead, SpiBusWrite, SpiDevice};

const AFTER_REG_DELAY_US: u32 = 4;

#[derive(Copy, Clone)]
pub enum LineFrequency {
    F50Hz,
    F60Hz,
}

#[derive(Copy, Clone)]
pub enum PGAGain {
    Gain1X = 0,
    Gain2X = 1,
    Gain4X = 2,
}

#[derive(Eq, PartialEq, Hash)]
pub enum Phase {
    A,
    B,
    C,
}

#[derive(Copy, Clone, Debug)]
pub enum ATM90E32Error<SPI> {
    Connect,
    Spi(SPI),
    Delay,
}

pub struct ATM90E32InitConfig {
    /// The line frequency
    pub frequency: LineFrequency,
    /// The PGA gain, 1X, 2X, or 4X
    pub pga_gain: PGAGain,
    /// Phase A voltage gain
    pub phase_a_vol_gain: u16,
    /// Phase B voltage gain
    pub phase_b_vol_gain: u16,
    /// Phase C voltage gain
    pub phase_c_vol_gain: u16,
    /// Phase A current gain
    pub phase_a_cur_gain: u16,
    /// Phase B current gain
    pub phase_b_cur_gain: u16,
    /// Phase C current gain
    pub phase_c_cur_gain: u16,
}

pub struct ATM90E32Driver<SPI, Delay> {
    spi: SPI,
    delay: Delay,
    forward_active_energy_cumulative_a: u32,
    forward_active_energy_cumulative_b: u32,
    forward_active_energy_cumulative_c: u32,
    reverse_active_energy_cumulative_a: u32,
    reverse_active_energy_cumulative_b: u32,
    reverse_active_energy_cumulative_c: u32,
}

impl<SPI, Delay> ATM90E32Driver<SPI, Delay>
where
    SPI: SpiDevice,
    SPI::Bus: SpiBus,
    Delay: DelayUs,
{
    ///
    ///
    /// # Arguments
    ///
    /// * `spi`: The spi device.
    /// * `delay`: The microsecond delay
    /// * `init_config`: An optional opinionated config that sets up the ATM90E32 for basic energy
    /// monitoring. If this is not provided, all initialization will need to be done manually.
    ///
    /// returns: Result<ATM90E32Driver<SPI, Delay>, ATM90E32Error<<SPI as ErrorType>::Error>>
    pub fn new(
        spi: SPI,
        delay: Delay,
        init_config: Option<ATM90E32InitConfig>,
    ) -> Result<Self, ATM90E32Error<SPI::Error>> {
        let mut driver = Self {
            spi,
            delay,
            forward_active_energy_cumulative_a: 0,
            forward_active_energy_cumulative_b: 0,
            forward_active_energy_cumulative_c: 0,
            reverse_active_energy_cumulative_a: 0,
            reverse_active_energy_cumulative_b: 0,
            reverse_active_energy_cumulative_c: 0,
        };

        if let Some(config) = init_config {
            driver.write(Register::SoftReset, 0x789A)?;
            driver.write(Register::CfgRegAccEn, 0x55AA)?;
            driver.write(Register::MeterEn, 0x0001)?;

            let last_spi = driver.read16(Register::LastSPIData)?;

            if last_spi != 0x0001 {
                return Err(ATM90E32Error::Connect);
            }

            let mmode0: u16 = 0x0087
                | match config.frequency {
                    LineFrequency::F50Hz => 0 << 12,
                    LineFrequency::F60Hz => 1 << 12,
                };

            driver.write(Register::PLconstH, 0x0861)?;
            driver.write(Register::PLconstL, 0xC468)?;
            driver.write(Register::ZXConfig, 0xD654)?;
            driver.write(Register::MMode0, mmode0)?;
            driver.write(Register::MMode1, config.pga_gain as u16)?;
            driver.write(Register::PStartTh, 0x1D4C)?;
            driver.write(Register::QStartTh, 0x1D4C)?;
            driver.write(Register::PPhaseTh, 0x02EE)?;
            driver.write(Register::QPhaseTh, 0x02EE)?;
            driver.write(Register::UgainA, config.phase_a_vol_gain)?;
            driver.write(Register::UgainB, config.phase_b_vol_gain)?;
            driver.write(Register::UgainC, config.phase_c_vol_gain)?;
            driver.write(Register::IgainA, config.phase_a_cur_gain)?;
            driver.write(Register::IgainB, config.phase_b_cur_gain)?;
            driver.write(Register::IgainC, config.phase_c_cur_gain)?;
            driver.write(Register::CfgRegAccEn, 0x0000)?;
        }

        Ok(driver)
    }

    pub fn get_line_voltage(&mut self, phase: &Phase) -> Result<f32, ATM90E32Error<SPI::Error>> {
        Ok((self.read16(match phase {
            Phase::A => Register::UrmsA,
            Phase::B => Register::UrmsB,
            Phase::C => Register::UrmsC,
        })? / 100) as f32)
    }

    pub fn get_line_current(&mut self, phase: &Phase) -> Result<f32, ATM90E32Error<SPI::Error>> {
        Ok((self.read16(match phase {
            Phase::A => Register::IrmsA,
            Phase::B => Register::IrmsB,
            Phase::C => Register::IrmsC,
        })? / 1000) as f32)
    }

    pub fn get_active_power(&mut self, phase: &Phase) -> Result<f32, ATM90E32Error<SPI::Error>> {
        let (reg_h, reg_l) = match phase {
            Phase::A => (Register::PmeanA, Register::PmeanALSB),
            Phase::B => (Register::PmeanB, Register::PmeanBLSB),
            Phase::C => (Register::PmeanC, Register::PmeanCLSB),
        };
        Ok(self.read32(reg_h, reg_l)? as f32 * 0.00032)
    }

    pub fn get_reactive_power(&mut self, phase: &Phase) -> Result<f32, ATM90E32Error<SPI::Error>> {
        let (reg_h, reg_l) = match phase {
            Phase::A => (Register::QmeanA, Register::QmeanALSB),
            Phase::B => (Register::QmeanB, Register::QmeanBLSB),
            Phase::C => (Register::QmeanC, Register::QmeanCLSB),
        };
        Ok(self.read32(reg_h, reg_l)? as f32 * 0.00032)
    }

    pub fn get_power_factor(&mut self, phase: &Phase) -> Result<f32, ATM90E32Error<SPI::Error>> {
        Ok((self.read16(match phase {
            Phase::A => Register::PFmeanA,
            Phase::B => Register::PFmeanB,
            Phase::C => Register::PFmeanC,
        })? / 1000) as f32)
    }

    pub fn get_forward_active_energy(
        &mut self,
        phase: &Phase,
    ) -> Result<f32, ATM90E32Error<SPI::Error>> {
        let (val, cumulative) = match phase {
            Phase::A => (
                self.read16(Register::APenergyA)? as u32,
                &mut self.forward_active_energy_cumulative_a,
            ),
            Phase::B => (
                self.read16(Register::APenergyB)? as u32,
                &mut self.forward_active_energy_cumulative_b,
            ),
            Phase::C => (
                self.read16(Register::APenergyC)? as u32,
                &mut self.forward_active_energy_cumulative_c,
            ),
        };

        if u32::MAX - *cumulative > val {
            *cumulative += val;
        } else {
            *cumulative = val;
        }

        return Ok(*cumulative as f32 * 10.0 / 3200.0);
    }

    pub fn get_reverse_active_energy(
        &mut self,
        phase: &Phase,
    ) -> Result<f32, ATM90E32Error<SPI::Error>> {
        let (val, cumulative) = match phase {
            Phase::A => (
                self.read16(Register::ANenergyA)? as u32,
                &mut self.reverse_active_energy_cumulative_a,
            ),
            Phase::B => (
                self.read16(Register::ANenergyB)? as u32,
                &mut self.reverse_active_energy_cumulative_b,
            ),
            Phase::C => (
                self.read16(Register::ANenergyC)? as u32,
                &mut self.reverse_active_energy_cumulative_c,
            ),
        };

        if u32::MAX - *cumulative > val {
            *cumulative += val;
        } else {
            *cumulative = val;
        }

        return Ok(*cumulative as f32 * 10.0 / 3200.0);
    }

    pub fn get_line_frequency(&mut self) -> Result<f32, ATM90E32Error<SPI::Error>> {
        Ok((self.read16(Register::Freq)? / 100) as f32)
    }

    pub fn write(
        &mut self,
        register: Register,
        data: u16,
    ) -> Result<(), ATM90E32Error<SPI::Error>> {
        self.spi
            .transaction(|bus| {
                bus.write(&[0x00, register as u8])?;
                self.delay.delay_us(AFTER_REG_DELAY_US).unwrap();
                bus.write(&data.to_be_bytes())
            })
            .map_err(ATM90E32Error::Spi)
    }

    pub fn read16(&mut self, register: Register) -> Result<u16, ATM90E32Error<SPI::Error>> {
        let mut read_buffer = [0u8; 2];
        self.spi
            .transaction(|bus| {
                bus.write(&[(1 << 7), register as u8])?;
                self.delay.delay_us(AFTER_REG_DELAY_US).unwrap();
                bus.read(&mut read_buffer)
            })
            .map_err(ATM90E32Error::Spi)?;

        let result = u16::from_be_bytes(read_buffer);

        Ok(result)
    }

    pub fn read32(
        &mut self,
        register_h: Register,
        register_l: Register,
    ) -> Result<u32, ATM90E32Error<SPI::Error>> {
        let val_h = self.read16(register_h)?;
        let val_l = self.read16(register_l)?;

        Ok(((val_h as u32) << 16) | val_l as u32)
    }
}

#[cfg(feature = "std")]
impl<SPI> std::fmt::Display for ATM90E32Error<SPI>
where
    SPI: std::fmt::Debug,
{
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ATM90E32Error::Spi(spi) => write!(f, "SPI Error: {spi:?}"),
            ATM90E32Error::Delay => write!(f, "Delay Error"),
            ATM90E32Error::Connect => write!(f, "Unable to connect to ATM90E32"),
        }
    }
}

#[cfg(feature = "std")]
impl<SPI> std::error::Error for ATM90E32Error<SPI>
where
    SPI: std::fmt::Debug,
{
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        None
    }
}
