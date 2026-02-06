mod register;
mod types;

use rtt_target::rprintln;
pub use types::{Error, Gain, InterruptSource, Mode, Resolution};

use embedded_hal::{delay::DelayNs, i2c::I2c};
use register as reg;

/// Driver for the LTR-390UV-01 UV / Ambient Light sensor.
pub struct Ltr390<I2C> {
    i2c: I2C,
    addr: u8,
}

impl<I2C, E> Ltr390<I2C>
where
    I2C: I2c<Error = E>,
{
    // ── Construction / teardown ────────────────────────────────────────────

    /// Create a new driver instance with the default I2C address (0x53).
    ///
    /// No bus traffic occurs here — call [`init`](Self::init) next.
    pub fn new(i2c: I2C) -> Self {
        Self {
            i2c,
            addr: reg::I2C_ADDRESS,
        }
    }

    /// Create a new driver instance with a custom I2C address.
    pub fn new_with_address(i2c: I2C, addr: u8) -> Self {
        Self { i2c, addr }
    }

    /// Release the I2C bus so it can be reused.
    pub fn release(self) -> I2C {
        self.i2c
    }

    // ── Initialisation ────────────────────────────────────────────────────

    /// Verify the part ID, soft-reset the device, and enable measurements.
    ///
    /// A `delay` provider is required for the 10 ms post-reset wait. You
    /// only need to pass it here — none of the other methods require one.
    pub fn init(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<E>> {
        // Check part ID (upper nibble of PART_ID register).
        let id = self.read_reg(reg::PART_ID)?;
        let part = id >> 4;
        if part != reg::EXPECTED_PART_ID {
            return Err(Error::InvalidPartId(part));
        }

        // // Soft reset.
        // self.reset(delay)?;


        // // Enable sensor.
        self.enable(true)?;

        // // Verify it actually turned on.
        if !self.is_enabled()? {
            return Err(Error::ResetFailed);
        }

        Ok(())
    }

    /// Perform a software reset.
    ///
    /// The LTR390 may NACK the write that sets the reset bit because it
    /// resets before it can ACK. We therefore ignore the write error and
    /// verify success by reading the bit back after the delay.
    pub fn reset(&mut self, delay: &mut impl DelayNs) -> Result<(), Error<E>> {
        // Set the reset bit — ignore errors (see note above).
        let _ = self.modify_reg(reg::MAIN_CTRL, |v| v | reg::MAIN_CTRL_RESET);

        delay.delay_ms(10);

        // Confirm the reset bit has cleared.
        let ctrl = self.read_reg(reg::MAIN_CTRL)?;
        if ctrl & reg::MAIN_CTRL_RESET != 0 {
            return Err(Error::ResetFailed);
        }

        Ok(())
    }

    // ── Enable / disable ──────────────────────────────────────────────────

    /// Enable or disable the sensor (MAIN_CTRL bit 1).
    pub fn enable(&mut self, enable: bool) -> Result<(), Error<E>> {
        self.modify_reg(reg::MAIN_CTRL, |v| {
            if enable {
                v | reg::MAIN_CTRL_ENABLE
            } else {
                v & !reg::MAIN_CTRL_ENABLE
            }
        })
    }

    /// Returns `true` if the sensor is currently enabled.
    pub fn is_enabled(&mut self) -> Result<bool, Error<E>> {
        let v = self.read_reg(reg::MAIN_CTRL)?;
        Ok(v & reg::MAIN_CTRL_ENABLE != 0)
    }

    // ── Mode ──────────────────────────────────────────────────────────────

    /// Select ALS or UVS measurement mode.
    pub fn set_mode(&mut self, mode: Mode) -> Result<(), Error<E>> {
        self.modify_reg(reg::MAIN_CTRL, |v| match mode {
            Mode::Als => v & !reg::MAIN_CTRL_UVS_MODE,
            Mode::Uvs => v | reg::MAIN_CTRL_UVS_MODE,
        })
    }

    /// Read the current measurement mode.
    pub fn mode(&mut self) -> Result<Mode, Error<E>> {
        let v = self.read_reg(reg::MAIN_CTRL)?;
        if v & reg::MAIN_CTRL_UVS_MODE != 0 {
            Ok(Mode::Uvs)
        } else {
            Ok(Mode::Als)
        }
    }

    // ── Gain ──────────────────────────────────────────────────────────────

    /// Set the sensor gain.
    pub fn set_gain(&mut self, gain: Gain) -> Result<(), Error<E>> {
        // Gain occupies bits [2:0] of the GAIN register.
        self.write_reg(reg::GAIN, gain as u8)
    }

    /// Read the current gain setting.
    pub fn gain(&mut self) -> Result<Gain, Error<E>> {
        let raw = self.read_reg(reg::GAIN)? & 0x07;
        Gain::from_raw(raw).ok_or(Error::InvalidValue(raw))
    }

    // ── Resolution ────────────────────────────────────────────────────────

    /// Set the ADC resolution / integration time.
    pub fn set_resolution(&mut self, res: Resolution) -> Result<(), Error<E>> {
        // Resolution occupies bits [6:4] of the MEAS_RATE register.
        self.modify_reg(reg::MEAS_RATE, |v| (v & 0x8F) | ((res as u8) << 4))
    }

    /// Read the current resolution setting.
    pub fn resolution(&mut self) -> Result<Resolution, Error<E>> {
        let raw = (self.read_reg(reg::MEAS_RATE)? >> 4) & 0x07;
        Resolution::from_raw(raw).ok_or(Error::InvalidValue(raw))
    }

    // ── Data readout ──────────────────────────────────────────────────────

    /// Returns `true` when a new measurement result is available.
    pub fn data_ready(&mut self) -> Result<bool, Error<E>> {
        let v = self.read_reg(reg::MAIN_STATUS)?;
        Ok(v & reg::STATUS_DATA_READY != 0)
    }

    /// Read the 20-bit ALS (ambient light) data register.
    ///
    /// Does **not** check whether new data is available — call
    /// [`data_ready`](Self::data_ready) first if you need that guarantee.
    pub fn read_als(&mut self) -> Result<u32, Error<E>> {
        self.read_3byte(reg::ALS_DATA_0)
    }

    /// Read the 20-bit UVS (ultraviolet) data register.
    ///
    /// Does **not** check whether new data is available — call
    /// [`data_ready`](Self::data_ready) first if you need that guarantee.
    pub fn read_uvs(&mut self) -> Result<u32, Error<E>> {
        self.read_3byte(reg::UVS_DATA_0)
    }

    // ── Interrupts / thresholds ───────────────────────────────────────────

    /// Set the 20-bit upper and lower interrupt thresholds.
    pub fn set_thresholds(&mut self, lower: u32, upper: u32) -> Result<(), Error<E>> {
        self.write_3byte(reg::THRESH_LOW_0, lower)?;
        self.write_3byte(reg::THRESH_UP_0, upper)?;
        Ok(())
    }

    /// Configure the interrupt output.
    ///
    /// * `enable`  — enable / disable the INT pin.
    /// * `source`  — which channel triggers the interrupt.
    /// * `persist` — number of consecutive out-of-range readings before
    ///              the interrupt fires (0 = every reading).
    pub fn configure_interrupt(
        &mut self,
        enable: bool,
        source: InterruptSource,
        persist: u8,
    ) -> Result<(), Error<E>> {
        // INT_CFG: bit 2 = enable, bits [5:4] = source.
        self.modify_reg(reg::INT_CFG, |v| {
            let mut out = v & !(reg::INT_CFG_ENABLE | reg::INT_CFG_SRC_MASK);
            if enable {
                out |= reg::INT_CFG_ENABLE;
            }
            out |= (source.register_value() << reg::INT_CFG_SRC_SHIFT) & reg::INT_CFG_SRC_MASK;
            out
        })?;

        // INT_PST: bits [7:4] = persistence count.
        self.modify_reg(reg::INT_PST, |v| {
            (v & !reg::INT_PST_MASK) | ((persist & 0x0F) << reg::INT_PST_SHIFT)
        })?;

        Ok(())
    }

    // ── Low-level register helpers ────────────────────────────────────────

    fn read_reg(&mut self, reg: u8) -> Result<u8, Error<E>> {
        let mut buf = [0u8];

        self.i2c
            .write_read(self.addr, &[reg], &mut buf)
            .map_err(Error::I2c)?;
        Ok(buf[0])
    }

    fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), Error<E>> {
        self.i2c.write(self.addr, &[reg, val]).map_err(Error::I2c)
    }

    /// Read-modify-write a single 8-bit register.
    fn modify_reg(&mut self, reg: u8, f: impl FnOnce(u8) -> u8) -> Result<(), Error<E>> {
        let val = self.read_reg(reg)?;
        self.write_reg(reg, f(val))
    }

    /// Read a 3-byte (20-bit) little-endian data register.
    fn read_3byte(&mut self, start_reg: u8) -> Result<u32, Error<E>> {
        let mut buf = [0u8; 3];
        self.i2c
            .write_read(self.addr, &[start_reg], &mut buf)
            .map_err(Error::I2c)?;
        Ok(u32::from(buf[0]) | (u32::from(buf[1]) << 8) | (u32::from(buf[2]) << 16))
    }

    /// Write a 3-byte (20-bit) little-endian register.
    fn write_3byte(&mut self, start_reg: u8, val: u32) -> Result<(), Error<E>> {
        let bytes = [
            start_reg,
            (val & 0xFF) as u8,
            ((val >> 8) & 0xFF) as u8,
            ((val >> 16) & 0x0F) as u8,
        ];
        self.i2c.write(self.addr, &bytes).map_err(Error::I2c)
    }
}
