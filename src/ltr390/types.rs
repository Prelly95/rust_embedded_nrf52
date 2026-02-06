/// Enumerations and error types for the LTR390 driver.

// ── Error type ────────────────────────────────────────────────────────────────

/// All errors that can occur when interacting with the LTR390.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<I2C> {
    /// Underlying I2C bus error.
    I2c(I2C),
    /// Part ID register did not match the expected value (0xB).
    InvalidPartId(u8),
    /// Soft-reset bit did not clear within the expected time.
    ResetFailed,
    /// An invalid raw value was read for a bitfield enum.
    InvalidValue(u8),
}

// ── Measurement mode ──────────────────────────────────────────────────────────

/// Sensor measurement mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Mode {
    /// Ambient Light Sensing.
    Als = 0,
    /// Ultraviolet Sensing.
    Uvs = 1,
}

// ── Gain ──────────────────────────────────────────────────────────────────────

/// Sensor gain setting.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Gain {
    /// 1× gain.
    Gain1 = 0,
    /// 3× gain.
    Gain3 = 1,
    /// 6× gain.
    Gain6 = 2,
    /// 9× gain.
    Gain9 = 3,
    /// 18× gain (default).
    Gain18 = 4,
}

impl Gain {
    /// Convert a raw register value to a [`Gain`] variant.
    pub fn from_raw(val: u8) -> Option<Self> {
        match val {
            0 => Some(Self::Gain1),
            1 => Some(Self::Gain3),
            2 => Some(Self::Gain6),
            3 => Some(Self::Gain9),
            4 => Some(Self::Gain18),
            _ => None,
        }
    }

    /// Return the numeric gain multiplier (useful for lux calculation).
    pub fn factor(self) -> u8 {
        match self {
            Self::Gain1 => 1,
            Self::Gain3 => 3,
            Self::Gain6 => 6,
            Self::Gain9 => 9,
            Self::Gain18 => 18,
        }
    }
}

// ── Resolution ────────────────────────────────────────────────────────────────

/// ADC resolution / integration time.
///
/// Higher resolution → longer integration time → lower noise.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Resolution {
    /// 20-bit, 400 ms integration.
    Bit20 = 0,
    /// 19-bit, 200 ms integration.
    Bit19 = 1,
    /// 18-bit, 100 ms integration (default).
    Bit18 = 2,
    /// 17-bit, 50 ms integration.
    Bit17 = 3,
    /// 16-bit, 25 ms integration.
    Bit16 = 4,
    /// 13-bit, 12.5 ms integration.
    Bit13 = 5,
}

impl Resolution {
    /// Convert a raw register value to a [`Resolution`] variant.
    pub fn from_raw(val: u8) -> Option<Self> {
        match val {
            0 => Some(Self::Bit20),
            1 => Some(Self::Bit19),
            2 => Some(Self::Bit18),
            3 => Some(Self::Bit17),
            4 => Some(Self::Bit16),
            5 => Some(Self::Bit13),
            _ => None,
        }
    }

    /// The effective number of ADC bits for this resolution.
    pub fn bits(self) -> u8 {
        match self {
            Self::Bit20 => 20,
            Self::Bit19 => 19,
            Self::Bit18 => 18,
            Self::Bit17 => 17,
            Self::Bit16 => 16,
            Self::Bit13 => 13,
        }
    }
}

// ── Interrupt source (encoded value written to INT_CFG[5:4]) ──────────────────

/// Interrupt source selection (maps to INT_CFG register source bits).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum InterruptSource {
    /// Interrupt triggers on ALS channel data.
    Als,
    /// Interrupt triggers on UVS channel data.
    Uvs,
}

impl InterruptSource {
    /// Raw 2-bit value written to INT_CFG[5:4].
    pub(crate) fn register_value(self) -> u8 {
        match self {
            Self::Als => 0x01,
            Self::Uvs => 0x03,
        }
    }
}