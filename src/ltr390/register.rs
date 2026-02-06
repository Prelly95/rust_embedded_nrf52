/// LTR390 register addresses and constants.

/// Default I2C address.
pub const I2C_ADDRESS: u8 = 0x53;

// ── Register map ──────────────────────────────────────────────────────────────

pub const MAIN_CTRL: u8 = 0x00;
pub const MEAS_RATE: u8 = 0x04;
pub const GAIN: u8 = 0x05;
pub const PART_ID: u8 = 0x06;
pub const MAIN_STATUS: u8 = 0x07;
pub const ALS_DATA_0: u8 = 0x0D; // LSB of 3-byte ALS data
pub const UVS_DATA_0: u8 = 0x10; // LSB of 3-byte UVS data
pub const INT_CFG: u8 = 0x19;
pub const INT_PST: u8 = 0x1A;
pub const THRESH_UP_0: u8 = 0x21; // LSB of 3-byte upper threshold
pub const THRESH_LOW_0: u8 = 0x24; // LSB of 3-byte lower threshold

// ── MAIN_CTRL (0x00) bits ─────────────────────────────────────────────────────

pub const MAIN_CTRL_ENABLE: u8 = 1 << 1;
pub const MAIN_CTRL_UVS_MODE: u8 = 1 << 3;
pub const MAIN_CTRL_RESET: u8 = 1 << 4;

// ── MAIN_STATUS (0x07) bits ───────────────────────────────────────────────────

pub const STATUS_DATA_READY: u8 = 1 << 3;

// ── INT_CFG (0x19) bits ───────────────────────────────────────────────────────

pub const INT_CFG_ENABLE: u8 = 1 << 2;
pub const INT_CFG_SRC_SHIFT: u8 = 4;
pub const INT_CFG_SRC_MASK: u8 = 0x03 << INT_CFG_SRC_SHIFT;

// ── INT_PST (0x1A) bits ──────────────────────────────────────────────────────

pub const INT_PST_SHIFT: u8 = 4;
pub const INT_PST_MASK: u8 = 0x0F << INT_PST_SHIFT;

// ── Expected part ID ──────────────────────────────────────────────────────────

pub const EXPECTED_PART_ID: u8 = 0xB;