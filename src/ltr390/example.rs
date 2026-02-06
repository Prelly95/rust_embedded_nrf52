//! Example: reading ambient light and UV index from the LTR390.
//!
//! This is a *skeleton* that compiles against any target whose HAL provides
//! `embedded_hal::i2c::I2c` and `embedded_hal::delay::DelayNs`.  Fill in
//! the platform-specific setup block for your board (nRF52, STM32, RP2040, …).

#![no_std]
#![no_main]

// ── Bring in your HAL here ────────────────────────────────────────────────────
//
// For example, on an nRF52840 with Embassy you might write:
//
//   use embassy_nrf::{self as hal, twim, peripherals};
//   use embassy_time::Delay;
//
// Or on an RP2040:
//
//   use rp2040_hal::{self as hal, i2c, pac};
//   use cortex_m::delay::Delay;

use ltr390::{Gain, Ltr390, Mode, Resolution};

// Placeholder panic handler — replace with your HAL's or `panic-probe`, etc.
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}

#[cortex_m_rt::entry]
fn main() -> ! {
    // ── Platform-specific I2C + delay setup ───────────────────────────────
    //
    // Replace the two lines below with real initialisation for your board.
    // For instance on an STM32F4:
    //
    //   let dp = pac::Peripherals::take().unwrap();
    //   let cp = cortex_m::Peripherals::take().unwrap();
    //   let rcc = dp.RCC.constrain();
    //   let clocks = rcc.cfgr.freeze();
    //   let gpiob = dp.GPIOB.split();
    //   let scl = gpiob.pb6.into_alternate_open_drain();
    //   let sda = gpiob.pb7.into_alternate_open_drain();
    //   let i2c = I2c::new(dp.I2C1, (scl, sda), 100.kHz(), &clocks);
    //   let mut delay = cp.SYST.delay(&clocks);
    //
    let i2c = todo!("initialise your platform's I2C peripheral");
    let mut delay = todo!("initialise your platform's delay provider");

    // ── Driver setup ──────────────────────────────────────────────────────

    let mut sensor = Ltr390::new(i2c);

    // init() verifies the part ID, soft-resets, and enables measurements.
    sensor.init(&mut delay).expect("LTR390 init failed");

    // Configure for UV sensing at maximum sensitivity.
    sensor.set_mode(Mode::Uvs).unwrap();
    sensor.set_gain(Gain::Gain18).unwrap();
    sensor.set_resolution(Resolution::Bit20).unwrap();

    loop {
        // Wait for fresh data.
        while !sensor.data_ready().unwrap() {
            delay.delay_ms(10);
        }

        let uv_raw = sensor.read_uvs().unwrap();

        // Approximate UV Index per LTR390 application note:
        //   UVI = UVS_DATA / (gain_factor × integration_factor) × WFAC
        // where WFAC ≈ 1.0 for open-air, no window.  With gain = 18 and
        // 20-bit resolution the sensitivity coefficient from the datasheet
        // is 2300 counts/(µW/cm²).
        let uvi: f32 = uv_raw as f32 / 2300.0;

        // --- do something with uvi, e.g. defmt::info!("{}", uvi); ---

        // Switch to ALS mode and take a light reading too.
        sensor.set_mode(Mode::Als).unwrap();

        while !sensor.data_ready().unwrap() {
            delay.delay_ms(10);
        }

        let als_raw = sensor.read_als().unwrap();

        // Rough lux estimate (gain=18, 20-bit, 0.6 integration factor):
        //   Lux ≈ 0.6 × ALS_DATA / (gain × integration)
        // Tune with your own window factor / calibration.
        let lux: f32 = 0.6 * als_raw as f32 / (18.0 * 4.0);

        // --- do something with lux ---

        // Go back to UV for the next loop.
        sensor.set_mode(Mode::Uvs).unwrap();

        let _ = (uvi, lux); // silence unused warnings in this skeleton
    }
}