#![no_main]
#![no_std]

mod ltr390;

use cortex_m::asm::nop;
use embedded_hal::digital::InputPin;
use embedded_hal::digital::OutputPin;

use embedded_hal::delay::DelayNs;
use nrf52840_hal::{
    self as hal, Timer,
    gpio::p0,
    pac,
    twim::{self, Twim},
};

use nrf52840_hal::gpio::Level;
use rtt_target::{rprintln, rtt_init_print};

use ltr390::{Gain, Ltr390, Mode, Resolution};

#[panic_handler] // panicking behavior
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    rtt_init_print!();
    ltr390_test()
}

fn ltr390_test() -> ! {
    rprintln!("LTR390 demo starting");

    let periph = pac::Peripherals::take().unwrap();

    // Set up GPIO port 0 for I2C pins.
    // Adjust these to match your board's SDA/SCL routing.
    let p0 = p0::Parts::new(periph.P0);
    let scl = p0.p0_05.into_floating_input().degrade();
    let sda = p0.p0_04.into_floating_input().degrade();

    let twim_pins = twim::Pins { scl, sda };

    let i2c = Twim::new(periph.TWIM0, twim_pins, twim::Frequency::K100);

    let mut delay = Timer::new(periph.TIMER0);

    // ── LTR390 init ───────────────────────────────────────────────────────
    let mut sensor = Ltr390::new(i2c);
    sensor.init(&mut delay).expect("LTR390 init failed");
    sensor.set_gain(Gain::Gain18).unwrap();
    sensor.set_resolution(Resolution::Bit20).unwrap();
    sensor.set_mode(Mode::Uvs).unwrap();

    rprintln!("LTR390 ready — entering measurement loop");

    // ── Main loop ─────────────────────────────────────────────────────────

    loop {
        // ── UV reading ────────────────────────────────────────────────────
        sensor.set_mode(Mode::Uvs).unwrap();
        while !sensor.data_ready().unwrap() {
            delay.delay_ms(10);
        }
        let uv_raw = sensor.read_uvs().unwrap();

        // Approximate UVI (gain=18, 20-bit, sensitivity ≈ 2300 cts/µW/cm²).
        let uvi_x100 = (uv_raw * 100) / 2300;

        // ── ALS reading ───────────────────────────────────────────────────
        sensor.set_mode(Mode::Als).unwrap();
        while !sensor.data_ready().unwrap() {
            delay.delay_ms(10);
        }
        let als_raw = sensor.read_als().unwrap();

        // Rough lux (gain=18, integration factor=4 for 20-bit):
        //   lux ≈ 0.6 × ALS / (gain × int_factor)
        // Using integer math: lux_x10 = (6 × als) / (18 × 4) = als / 12
        let lux_x10 = als_raw / 12;

        rprintln!(
            "UV raw: {}  UVI: {}.{}   ALS raw: {}  lux: {}.{}",
            uv_raw,
            uvi_x100 / 100,
            uvi_x100 % 100,
            als_raw,
            lux_x10 / 10,
            lux_x10 % 10,
        );

        delay.delay_ms(1000);
    }
}

fn hal_nrf52840_button() -> ! {
    rprintln!("Hello");
    let p = hal::pac::Peripherals::take().unwrap();
    let port0 = hal::gpio::p0::Parts::new(p.P0);
    let port1 = hal::gpio::p1::Parts::new(p.P1);
    let mut button = port1.p1_15.into_pullup_input().degrade();
    let mut led = port0.p0_30.into_push_pull_output(Level::Low);
    let mut led_state = false;
    led.set_state(led_state.into()).unwrap();

    loop {
        if button.is_low().unwrap() {
            rprintln!("Button Pressed");
            led_state = !led_state;
            led.set_state(led_state.into()).unwrap();
            while button.is_low().unwrap() {
                nop();
            }
            led_state = !led_state;
            led.set_state(led_state.into()).unwrap();
        }
    }
}
