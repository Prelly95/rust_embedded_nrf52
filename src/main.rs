mod button;
mod lights;
mod pattern;
mod state;
mod web;
mod wifi;

use std::sync::Arc;

use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::{PinDriver, Pull};
use esp_idf_hal::ledc::{config::TimerConfig, LedcDriver, LedcTimerDriver};
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::units::FromValueType;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::nvs::EspDefaultNvsPartition;

use button::Button;
use lights::Lights;
use pattern::Pattern;
use state::SharedState;

/// How often the button is polled and the light animation is advanced.
const POLL_MS: u32 = 20;
/// Ignore further button edges for this long after a press is registered.
const DEBOUNCE_MS: u32 = 200;
/// Initial brightness, as a percentage of the available duty range.
const DEFAULT_BRIGHTNESS_PERCENT: u32 = 50;
/// Initial animation pattern.
const DEFAULT_PATTERN: Pattern = Pattern::AlternateBreathe;
/// PWM frequency for both LEDC channels; `Lights` needs this too, to size fade steps.
const PWM_FREQUENCY_HZ: u32 = 5_000;
/// Initial duration of one breathing leg (valid range 500-10,000ms).
const DEFAULT_BREATHE_MS: u32 = 2_000;
/// Initial twinkle jump interval (valid range 10-400ms).
const DEFAULT_TWINKLE_INTERVAL_MS: u32 = 200;

fn main() -> anyhow::Result<()> {
    // It is necessary to call this function once. Otherwise, some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take()?;
    let sys_loop = EspSystemEventLoop::take()?;
    let nvs = EspDefaultNvsPartition::take()?;

    let timer = LedcTimerDriver::new(
        peripherals.ledc.timer0,
        &TimerConfig::new().frequency(PWM_FREQUENCY_HZ.Hz()),
    )?;
    let side_a = LedcDriver::new(peripherals.ledc.channel0, &timer, peripherals.pins.gpio3)?;
    let side_b = LedcDriver::new(peripherals.ledc.channel1, &timer, peripherals.pins.gpio4)?;
    let mut lights = Lights::new(
        side_a,
        side_b,
        DEFAULT_BRIGHTNESS_PERCENT,
        DEFAULT_PATTERN,
        PWM_FREQUENCY_HZ,
        DEFAULT_BREATHE_MS,
        DEFAULT_TWINKLE_INTERVAL_MS,
    )?;

    let mut button = Button::new(
        PinDriver::input(peripherals.pins.gpio9, Pull::Up)?,
        POLL_MS,
        DEBOUNCE_MS,
    );

    let shared_state = Arc::new(SharedState::new(
        true,
        DEFAULT_BRIGHTNESS_PERCENT as u8,
        DEFAULT_PATTERN,
        DEFAULT_BREATHE_MS as u16,
        DEFAULT_TWINKLE_INTERVAL_MS as u16,
    ));

    // A Wi-Fi hiccup shouldn't take the lights down with it - fall back to
    // button-only control rather than propagating the error out of main().
    match wifi::connect(peripherals.modem, sys_loop, nvs) {
        Ok(wifi) => match web::start_server(shared_state.clone()) {
            Ok(server) => {
                // Keep Wi-Fi and the HTTP server running for the lifetime of the
                // program; both would otherwise be torn down as soon as they're
                // dropped, and neither variable is ever read again after this point.
                core::mem::forget(wifi);
                core::mem::forget(server);
            }
            Err(err) => log::warn!("HTTP server failed to start ({err}); continuing with button control only"),
        },
        Err(err) => log::warn!("Wi-Fi unavailable ({err}); continuing with button control only"),
    }

    log::info!("Starting fairy lights");

    let mut applied_on = shared_state.is_on();
    let mut applied_brightness = shared_state.brightness_percent();
    let mut applied_pattern = shared_state.pattern();
    let mut applied_breathe_ms = shared_state.breathe_ms();
    let mut applied_twinkle_interval_ms = shared_state.twinkle_interval_ms();

    loop {
        if button.poll_pressed() {
            shared_state.set_on(!shared_state.is_on());
        }

        let on = shared_state.is_on();
        if on != applied_on {
            applied_on = on;
            lights.set_on(on)?;
        }

        let brightness = shared_state.brightness_percent();
        if brightness != applied_brightness {
            applied_brightness = brightness;
            lights.set_brightness_percent(brightness as u32)?;
        }

        let pattern = shared_state.pattern();
        if pattern != applied_pattern {
            applied_pattern = pattern;
            lights.set_pattern(pattern)?;
        }

        let breathe_ms = shared_state.breathe_ms();
        if breathe_ms != applied_breathe_ms {
            applied_breathe_ms = breathe_ms;
            lights.set_breathe_ms(breathe_ms as u32)?;
        }

        let twinkle_interval_ms = shared_state.twinkle_interval_ms();
        if twinkle_interval_ms != applied_twinkle_interval_ms {
            applied_twinkle_interval_ms = twinkle_interval_ms;
            lights.set_twinkle_interval_ms(twinkle_interval_ms as u32)?;
        }

        lights.tick(POLL_MS)?;

        FreeRtos::delay_ms(POLL_MS);
    }
}
