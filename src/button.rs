use esp_idf_hal::gpio::{Input, PinDriver};

/// Polls an active-low button (pulled up, pressed = shorted to ground) and reports a
/// debounced press edge at most once per `debounce_ms`.
pub struct Button<'d> {
    pin: PinDriver<'d, Input>,
    poll_ms: u32,
    debounce_ms: u32,
    debounce_remaining_ms: u32,
    was_pressed: bool,
}

impl<'d> Button<'d> {
    pub fn new(pin: PinDriver<'d, Input>, poll_ms: u32, debounce_ms: u32) -> Self {
        Self {
            pin,
            poll_ms,
            debounce_ms,
            debounce_remaining_ms: 0,
            was_pressed: false,
        }
    }

    /// Call once per `poll_ms` tick. Returns true exactly once per debounced press.
    pub fn poll_pressed(&mut self) -> bool {
        let pressed = self.pin.is_low();
        let mut edge = false;

        if self.debounce_remaining_ms > 0 {
            self.debounce_remaining_ms = self.debounce_remaining_ms.saturating_sub(self.poll_ms);
        } else if pressed && !self.was_pressed {
            edge = true;
            self.debounce_remaining_ms = self.debounce_ms;
        }
        self.was_pressed = pressed;

        edge
    }
}
