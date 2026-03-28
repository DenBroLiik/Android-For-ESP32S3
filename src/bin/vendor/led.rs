use esp_hal::{
    delay::Delay,
    gpio::{Level, Output, OutputConfig},
};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RgbColor {
    pub red: u8,
    pub green: u8,
    pub blue: u8,
}

impl RgbColor {
    pub const OFF: Self = Self::new(0, 0, 0);
    pub const STARTUP_BLUE: Self = Self::new(0, 0, 32);
    pub const SUCCESS_GREEN: Self = Self::new(0, 32, 0);
    pub const ERROR_RED: Self = Self::new(32, 0, 0);
    pub const WRITE_YELLOW: Self = Self::new(32, 24, 0);
    pub const FASTBOOT_AMBER: Self = Self::new(32, 12, 0);

    pub const fn new(red: u8, green: u8, blue: u8) -> Self {
        Self { red, green, blue }
    }
}

pub struct StatusLed<'d> {
    data: Output<'d>,
}

// WS2812B timings for ESP32-S3 running at 80 MHz (1 cycle = 12.5 ns):
//
//   T1H = 800 ns  →  64 cycles
//   T1L = 450 ns  →  36 cycles   (total 100 cycles per bit)
//   T0H = 400 ns  →  32 cycles
//   T0L = 850 ns  →  68 cycles   (total 100 cycles per bit)
//   Reset         > 50 µs  (handled via delay.delay_micros)
//
// delay_nanos() cannot be used here: its overhead (~200 ns at 80 MHz due to
// function call + GPIO write setup) adds to T_HIGH unconditionally, making
// every 0-bit look like a 1-bit to the WS2812 → LED always shows white.
//
// Instead we use the Xtensa CCOUNT register (increments every CPU cycle),
// which is immune to opt-level and inlining decisions.

#[inline(always)]
fn ccount() -> u32 {
    let v: u32;
    // SAFETY: rsr.ccount is a read-only Xtensa architectural register.
    unsafe {
        core::arch::asm!(
            "rsr.ccount {0}",
            out(reg) v,
            options(nomem, nostack, preserves_flags)
        );
    }
    v
}

/// Busy-wait until `cycles` have elapsed since `start`.
#[inline(always)]
fn wait_cycles_from(start: u32, cycles: u32) {
    while ccount().wrapping_sub(start) < cycles {}
}

impl<'d> StatusLed<'d> {
    pub fn new(pin: impl esp_hal::gpio::OutputPin + 'd) -> Self {
        let data = Output::new(pin, Level::Low, OutputConfig::default());
        Self { data }
    }

    pub fn show(&mut self, delay: &Delay, color: RgbColor) {
        critical_section::with(|_| {
            // LED uses RGB wire order (not the WS2812 standard GRB).
            self.write_byte(color.red);
            self.write_byte(color.green);
            self.write_byte(color.blue);
        });
        // Reset: hold data line low for > 50 µs.
        // The line is already low after the last bit, so just wait.
        delay.delay_micros(80);
    }

    pub fn show_startup(&mut self, delay: &Delay) {
        self.show(delay, RgbColor::STARTUP_BLUE);
    }

    pub fn show_error(&mut self, delay: &Delay) {
        self.show(delay, RgbColor::ERROR_RED);
    }

    pub fn show_success(&mut self, delay: &Delay) {
        self.show(delay, RgbColor::SUCCESS_GREEN);
    }

    pub fn show_write(&mut self, delay: &Delay) {
        self.show(delay, RgbColor::WRITE_YELLOW);
    }

    pub fn show_fastboot(&mut self, delay: &Delay) {
        self.show(delay, RgbColor::FASTBOOT_AMBER);
    }

    pub fn off(&mut self, delay: &Delay) {
        self.show(delay, RgbColor::OFF);
    }

    fn write_byte(&mut self, byte: u8) {
        for bit in (0..8).rev() {
            self.write_bit((byte & (1 << bit)) != 0);
        }
    }

    #[inline(always)]
    fn write_bit(&mut self, bit: bool) {
        // Snapshot the counter *before* set_high so the GPIO write time
        // is included in T_HIGH — this prevents systematic overshoot.
        let start = ccount();
        self.data.set_high();

        if bit {
            // T1H = 64 cycles (800 ns at 80 MHz)
            wait_cycles_from(start, 64);
            self.data.set_low();
            // Hold for full bit period (100 cycles total)
            wait_cycles_from(start, 100);
        } else {
            // T0H = 32 cycles (400 ns at 80 MHz)
            wait_cycles_from(start, 32);
            self.data.set_low();
            // Hold for full bit period (100 cycles total)
            wait_cycles_from(start, 100);
        }
    }
}