use esp_hal::{
    delay::Delay,
    gpio::{Input, InputConfig, Pin, Pull},
    i2c::master::{Config as I2cConfig, I2c},
    time::Rate,
};

use crate::{
    ssd1306::Ssd1306Display,
    vendor::led::{RgbColor, StatusLed},
};

pub const PIN_STATUS_LED: u8 = 21;
pub const PIN_SD_CS: u8 = 12;
pub const PIN_SD_MOSI: u8 = 11;
pub const PIN_SD_MISO: u8 = 10;
pub const PIN_DISPLAY_SDA: u8 = 2;
pub const PIN_DISPLAY_SCL: u8 = 1;
pub const PIN_ENCODER_A: u8 = 8;
pub const PIN_ENCODER_B: u8 = 7;
pub const PIN_POWER_BUTTON: u8 = 9;

const ENCODER_COLOR_PALETTE: [RgbColor; 8] = [
    RgbColor::new(255, 0, 0),
    RgbColor::new(255, 96, 0),
    RgbColor::new(255, 255, 0),
    RgbColor::new(0, 255, 0),
    RgbColor::new(0, 255, 255),
    RgbColor::new(0, 0, 255),
    RgbColor::new(180, 0, 255),
    RgbColor::new(255, 255, 255),
];

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SdStatus {
    NoResponse,
    Unknown,
}

impl SdStatus {
    pub fn is_present(self) -> bool {
        false
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::NoResponse => "no-response",
            Self::Unknown => "unknown",
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DisplayStatus {
    Deferred,
    Ready,
}

pub struct Board<'d> {
    pub status_led: StatusLed<'d>,
    encoder_a: Input<'d>,
    encoder_b: Input<'d>,
    power_button: Input<'d>,
    display: Option<Ssd1306Display<'d>>,
    display_status: DisplayStatus,
    sd_status: SdStatus,
    last_power_button_pressed: bool,
    torch_enabled: bool,
    power_click_count: u8,
    power_click_window_ticks: u8,
    encoder_state: u8,
    encoder_color_index: usize,
}

impl<'d> Board<'d> {
    pub fn new(
        _spi2: impl esp_hal::spi::master::Instance + 'd,
        i2c0: impl esp_hal::i2c::master::Instance + 'd,
        gpio21: impl Pin + 'd,
        _gpio10: impl Pin + 'd,
        gpio8: impl Pin + 'd,
        gpio7: impl Pin + 'd,
        _gpio12: impl Pin + 'd,
        gpio9: impl Pin + 'd,
        _gpio11: impl Pin + 'd,
        gpio2: impl Pin + 'd,
        gpio1: impl Pin + 'd,
    ) -> Self {
        let status_led = StatusLed::new(gpio21.degrade());
        let encoder_a = Input::new(gpio8.degrade(), InputConfig::default().with_pull(Pull::Up));
        let encoder_b = Input::new(gpio7.degrade(), InputConfig::default().with_pull(Pull::Up));
        let power_button = Input::new(gpio9.degrade(), InputConfig::default().with_pull(Pull::Up));

        let display = I2c::new(
            i2c0,
            I2cConfig::default().with_frequency(Rate::from_khz(400)),
        )
        .ok()
        .map(|i2c| i2c.with_sda(gpio2.degrade()).with_scl(gpio1.degrade()))
        .and_then(|i2c| Ssd1306Display::new(i2c).ok());

        let display_status = if display.is_some() {
            DisplayStatus::Ready
        } else {
            DisplayStatus::Deferred
        };

        let encoder_state = ((encoder_a.is_low() as u8) << 1) | (encoder_b.is_low() as u8);

        Self {
            status_led,
            encoder_a,
            encoder_b,
            power_button,
            display,
            display_status,
            sd_status: SdStatus::Unknown,
            last_power_button_pressed: false,
            torch_enabled: false,
            power_click_count: 0,
            power_click_window_ticks: 0,
            encoder_state,
            encoder_color_index: 0,
        }
    }

    pub fn probe_sd_card(&mut self) -> SdStatus {
        self.sd_status = SdStatus::NoResponse;
        self.sd_status
    }

    pub fn sd_card_present(&self) -> bool {
        self.sd_status.is_present()
    }

    pub fn sd_status(&self) -> SdStatus {
        self.sd_status
    }

    pub fn power_button_pressed(&self) -> bool {
        self.power_button.is_low()
    }

    pub fn encoder_active(&self) -> bool {
        self.encoder_a.is_low() || self.encoder_b.is_low()
    }

    pub fn display_status(&self) -> DisplayStatus {
        self.display_status
    }

    pub fn show_display_lines(&mut self, lines: [&str; 4]) {
        if let Some(display) = self.display.as_mut() {
            let _ = display.show_lines(lines);
        }
    }

    pub fn show_fastboot_logo(&mut self) {
        if let Some(display) = self.display.as_mut() {
            let _ = display.show_fastboot_logo();
        }
    }

    pub fn poll_encoder_color_adjust(&mut self, delay: &Delay) -> Option<RgbColor> {
        let current_state = ((self.encoder_a.is_low() as u8) << 1) | (self.encoder_b.is_low() as u8);
        let transition = (self.encoder_state << 2) | current_state;
        self.encoder_state = current_state;

        let delta = match transition {
            0b0001 | 0b0111 | 0b1110 | 0b1000 => 1i8,
            0b0010 | 0b0100 | 0b1101 | 0b1011 => -1i8,
            _ => 0,
        };

        if delta == 0 || !self.power_button_pressed() {
            return None;
        }

        self.power_click_count = 0;
        self.power_click_window_ticks = 0;

        let len = ENCODER_COLOR_PALETTE.len() as i32;
        let next = (self.encoder_color_index as i32 + delta as i32).rem_euclid(len) as usize;
        self.encoder_color_index = next;

        let color = ENCODER_COLOR_PALETTE[self.encoder_color_index];
        self.torch_enabled = true;
        self.status_led.show(delay, color);
        Some(color)
    }

    pub fn poll_torch_toggle(&mut self, delay: &Delay) -> Option<bool> {
        let pressed = self.power_button_pressed();
        let mut toggled = None;

        if self.power_click_window_ticks > 0 {
            self.power_click_window_ticks -= 1;
            if self.power_click_window_ticks == 0 {
                self.power_click_count = 0;
            }
        }

        if pressed && !self.last_power_button_pressed {
            self.power_click_count = self.power_click_count.saturating_add(1);
            self.power_click_window_ticks = 8;

            if self.power_click_count >= 2 {
                self.power_click_count = 0;
                self.power_click_window_ticks = 0;
                self.torch_enabled = !self.torch_enabled;
                if self.torch_enabled {
                    self.status_led.show(delay, RgbColor::new(255, 255, 255));
                } else {
                    self.status_led.off(delay);
                }
                toggled = Some(self.torch_enabled);
            }
        }

        self.last_power_button_pressed = pressed;
        toggled
    }
}
