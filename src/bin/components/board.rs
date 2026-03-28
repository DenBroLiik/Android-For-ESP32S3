use esp_hal::{
    Blocking,
    gpio::{Input, InputConfig, Output, OutputConfig, Pin, Pull},
    i2c::master::{Config as I2cConfig, I2c},
    spi::{
        Mode,
        master::{Config as SpiConfig, Spi},
    },
    time::Rate,
};

use crate::{ssd1306::Ssd1306Display, vendor::led::StatusLed};

pub const PIN_STATUS_LED: u8 = 21;
pub const PIN_SD_CS: u8 = 12;
pub const PIN_SD_CLK: u8 = 9;
pub const PIN_SD_MOSI: u8 = 11;
pub const PIN_SD_MISO: u8 = 10;
pub const PIN_DISPLAY_SDA: u8 = 2;
pub const PIN_DISPLAY_SCL: u8 = 1;
pub const PIN_ENCODER: u8 = 8;
pub const PIN_POWER_BUTTON: u8 = 7;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SdStatus {
    Present,
    NoResponse,
    BusError,
    Unknown,
}

impl SdStatus {
    pub fn is_present(self) -> bool {
        matches!(self, Self::Present)
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::Present => "present",
            Self::NoResponse => "no-response",
            Self::BusError => "bus-error",
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
    sd_cs: Output<'d>,
    sd_spi: Spi<'d, Blocking>,
    encoder: Input<'d>,
    power_button: Input<'d>,
    display: Option<Ssd1306Display<'d>>,
    display_status: DisplayStatus,
    sd_status: SdStatus,
}

impl<'d> Board<'d> {
    pub fn new(
        spi2: impl esp_hal::spi::master::Instance + 'd,
        i2c0: impl esp_hal::i2c::master::Instance + 'd,
        gpio21: impl Pin + 'd,
        gpio10: impl Pin + 'd,
        gpio8: impl Pin + 'd,
        gpio7: impl Pin + 'd,
        gpio12: impl Pin + 'd,
        gpio9: impl Pin + 'd,
        gpio11: impl Pin + 'd,
        gpio2: impl Pin + 'd,
        gpio1: impl Pin + 'd,
    ) -> Self {
        let status_led = StatusLed::new(gpio21.degrade());
        let encoder = Input::new(gpio8.degrade(), InputConfig::default().with_pull(Pull::Up));
        let power_button = Input::new(gpio7.degrade(), InputConfig::default().with_pull(Pull::Up));
        let mut sd_cs = Output::new(gpio12.degrade(), esp_hal::gpio::Level::High, OutputConfig::default());
        sd_cs.set_high();

        let sd_spi = Spi::new(
            spi2,
            SpiConfig::default()
                .with_frequency(Rate::from_khz(400))
                .with_mode(Mode::_0),
        )
        .expect("failed to initialize SPI2 for SD")
        .with_sck(gpio9.degrade())
        .with_mosi(gpio11.degrade())
        .with_miso(gpio10.degrade());

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

        Self {
            status_led,
            sd_cs,
            sd_spi,
            encoder,
            power_button,
            display,
            display_status,
            sd_status: SdStatus::Unknown,
        }
    }

    pub fn probe_sd_card(&mut self) -> SdStatus {
        self.sd_cs.set_high();
        let idle_clocks = [0xFFu8; 10];
        if self.sd_spi.write(&idle_clocks).is_err() {
            self.sd_status = SdStatus::BusError;
            return self.sd_status;
        }

        self.sd_cs.set_low();

        let cmd0 = [0x40, 0x00, 0x00, 0x00, 0x00, 0x95];
        for byte in cmd0 {
            let mut transfer = [byte];
            if self.sd_spi.transfer(&mut transfer).is_err() {
                self.sd_cs.set_high();
                self.sd_status = SdStatus::BusError;
                return self.sd_status;
            }
        }

        let mut status = SdStatus::NoResponse;
        for _ in 0..10 {
            let mut response = [0xFF];
            if self.sd_spi.transfer(&mut response).is_err() {
                status = SdStatus::BusError;
                break;
            }

            if response[0] == 0x01 {
                status = SdStatus::Present;
                break;
            }

            if response[0] != 0xFF {
                status = SdStatus::BusError;
                break;
            }
        }

        self.sd_cs.set_high();
        let _ = self.sd_spi.write(&[0xFF]);

        self.sd_status = status;
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
        self.encoder.is_low()
    }

    pub fn display_status(&self) -> DisplayStatus {
        self.display_status
    }

    pub fn show_display_lines(&mut self, lines: [&str; 4]) {
        if let Some(display) = self.display.as_mut() {
            let _ = display.show_lines(lines);
        }
    }
}
