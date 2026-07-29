#![no_std]
#![no_main]
#![feature(asm_experimental_arch)]
use esp_backtrace as _;

extern crate alloc;

use esp_hal::{
    clock::CpuClock,
    delay::Delay,
    main,
    usb_serial_jtag::UsbSerialJtag,
};

mod drivers;
#[path = "drivers/xtensa_lx7/xtensa_lx7_core_temperature.rs"]
pub mod xtensa_lx7_core_temperature;
#[path = "drivers/xtensa_lx7/xtensa_lx7_core_control.rs"]
pub mod xtensa_lx7_core_control;
#[path = "drivers/xtensa_lx7/xtensa_lx7_cpu_to_gpu.rs"]
mod xtensa_lx7_cpu_to_gpu;
#[path = "vendor/driver/ssd1306.rs"]
mod ssd1306;
mod components;
mod vendor;

use components::{board::Board, boot::run_bootloader};
use vendor::fastboot::Fastboot;

esp_bootloader_esp_idf::esp_app_desc!();

fn init() -> esp_hal::peripherals::Peripherals {
    esp_alloc::heap_allocator!(size: 96 * 1024);
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    esp_hal::init(config)
}

#[main]
fn main() -> ! {
    // VAR
    let version_bootloader = env!("CARGO_PKG_VERSION");
    let product = "ESP32 S3 Zero";
    let model = "zero";
    let serialno = "0";
    let secure = "yes";
    let mut unlocked = "yes";
    let mut current_slot = "a";
    let slot_count: u8 = 2;


    let peripherals = init();
    let delay = Delay::new();

    xtensa_lx7_core_control::init();

    let mut fastboot = Fastboot::new(UsbSerialJtag::new(peripherals.USB_DEVICE));
    let mut board = Board::new(
        peripherals.SPI2,
        peripherals.I2C0,
        peripherals.GPIO6,   // SCL  (display)
        peripherals.GPIO5,   // SDA  (display)
        peripherals.GPIO7,   // MISO (SD, unused)
        peripherals.GPIO8,   // CLK  (SD, unused)
        peripherals.GPIO9,   // MOSI (SD, unused)
        peripherals.GPIO10,  // CS   (SD, unused)
        peripherals.GPIO11,  // encoder A
        peripherals.GPIO12,  // encoder B
        peripherals.GPIO13,  // power button
        peripherals.GPIO21,  // status LED
    );
    run_bootloader(&mut board, &delay, &mut fastboot)
}