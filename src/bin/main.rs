#![no_std]
#![no_main]
#![feature(asm_experimental_arch)]  // ← добавить
use esp_backtrace as _;

extern crate alloc;

use esp_hal::{
    clock::CpuClock,
    delay::Delay,
    main,
    usb_serial_jtag::UsbSerialJtag,
};

#[path = "drivers/xtensa_lx7/xtensa_lx7_cpu_to_gpu.rs"]
mod xtensa_lx7_cpu_to_gpu;
#[path = "drivers/ssd1306.rs"]
mod ssd1306;
mod components;
mod vendor;

use components::{board::Board, boot::run_bootloader};
use vendor::fastboot::FastbootPlus;

esp_bootloader_esp_idf::esp_app_desc!();

fn init() -> esp_hal::peripherals::Peripherals {
    esp_alloc::heap_allocator!(size: 96 * 1024);

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::_80MHz);
    esp_hal::init(config)
}

#[main]
fn main() -> ! {
    let peripherals = init();
    let delay = Delay::new();

    let mut fastboot = FastbootPlus::new(UsbSerialJtag::new(peripherals.USB_DEVICE));
    let mut board = Board::new(
        peripherals.SPI2,
        peripherals.I2C0,
        peripherals.GPIO21,
        peripherals.GPIO10,
        peripherals.GPIO8,
        peripherals.GPIO7,
        peripherals.GPIO12,
        peripherals.GPIO9,
        peripherals.GPIO11,
        peripherals.GPIO2,
        peripherals.GPIO1,
    );

    run_bootloader(&mut board, &delay, &mut fastboot)
}
