#![no_std]
#![no_main]
use esp_backtrace as _;

extern crate alloc;
use esp_hal::{clock::CpuClock, delay::Delay, main};
use esp_println::println;

mod vendor;
use vendor::ram::RAM;
use vendor::rtc;

esp_bootloader_esp_idf::esp_app_desc!();

fn init() {
    // Initialize heap allocator
    esp_alloc::heap_allocator!(size: 72 * 1024);

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::_80MHz);
    let _peripherals = esp_hal::init(config);
}

#[main]
fn main() -> ! {
    init();

    // Инициализация RTC
    rtc::set_date(2026, 2, 19);
    rtc::set_time(12, 0, 0);

    // Инициализация памяти
    let ram = RAM::new(4, 65536, 320, 2048); // 4KB для bootloader (в PSRAM), 64KB зарезервировано для GPU, 320KB SRAM, 2MB PSRAM

    println!(
        "Memory Info: Bootloader: {}KB, GPU (Reserved: {}KB, Used: {}KB), SRAM: {}KB, PSRAM: {}KB, Total: {}KB",
        ram.bootloader_memory(),
        ram.gpu_capacity(),
        ram.gpu_memory(),
        ram.sram(),
        ram.psram(),
        ram.total_memory()
    );

    let delay = Delay::new();
    let mut counter = 0;

    loop {
        // Обновляем RTC каждую секунду
        rtc::rtc_tick();
        counter += 1;

        // Получаем текущие дату и время
        let (year, month, day) = rtc::get_date();
        let (hour, minute, second) = rtc::get_time();

        println!(
            "Counter: {} | Date: {:04}-{:02}-{:02} Time: {:02}:{:02}:{:02}",
            counter, year, month, day, hour, minute, second
        );

        delay.delay_millis(1000);
    }
}
