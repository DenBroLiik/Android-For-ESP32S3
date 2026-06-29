use esp_hal::{delay::Delay, system};
use esp_println::println;

use crate::{
    components::{
        board::{Board, DisplayStatus},
        diagnostics::{log_component_map, log_component_status, log_memory},
    },
    vendor::{
        fastboot::{FastbootExit, FastbootPlus, enter_fastboot},
        logo, ram::RAM, rtc,
    },
    xtensa_lx7_cpu_to_gpu::{GpuCommand, PixelFormat, XtensaLx7CpuToGpu},
};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BootError {
    SdCardMissing,
    DisplayNotConfigured,
}

impl BootError {
    pub fn message(self) -> &'static str {
        match self {
            Self::SdCardMissing => {
                "SD card did not respond on the configured SPI lines; no dedicated CD pin is configured"
            }
            Self::DisplayNotConfigured => "Display backend is not configured yet for the selected board",
        }
    }
}

pub fn run_bootloader<'a>(
    board: &mut Board<'a>,
    delay: &Delay,
    fastboot: &mut FastbootPlus<'a>,
) -> ! {
    board.status_led.show_startup(delay);
    board.show_display_lines(["ZEPHYR WATCH", "BOOTING", "", ""]);
    logo::print_boot_logo();
    println!("Bootloader start");
    log_component_map();
    board.probe_sd_card();
    log_component_status(board);

    rtc::set_date(2026, 2, 19);  // u16, u8, u8
    rtc::set_time(12, 0, 0);     // u8, u8, u8

    let mut ram = RAM::new(4, 65536, 320, 2048);
    let mut cpu_to_gpu = XtensaLx7CpuToGpu::new();
    let framebuffer = cpu_to_gpu
        .create_framebuffer(&mut ram, 64, 64, PixelFormat::Rgb565)
        .expect("failed to reserve framebuffer");
    let clear_frame = [0u8; 64 * 64 * 2];
    cpu_to_gpu
        .upload_frame(&mut ram, framebuffer, &clear_frame)
        .expect("failed to upload framebuffer");
    cpu_to_gpu.clear(0x0000);
    cpu_to_gpu.present(framebuffer);

    if !board.sd_card_present() {
        println!("Boot halted: no SD card detected, switching to fastboot.");
        board.status_led.show_error(delay);
        board.show_fastboot_logo();
        match enter_fastboot(
            board,
            delay,
            fastboot,
            BootError::SdCardMissing,
            &ram,
            &cpu_to_gpu,
        ) {
            FastbootExit::ContinueBoot => println!("fastboot+ released boot flow"),
            FastbootExit::Reboot => system::software_reset(),
        }
    }

    if matches!(board.display_status(), DisplayStatus::Deferred) {
        println!(
            "Display init warning: {}",
            BootError::DisplayNotConfigured.message()
        );
        fastboot.write_line("Display backend is still deferred.");
    }
    board.show_display_lines(["ZEPHYR WATCH", "BOOT OK", "DISPLAY READY", ""]);

    let gpu_preview = ram
        .gpu_slice(framebuffer.offset, 8)
        .expect("failed to read framebuffer preview");
    let supported_formats = PixelFormat::supported();

    log_memory(&ram, &cpu_to_gpu);
    println!(
        "GPU formats: RGB565={}B RGB888={}B Gray8={}B | FB preview: {:02X?}",
        supported_formats[0].bytes_per_pixel(),
        supported_formats[1].bytes_per_pixel(),
        supported_formats[2].bytes_per_pixel(),
        gpu_preview
    );

    for command in cpu_to_gpu.drain_commands() {
        match command {
            GpuCommand::Upload(handle) => {
                println!(
                    "GPU command: upload {} bytes at offset {} ({}x{}, stride {})",
                    handle.len, handle.offset, handle.width, handle.height, handle.stride
                );
            }
            GpuCommand::Present(handle) => {
                println!(
                    "GPU command: present framebuffer at offset {} ({}x{})",
                    handle.offset, handle.width, handle.height
                );
            }
            GpuCommand::Clear(color) => {
                println!("GPU command: clear color 0x{:04X}", color);
            }
        }
    }

    println!("Boot completed to diagnostic runtime.");
    board.status_led.show_startup(delay);
    delay.delay_millis(150);
    board.status_led.off(delay);

    let mut counter = 0;
    loop {
        counter += 1;

        for _ in 0..20 {
            let _ = board.poll_encoder_color_adjust(delay);

            if let Some(torch_enabled) = board.poll_torch_toggle(delay) {
                println!(
                    "Torch button on GPIO9 -> {}",
                    if torch_enabled { "ON" } else { "OFF" }
                );
            }

            delay.delay_millis(50);
            rtc::rtc_tick_ms(50); // ← сюда, каждые реальные 50ms
        }

        let (year, month, day) = rtc::get_date();
        let (hour, minute, second) = rtc::get_time();

        println!(
            "Counter: {} | Date: {:04}-{:02}-{:02} Time: {:02}:{:02}:{:02} | GPU cmds: {} | Frames: {} | SD={} | PWR={} | ENC={}",
            counter,
            year, month, day,
            hour, minute, second,
            cpu_to_gpu.pending_commands(),
            cpu_to_gpu.submitted_frames(),
            board.sd_card_present(),
            board.power_button_pressed(),
            board.encoder_active()
        );
    }
}