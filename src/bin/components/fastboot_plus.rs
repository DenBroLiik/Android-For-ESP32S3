use esp_hal::delay::Delay;
use esp_println::println;

use crate::{
    components::{board::Board, boot::BootError, diagnostics::log_memory},
    vendor::{
        fastboot::{FastbootCommand, FastbootPlus},
        rtc,
    },
    xtensa_lx7_cpu_to_gpu::XtensaLx7CpuToGpu,
    vendor::ram::RAM,
};

pub enum FastbootExit {
    ContinueBoot,
    Reboot,
}

fn flash_error_then_restore_fastboot(board: &mut Board<'_>, delay: &Delay) {
    board.status_led.show_error(delay);
    delay.delay_millis(1000);
    board.status_led.show_fastboot(delay);
}

fn print_menu(fastboot: &mut FastbootPlus<'_>) {
    fastboot.write_line("");
    fastboot.write_line("fastboot+ quick actions:");
    fastboot.write_line("  status   pins   memory   rtc");
    fastboot.write_line("  logs on  logs off");
    fastboot.write_line("  led R G B BRT  e.g. led 255 128 0 4");
    fastboot.write_line("  boot     reboot");
    fastboot.write_prompt();
}

pub fn enter_fastboot(
    board: &mut Board<'_>,
    delay: &Delay,
    fastboot: &mut FastbootPlus<'_>,
    reason: BootError,
    ram: &RAM,
    cpu_to_gpu: &XtensaLx7CpuToGpu,
) -> FastbootExit {
    println!();
    println!("========== FASTBOOT+ ==========");
    println!("Fastboot reason: {}", reason.message());
    println!("Device is waiting in diagnostics mode.");
    println!("No firmware boot will be attempted until the fault is fixed.");
    println!("If SD is missing, insert the card and reset the board.");
    println!("===============================");
    fastboot.print_banner();
    fastboot.write_line(reason.message());
    print_menu(fastboot);

    let mut ticker = 0u32;
    let mut stream_logs = false;

    #[derive(Clone, Copy, PartialEq)]
    enum LedMode {
        Fastboot, // solid amber — default
        Manual,   // user set a specific color, hold it
    }
    let mut led_mode = LedMode::Fastboot;

    // Initial solid amber.
    board.status_led.show_fastboot(delay);

    loop {
        if let Some(command) = fastboot.poll_command() {
            match command {
                FastbootCommand::Help => fastboot.print_help(),
                FastbootCommand::Status => {
                    let sd_status = board.probe_sd_card();
                    fastboot.write_line("Board status:");
                    fastboot.write_fmt_line(format_args!(
                        "  sd_present={} ({})",
                        sd_status.is_present(),
                        sd_status.as_str()
                    ));
                    fastboot.write_line(if board.encoder_active() {
                        "  encoder_active=true"
                    } else {
                        "  encoder_active=false"
                    });
                    fastboot.write_line(if board.power_button_pressed() {
                        "  power_pressed=true"
                    } else {
                        "  power_pressed=false"
                    });
                    fastboot.write_fmt_line(format_args!(
                        "  display={:?}",
                        board.display_status()
                    ));
                    fastboot.write_prompt();
                }
                FastbootCommand::Pins => {
                    fastboot.write_line("Pin map:");
                    fastboot.write_line("  RGB_LED=21");
                    fastboot.write_line("  SD: CS=12 CLK=disabled MOSI=11 MISO=10 CD=none");
                    fastboot.write_line("  DISPLAY: SDA=2 SCL=1");
                    fastboot.write_line("  ENCODER: A=8 B=7");
                    fastboot.write_line("  POWER_BUTTON=9");
                    fastboot.write_prompt();
                }
                FastbootCommand::Memory => {
                    fastboot.write_line("Memory snapshot written to main console.");
                    log_memory(ram, cpu_to_gpu);
                    fastboot.write_prompt();
                }
                FastbootCommand::Rtc => {
                    let (year, month, day) = rtc::get_date();
                    let (hour, minute, second) = rtc::get_time();
                    fastboot.write_fmt_line(format_args!(
                        "RTC {:04}-{:02}-{:02} {:02}:{:02}:{:02}",
                        year, month, day, hour, minute, second
                    ));
                    fastboot.write_prompt();
                }
                FastbootCommand::LogsOn => {
                    stream_logs = true;
                    led_mode = LedMode::Manual;
                    board.status_led.show_success(delay);
                    fastboot.write_line("Live fastboot logs enabled.");
                    fastboot.write_prompt();
                }
                FastbootCommand::LogsOff => {
                    stream_logs = false;
                    led_mode = LedMode::Manual;
                    board.status_led.show_success(delay);
                    fastboot.write_line("Live fastboot logs disabled.");
                    fastboot.write_prompt();
                }
                FastbootCommand::Led { color, brightness } => {
                    led_mode = LedMode::Manual;
                    let scaled = color.with_brightness(brightness);
                    board.status_led.show(delay, scaled);
                    fastboot.write_fmt_line(format_args!(
                        "LED -> rgb({}, {}, {}) brightness {}",
                        color.red, color.green, color.blue, brightness
                    ));
                    fastboot.write_prompt();
                }
                FastbootCommand::Boot => {
                    let sd_status = board.probe_sd_card();
                    if sd_status.is_present() {
                        fastboot.write_line("SD detected, leaving fastboot+ and continuing boot.");
                        board.status_led.show_success(delay);
                        return FastbootExit::ContinueBoot;
                    }

                    fastboot.write_fmt_line(format_args!(
                        "ERR SD probe result: {}, boot denied.",
                        sd_status.as_str()
                    ));
                    flash_error_then_restore_fastboot(board, delay);
                    fastboot.write_prompt();
                }
                FastbootCommand::Reboot => {
                    fastboot.write_line("Reboot requested.");
                    board.status_led.show_write(delay);
                    return FastbootExit::Reboot;
                }
                FastbootCommand::Unknown => {
                    fastboot.write_line("ERR unknown command");
                    flash_error_then_restore_fastboot(board, delay);
                    fastboot.write_prompt();
                }
            }
        }

        if board.poll_encoder_color_adjust(delay).is_some() {
            led_mode = LedMode::Manual;
        }

        if board.poll_torch_toggle(delay).is_some() {
            // Keep the torch toggle independent from USB monitor presence.
            // Writing to USB Serial/JTAG without an attached host can stall
            // the fastboot loop after a few buffered messages.
        }

        // LED mode dispatch — runs every 100 ms tick.
        match led_mode {
            LedMode::Fastboot => {}
            LedMode::Manual => {}
        }
        ticker = ticker.wrapping_add(1);

        if stream_logs && ticker % 10 == 0 {
            println!(
                "[fastboot+] sd_present={} sd_status={} encoder_active={} power_pressed={}",
                board.sd_card_present(),
                board.sd_status().as_str(),
                board.encoder_active(),
                board.power_button_pressed()
            );
        }

        delay.delay_millis(50);
    }
}
