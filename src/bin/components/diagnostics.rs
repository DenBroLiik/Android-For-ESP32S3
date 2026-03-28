use esp_println::println;

use crate::{
    components::board::{
        Board, PIN_DISPLAY_SCL, PIN_DISPLAY_SDA, PIN_ENCODER_A, PIN_ENCODER_B,
        PIN_POWER_BUTTON, PIN_SD_CS, PIN_SD_MISO, PIN_SD_MOSI, PIN_STATUS_LED,
    },
    xtensa_lx7_cpu_to_gpu::XtensaLx7CpuToGpu,
    vendor::ram::RAM,
};

pub fn log_component_map() {
    println!(
        "Pins: RGB_LED={} SD[CS={},CLK=disabled,MOSI={},MISO={},CD=none] DISP[SDA={},SCL={}] ENCODER[A={},B={}] POWER_BTN={}",
        PIN_STATUS_LED,
        PIN_SD_CS,
        PIN_SD_MOSI,
        PIN_SD_MISO,
        PIN_DISPLAY_SDA,
        PIN_DISPLAY_SCL,
        PIN_ENCODER_A,
        PIN_ENCODER_B,
        PIN_POWER_BUTTON
    );
}

pub fn log_component_status(board: &Board<'_>) {
    println!(
        "Component status: sd_present={} sd_status={} encoder_active={} power_pressed={} display={:?}",
        board.sd_card_present(),
        board.sd_status().as_str(),
        board.encoder_active(),
        board.power_button_pressed(),
        board.display_status()
    );
}

pub fn log_memory(ram: &RAM, cpu_to_gpu: &XtensaLx7CpuToGpu) {
    println!(
        "Memory Info: Bootloader: {}KiB, GPU (Reserved: {}KiB, Used: {}KiB, Free: {}KiB, Empty: {}), SRAM: {}KiB, PSRAM: {}KiB, RAM: {}KiB, Reserved Total: {}KiB, Used Total: {}KiB, Free Total: {}KiB, Available: {}KiB",
        ram.bootloader_memory(),
        ram.gpu_capacity(),
        ram.gpu_memory(),
        ram.gpu_free(),
        ram.gpu_is_empty(),
        ram.sram(),
        ram.psram(),
        ram.ram(),
        ram.total_reserved_bytes() / RAM::BYTES_PER_KIB,
        ram.total_used_bytes() / RAM::BYTES_PER_KIB,
        ram.total_memory(),
        ram.available_memory()
    );
    println!(
        "GPU queue: pending={} submitted_frames={}",
        cpu_to_gpu.pending_commands(),
        cpu_to_gpu.submitted_frames()
    );
}
