#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h"
#include "esp_rom_gpio.h"
#include "esp_secure_boot.h"
#include "esp_image_format.h"
#include "mbedtls/sha256.h"
#include "esp_partition.h"
#include "esp_ota_ops.h"
#include "driver/uart.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "esp_spiffs.h"  // Added for SPIFFS

static const char *TAG = "BOOT";

// Custom hardware init
void custom_hardware_init(void) {
    // CPU, memory, clocks: Handled by ROM/esp_clk_init/esp_psram_init

    // GPIO example
    esp_rom_gpio_pad_select_gpio(GPIO_NUM_18);
    gpio_set_direction(GPIO_NUM_18, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_18, 1);

    // SPI init (SPI2_HOST for ST7789/SD)
    spi_bus_config_t buscfg = {
        .mosi_io_num = 23, .miso_io_num = -1, .sclk_io_num = 18,
        .quadwp_io_num = -1, .quadhd_io_num = -1, .max_transfer_sz = 4092,
        .flags = SPICOMMON_BUSFLAG_MASTER
    };
    spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);

    // ST7789 device
    spi_device_interface_config_t devcfg_st = {
        .mode = 0, .clock_speed_hz = 20000000, .spics_io_num = 16,
        .queue_size = 7
    };
    spi_device_handle_t spi_st;
    spi_bus_add_device(SPI2_HOST, &devcfg_st, &spi_st);

    // SD device (half-duplex) - Fixed order: flags before queue_size
    spi_device_interface_config_t devcfg_sd = {
        .mode = 0, .clock_speed_hz = 20000000, .spics_io_num = 9,
        .flags = SPI_DEVICE_HALFDUPLEX, .queue_size = 1
    };
    spi_device_handle_t spi_sd;
    spi_bus_add_device(SPI2_HOST, &devcfg_sd, &spi_sd);

    // I2C init for SSD1306 - Use separate assignments to avoid initializer issues
    i2c_master_bus_config_t i2c_cfg = {0};
    i2c_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    i2c_cfg.i2c_port = 0;
    i2c_cfg.scl_io_num = 21;
    i2c_cfg.sda_io_num = 22;
    i2c_cfg.glitch_ignore_cnt = 7;
    i2c_cfg.flags.enable_internal_pullup = true;

    i2c_master_bus_handle_t i2c_bus;
    i2c_new_master_bus(&i2c_cfg, &i2c_bus);

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7, .device_address = 0x3C,
        .scl_speed_hz = 100000
    };
    i2c_master_dev_handle_t i2c_dev;
    i2c_master_bus_add_device(i2c_bus, &dev_cfg, &i2c_dev);

    // UART init for fastboot emulation
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_0, 1024, 1024, 0, NULL, 0);

    ESP_LOGI(TAG, "Hardware initialized");
}

// Integrity check - Fixed to use esp_image_load_metadata and metadata.start_addr
void custom_integrity_check(void) {
    esp_image_metadata_t metadata;
    const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
    esp_image_load_metadata((const void *)part->address, &metadata);

    uint8_t digest[32];
    mbedtls_sha256((uint8_t*)metadata.start_addr, metadata.image_len, digest, 0);

    // Compare with expected_hash (store in secure NVS or define)
    extern const uint8_t expected_hash[32];  // Define elsewhere
    if (memcmp(digest, expected_hash, 32) != 0) {
        ESP_LOGE(TAG, "SHA256 fail");
        esp_restart();
    }

    // Secure Boot V2 - Fixed arguments
    if (esp_secure_boot_verify_signature(metadata.start_addr, metadata.image_len) != ESP_OK) {
        ESP_LOGE(TAG, "Secure Boot fail");
        esp_restart();
    }

    ESP_LOGI(TAG, "Integrity verified");
}

// Select boot partition
const esp_partition_t* select_boot_partition(void) {
    const esp_partition_t *boot_part = NULL;

    // Check factory partition
    boot_part = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_FACTORY, NULL);
    if (boot_part != NULL) {
        ESP_LOGI(TAG, "Booting from factory partition");
        return boot_part;
    }

    // Check OTA partitions
    esp_ota_img_states_t ota_state;
    if (esp_ota_get_state_partition(esp_ota_get_running_partition(), &ota_state) == ESP_OK) {
        if (ota_state == ESP_OTA_IMG_VALID) {
            boot_part = esp_ota_get_running_partition();
            ESP_LOGI(TAG, "Booting from current OTA partition");
            return boot_part;
        }
    }

    // Try OTA_0
    boot_part = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
    if (boot_part != NULL) {
        ESP_LOGI(TAG, "Booting from OTA_0");
        return boot_part;
    }

    // Try OTA_1
    boot_part = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_1, NULL);
    if (boot_part != NULL) {
        ESP_LOGI(TAG, "Booting from OTA_1");
        return boot_part;
    }

    ESP_LOGE(TAG, "No valid boot partition found");
    esp_restart();
    return NULL;
}

// OTA support with rollback and verification - Fixed esp_image_load_metadata and arguments
void handle_ota_update(void) {
    esp_ota_handle_t update_handle = 0;
    const esp_partition_t *update_part = esp_ota_get_next_update_partition(NULL);

    if (update_part == NULL) {
        ESP_LOGE(TAG, "No OTA partition available");
        esp_restart();
        return;
    }

    // Begin OTA update
    esp_err_t err = esp_ota_begin(update_part, OTA_SIZE_UNKNOWN, &update_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA begin failed: %s", esp_err_to_name(err));
        esp_restart();
        return;
    }

    // Assume OTA image is written (e.g., via ESP-NOW or SPI)
    // For demo, skip actual write, verify integrity
    esp_image_metadata_t metadata;
    if (esp_image_load_metadata((const void *)update_part->address, &metadata) != ESP_OK) {
        ESP_LOGE(TAG, "OTA image load failed");
        esp_ota_abort(update_handle);
        esp_restart();
        return;
    }

    uint8_t digest[32];
    mbedtls_sha256((uint8_t*)metadata.start_addr, metadata.image_len, digest, 0);
    extern const uint8_t ota_expected_hash[32];  // Define OTA hash
    if (memcmp(digest, ota_expected_hash, 32) != 0) {
        ESP_LOGE(TAG, "OTA SHA256 fail");
        esp_ota_abort(update_handle);
        esp_restart();
        return;
    }

    // Secure Boot V2 verification - Fixed arguments
    if (esp_secure_boot_verify_signature(metadata.start_addr, metadata.image_len) != ESP_OK) {
        ESP_LOGE(TAG, "OTA Secure Boot fail");
        esp_ota_abort(update_handle);
        esp_restart();
        return;
    }

    // Mark OTA partition as active
    err = esp_ota_set_boot_partition(update_part);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA set boot partition failed: %s", esp_err_to_name(err));
        esp_ota_abort(update_handle);
        esp_restart();
        return;
    }

    // End OTA update
    err = esp_ota_end(update_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA end failed: %s", esp_err_to_name(err));
        esp_restart();
        return;
    }

    // Rollback support: Mark previous partition if boot fails
    const esp_partition_t *running_part = esp_ota_get_running_partition();
    esp_ota_mark_app_invalid_rollback_and_reboot(); // Rollback on next fail

    ESP_LOGI(TAG, "OTA update successful, rebooting");
    esp_restart();
}

// Fastboot emulation - Fixed to use esp_image_verify
void fastboot_emulation(void) {
    char cmd_buffer[256];
    int len = uart_read_bytes(UART_NUM_0, (uint8_t*)cmd_buffer, sizeof(cmd_buffer) - 1, 1000 / portTICK_PERIOD_MS);
    if (len <= 0) return;

    cmd_buffer[len] = '\0';
    ESP_LOGI(TAG, "Fastboot cmd: %s", cmd_buffer);

    if (strncmp(cmd_buffer, "fastboot unlock", 15) == 0) {
        // Unlock (disable secure boot or set flag in NVS)
        ESP_LOGI(TAG, "Unlock requested");
        // esp_secure_boot_permanently_disable(); // Dangerous, use with caution
        uart_write_bytes(UART_NUM_0, "OKAY", 4);
    } else if (strncmp(cmd_buffer, "fastboot flash", 14) == 0) {
        // Flash partition (e.g., OTA_0, OTA_1)
        const esp_partition_t *target_part = esp_ota_get_next_update_partition(NULL);
        esp_ota_handle_t handle;
        esp_ota_begin(target_part, OTA_SIZE_UNKNOWN, &handle);
        // Assume image data follows via UART (simplified)
        ESP_LOGI(TAG, "Flashing partition");
        esp_ota_end(handle);
        uart_write_bytes(UART_NUM_0, "OKAY", 4);
    } else if (strncmp(cmd_buffer, "fastboot boot", 13) == 0) {
        // Boot specific partition
        const esp_partition_t *boot_part = select_boot_partition();
        esp_image_metadata_t metadata;
        esp_image_verify(ESP_IMAGE_VERIFY, (esp_image_load_address_t)boot_part->address, &metadata);
        ESP_LOGI(TAG, "Booting partition");
        uart_write_bytes(UART_NUM_0, "OKAY", 4);
        esp_restart();
    } else if (strncmp(cmd_buffer, "fastboot reboot", 15) == 0) {
        ESP_LOGI(TAG, "Reboot requested");
        uart_write_bytes(UART_NUM_0, "OKAY", 4);
        esp_restart();
    } else {
        uart_write_bytes(UART_NUM_0, "FAILUnknown command", 18);
    }
}

// Filesystem mount - Fixed to use esp_vfs_fat_sdspi_mount for SD SPI
// Moved SPIFFS/SDFS mount to app, as bootloader should be minimal
// Here, just check if SD can be initialized (optional)
bool mount_filesystem(void) {
    // For SD via SPI
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = GPIO_NUM_9;
    slot_config.host_id = host.slot;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    sdmmc_card_t *card;
    esp_err_t ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SD card mount failed: %s", esp_err_to_name(ret));
        return false;
    }

    // For /system as SPIFFS (example, but prefer in app)
    esp_vfs_spiffs_conf_t spiffs_cfg = {
        .base_path = "/system",
        .partition_label = "system",
        .max_files = 5,
        .format_if_mount_failed = false
    };
    ret = esp_vfs_spiffs_register(&spiffs_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPIFFS /system mount failed: %s", esp_err_to_name(ret));
        return false;
    }

    spiffs_cfg.base_path = "/vendor";
    spiffs_cfg.partition_label = "vendor";
    ret = esp_vfs_spiffs_register(&spiffs_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPIFFS /vendor mount failed: %s", esp_err_to_name(ret));
        return false;
    }

    ESP_LOGI(TAG, "Filesystem mounted");
    return true;
}

// Load bootanimation.json (placeholder, actual LVGL in app)
void load_bootanimation(void) {
    // Bootanimation loading requires LVGL, which is too heavy for bootloader
    // Pass flag to app to load /boot/bootanimation.json
    ESP_LOGI(TAG, "Bootanimation load deferred to app");
}

// Boot mode selection - Fixed subtype to ESP_PARTITION_SUBTYPE_ANY for "recovery" label
void boot_mode_selection(void) {
    // Check for recovery mode (e.g., GPIO pin or UART command)
    bool recovery_mode = false; // Replace with actual trigger
    const esp_partition_t *recovery_part = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, "recovery");

    if (recovery_mode && recovery_part != NULL) {
        esp_image_metadata_t metadata;
        esp_image_verify(ESP_IMAGE_VERIFY, (esp_image_load_address_t)recovery_part->address, &metadata);
        ESP_LOGI(TAG, "Booting recovery partition at 0x%x", recovery_part->address);
        // Jump to recovery (handled by ESP-IDF bootloader)
        return;
    }

    // Try normal boot
    const esp_partition_t *boot_part = select_boot_partition();
    if (boot_part != NULL) {
        esp_image_metadata_t metadata;
        esp_image_verify(ESP_IMAGE_VERIFY, (esp_image_load_address_t)boot_part->address, &metadata);
        // Mount filesystem and defer bootanimation to app
        if (mount_filesystem()) {
            load_bootanimation();
            ESP_LOGI(TAG, "Booting partition at 0x%x", boot_part->address);
            // Jump to app (handled by ESP-IDF bootloader)
            return;
        }
    }

    // No valid system, enter fastboot
    ESP_LOGI(TAG, "No valid system, entering fastboot mode");
    while (true) {
        fastboot_emulation();
    }
}

// In bootloader_start.c
void bootloader_start(void) {
    custom_hardware_init();
    custom_integrity_check();

    // Check for fastboot mode (e.g., GPIO pin or UART input)
    bool fastboot_mode = false; // Replace with actual trigger (e.g., GPIO check)
    if (fastboot_mode) {
        ESP_LOGI(TAG, "Entering fastboot mode");
        while (true) {
            fastboot_emulation();
        }
    }

    // Check for OTA update request (e.g., via GPIO or command)
    bool ota_triggered = false; // Replace with actual trigger check
    if (ota_triggered) {
        handle_ota_update();
    }

    boot_mode_selection();
}