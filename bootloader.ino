/* bootloader_full_lvgl9_fastboot.ino
 *
 * Полная версия, адаптированная под LVGL v9 API.
 *
 * Требования:
 *  - Arduino ESP32-S3 core (3.x)
 *  - Adafruit TinyUSB (включающий VENDOR interface) или tinyusb-dev пакет
 *  - LVGL v9.x установлен через Library Manager (обязательно!)
 *
 * TODO:
 *  - Подставьте реальный ST7789 init sequence в init_st7789().
 *  - При необходимости скорректируйте размеры draw-buffer'ов под вашу RAM.
 *  - SSD1306: реализовать преобразование RGB565 -> 1-bit pages (для монохромных дисплеев).
 */

#define CFG_TUSB_RHPORT0_MODE (OPT_MODE_DEVICE)
#define CFG_TUSB_RHPORT1_MODE (0)

#include <Arduino.h>
#include <esp_system.h>
#include <esp_flash.h>
#include <esp_log.h>
#include <esp_partition.h>
#include <esp_ota_ops.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <driver/i2c.h>
#include <driver/ledc.h>
#include <lvgl.h>
#include <Adafruit_TinyUSB.h>
#include "tusb.h"

static const char *TAG = "SBL";

/* ---------- Pins ---------- */
// ST7789 SPI pins (пример — поменяй под свою плату при необходимости)
#define PIN_SCLK 18
#define PIN_MOSI 23
#define PIN_DC   17
#define PIN_CS   16
#define PIN_RES  5
#define PIN_TOUCH_CS 15

// SSD1306 I2C pins
#define PIN_SDA 8
#define PIN_SCL 9
#define I2C_PORT I2C_NUM_0
#define SSD1306_ADDR 0x3C

// Encoder pins
#define PIN_CLK 4
#define PIN_DT 2
#define PIN_SW 1

// RGB LED (LEDC)
#define PIN_LED_R 25
#define PIN_LED_G 26
#define PIN_LED_B 27

/* ---------- Fastboot constants ---------- */
#define FASTBOOT_CMD_SIZE 512
#define FASTBOOT_RESP_OKAY "OKAY"
#define FASTBOOT_RESP_FAIL "FAIL"
#define FASTBOOT_RESP_DATA "DATA"

/* ---------- Compatibility / safety ---------- */
/* Some SDK variants may not define LEDC_HIGH_SPEED_MODE macro */
#ifndef LEDC_HIGH_SPEED_MODE
#define LEDC_HIGH_SPEED_MODE LEDC_LOW_SPEED_MODE
#endif

/* Detect LVGL major version (support в основном LVGL v9) */
#if defined(LVGL_VERSION_MAJOR)
  #define _LV_MAJOR LVGL_VERSION_MAJOR
#elif defined(LV_VERSION_MAJOR)
  #define _LV_MAJOR LV_VERSION_MAJOR
#else
  #define _LV_MAJOR 9
#endif

/* ---------- Globals ---------- */
static bool st7789_detected = false;
static bool ssd1306_detected = false;
static spi_device_handle_t st7789_spi = NULL; // assigned in init_st7789()
Adafruit_USBD_CDC usb_serial;

/* ---------- Forward declarations (LVGL v9 flush signatures) ---------- */
#if _LV_MAJOR >= 9
static void st7789_flush_cb(lv_display_t *disp, const lv_area_t *area, const lv_color_t *color_p);
static void ssd1306_flush_cb(lv_display_t *disp, const lv_area_t *area, const lv_color_t *color_p);
#else
/* if somehow compiling older LVGL, you'd need other signatures — but this file targets v9 */
#endif

/* ---------- TinyUSB vendor helpers ---------- */
static int vendor_read_cmd_blocking(char *buf, int maxlen, int timeout_ms) {
  int total = 0;
  TickType_t start = xTaskGetTickCount();
  while (total < maxlen) {
    if (tud_vendor_available()) {
      int r = tud_vendor_read((uint8_t*)(buf + total), maxlen - total);
      if (r > 0) {
        total += r;
        if (memchr(buf, '\n', total) || memchr(buf, '\r', total)) break;
      }
    }
    if ((xTaskGetTickCount() - start) * portTICK_PERIOD_MS > timeout_ms) break;
    vTaskDelay(pdMS_TO_TICKS(5));
  }
  if (total > 0) {
    while (total > 0 && (buf[total-1] == '\n' || buf[total-1] == '\r')) total--;
    buf[total] = 0;
  }
  return total;
}

static void vendor_write_resp(const char *s) {
  if (!s) return;
  tud_vendor_write((const uint8_t*)s, strlen(s));
  tud_vendor_write_flush();
}

/* ---------- ST7789 init & LVGL flush (DMA-capable) ---------- */

bool init_st7789(void) {
  spi_bus_config_t buscfg = {};
  buscfg.mosi_io_num = PIN_MOSI;
  buscfg.miso_io_num = -1;
  buscfg.sclk_io_num = PIN_SCLK;
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;

  esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(ret));
    return false;
  }

  spi_device_interface_config_t devcfg = {};
  devcfg.clock_speed_hz = 40 * 1000 * 1000; // попробуй 40MHz, при проблемах уменьшить
  devcfg.mode = 0;
  devcfg.spics_io_num = PIN_CS;
  devcfg.queue_size = 7;
  devcfg.flags = SPI_DEVICE_NO_DUMMY;

  ret = spi_bus_add_device(SPI2_HOST, &devcfg, &st7789_spi);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(ret));
    return false;
  }

  // Configure DC/RES pins
  gpio_set_direction((gpio_num_t)PIN_DC, GPIO_MODE_OUTPUT);
  gpio_set_direction((gpio_num_t)PIN_RES, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t)PIN_RES, 0);
  vTaskDelay(pdMS_TO_TICKS(10));
  gpio_set_level((gpio_num_t)PIN_RES, 1);
  vTaskDelay(pdMS_TO_TICKS(120));

  // TODO: сюда поставить реальный инициализационный набор команд ST7789 (команды + аргументы).
  ESP_LOGI(TAG, "ST7789 spi device created (handle stored)");
  return true;
}

/* LVGL v9 flush callback: st7789 */
#if _LV_MAJOR >= 9
static void st7789_flush_cb(lv_display_t *disp, const lv_area_t *area, const lv_color_t *color_p) {
  if (!st7789_spi) {
    lv_display_flush_ready(disp);
    return;
  }

  int32_t x1 = area->x1;
  int32_t y1 = area->y1;
  int32_t x2 = area->x2;
  int32_t y2 = area->y2;
  int32_t w = (x2 - x1 + 1);
  int32_t h = (y2 - y1 + 1);
  size_t px_cnt = (size_t)w * (size_t)h;
  size_t bytes_total = px_cnt * sizeof(lv_color_t);
  const uint8_t *data_ptr = (const uint8_t*)color_p;

  // CASET/RASET/RAMWR
  uint8_t col_cmd[] = {
    0x2A,
    (uint8_t)((x1 >> 8) & 0xFF), (uint8_t)(x1 & 0xFF),
    (uint8_t)((x2 >> 8) & 0xFF), (uint8_t)(x2 & 0xFF),
    0x2B,
    (uint8_t)((y1 >> 8) & 0xFF), (uint8_t)(y1 & 0xFF),
    (uint8_t)((y2 >> 8) & 0xFF), (uint8_t)(y2 & 0xFF),
    0x2C
  };

  spi_transaction_t t = {};
  gpio_set_level((gpio_num_t)PIN_DC, 0);
  t.length = 8 * (sizeof(col_cmd));
  t.tx_buffer = col_cmd;
  esp_err_t r = spi_device_transmit(st7789_spi, &t);
  if (r != ESP_OK) {
    ESP_LOGW(TAG, "st7789: command tx failed %s", esp_err_to_name(r));
  }

  size_t sent = 0;
  gpio_set_level((gpio_num_t)PIN_DC, 1); // data mode
  while (sent < bytes_total) {
    size_t chunk = bytes_total - sent;
    if (chunk > 4096) chunk = 4096;
    spi_transaction_t tx = {};
    tx.length = 8 * chunk;
    tx.tx_buffer = data_ptr + sent;
    esp_err_t rr = spi_device_transmit(st7789_spi, &tx);
    if (rr != ESP_OK) {
      ESP_LOGE(TAG, "st7789 data tx err %s", esp_err_to_name(rr));
      break;
    }
    sent += chunk;
  }

  lv_display_flush_ready(disp);
}
#endif

/* ---------- SSD1306 stub (I2C) ---------- */
bool init_ssd1306(void) {
  i2c_config_t conf = {};
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = PIN_SDA;
  conf.scl_io_num = PIN_SCL;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = 400000;
  esp_err_t r = i2c_param_config(I2C_PORT, &conf);
  if (r != ESP_OK) {
    ESP_LOGE(TAG, "i2c_param_config failed");
    return false;
  }
  r = i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
  if (r != ESP_OK) {
    ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(r));
    return false;
  }

  // quick probe
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (SSD1306_ADDR << 1) | I2C_MASTER_WRITE, true);
  i2c_master_stop(cmd);
  r = i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  if (r != ESP_OK) {
    ESP_LOGW(TAG, "SSD1306 not present: %s", esp_err_to_name(r));
    return false;
  }
  ESP_LOGI(TAG, "SSD1306 detected");
  return true;
}

#if _LV_MAJOR >= 9
static void ssd1306_flush_cb(lv_display_t *disp, const lv_area_t *area, const lv_color_t *color_p) {
  // TODO: преобразование RGB565 -> 1-bit страницы SSD1306 и запись через I2C.
  // Пока сообщаем LVGL, что flush завершён.
  lv_display_flush_ready(disp);
}
#endif

/* ---------- LVGL display registration (v9) ---------- */

static void lvgl_tick_task(void *arg) {
  (void)arg;
  while (1) {
    lv_timer_handler();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void init_lvgl_display(void) {
  lv_init();

  st7789_detected = init_st7789();
  ssd1306_detected = init_ssd1306();

  if (!st7789_detected && !ssd1306_detected) {
    ESP_LOGW(TAG, "No display found, LVGL will still run but no actual flush.");
  }

#if _LV_MAJOR >= 9
  // choose resolution based on detected display
  int hor = st7789_detected ? 240 : (ssd1306_detected ? 128 : 128);
  int ver = st7789_detected ? 240 : (ssd1306_detected ? 64 : 64);

  lv_display_t *disp = lv_display_create(hor, ver);
  if (!disp) {
    ESP_LOGE(TAG, "lv_display_create failed");
    return;
  }

  // create draw buffer(s): make a small chunk to save RAM; LVGL expects buffer size in bytes.
  // For color ST7789 (RGB565) sizeof(lv_color_t) == 2 normally.
  // Reserve a buffer that fits a few lines: e.g., hor * 40 pixels.
  static lv_color_t draw_buf1[240 * 40]; // ~19 KB for 240x40 (RGB565)
  void *buf1 = draw_buf1;
  void *buf2 = NULL; // single-buffer mode; set second buffer if you have RAM and want double buffering
  uint32_t buf_size_bytes = sizeof(draw_buf1);

  // Some LVGL builds require +8 bytes in buffer for palette; docs recommend adding 8 bytes.
  // We allocate as lv_color_t[], so pass raw pointer and size in bytes.
  lv_display_set_buffers(disp, buf1, buf2, buf_size_bytes, LV_DISPLAY_RENDER_MODE_PARTIAL);

  // set color format for ST7789 (RGB565), for SSD1306 you'd handle differently
  lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565);

  // set flush callback depending on panel
  if (st7789_detected) {
    lv_display_set_flush_cb(disp, st7789_flush_cb);
  } else {
    lv_display_set_flush_cb(disp, ssd1306_flush_cb);
  }

  // make this display the default
  lv_display_set_default(disp);

  // create a splash screen (LVGL v9 style uses screen objects)
  lv_obj_t *scr = lv_obj_create(NULL); // new screen
  lv_obj_set_size(scr, hor, ver);
  lv_obj_set_style_pad_all(scr, 0, 0);

  // Background and labels depending on color/mono
  if (st7789_detected) {
    // green background
    lv_obj_set_style_bg_color(scr, lv_color_make(0, 200, 0), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    lv_obj_t *label = lv_label_create(scr);
    lv_label_set_text(label, "Zephyr");
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_14, 0); // commonly present, fallback if missing
    lv_obj_center(label);

    lv_obj_t *lock = lv_label_create(scr);
    lv_label_set_text(lock, "🔓"); // stub: replace with actual efuse read
    lv_obj_align(lock, LV_ALIGN_TOP_MID, 0, 6);

    lv_obj_t *pb = lv_label_create(scr);
    lv_label_set_text(pb, "Powered by Android");
    lv_obj_align(pb, LV_ALIGN_BOTTOM_MID, 0, -6);
  } else {
    // monochrome-like background (light gray)
    lv_obj_set_style_bg_color(scr, lv_color_make(180, 180, 180), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    lv_obj_t *label = lv_label_create(scr);
    lv_label_set_text(label, "Zephyr");
    lv_obj_set_style_text_color(label, lv_color_white(), 0);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_14, 0);
    lv_obj_center(label);

    lv_obj_t *lock = lv_label_create(scr);
    lv_label_set_text(lock, "🔓");
    lv_obj_align(lock, LV_ALIGN_TOP_MID, 0, 4);

    lv_obj_t *pb = lv_label_create(scr);
    lv_label_set_text(pb, "Powered by Android");
    lv_obj_align(pb, LV_ALIGN_BOTTOM_MID, 0, -4);
  }

  // load our screen to the display
  lv_display_load_scr(scr);

  // start LVGL tick task
  xTaskCreatePinnedToCore(lvgl_tick_task, "lvgl", 4096, NULL, 5, NULL, tskNO_AFFINITY);

#else
  // If somehow compiled with LVGL <9 (not intended), do nothing here.
  ESP_LOGE(TAG, "This file is written for LVGL v9; found older LVGL version.");
#endif
}

/* ---------- Encoder init ---------- */
void init_encoder(void) {
  gpio_config_t io_conf = {
    .pin_bit_mask = (1ULL << PIN_CLK) | (1ULL << PIN_DT) | (1ULL << PIN_SW),
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
  };
  gpio_config(&io_conf);
}

/* ---------- LED manager (LEDC) ---------- */

enum led_state_e {
  LED_STATE_OFF = 0,
  LED_STATE_FASTBOOT,   // red breathing
  LED_STATE_FLASHING,   // fast blink while flashing
  LED_STATE_FLASH_OK,   // green for 3s then revert
  LED_STATE_FLASH_FAIL, // yellow for 3s then revert
  LED_STATE_NORMAL_BOOT
};

static volatile led_state_e led_state = LED_STATE_OFF;
static led_state_e led_prev_state = LED_STATE_OFF;

#define LEDC_TIMER       LEDC_TIMER_0
#define LEDC_MODE_LOCAL  LEDC_HIGH_SPEED_MODE
#define LEDC_DUTY_RES    LEDC_TIMER_13_BIT
#define LEDC_FREQUENCY   5000

static void led_init(void) {
  ledc_timer_config_t ledc_timer = {};
  ledc_timer.speed_mode = LEDC_MODE_LOCAL;
  ledc_timer.timer_num = LEDC_TIMER;
  ledc_timer.duty_resolution = LEDC_DUTY_RES;
  ledc_timer.freq_hz = LEDC_FREQUENCY;
  ledc_timer.clk_cfg = LEDC_AUTO_CLK;
  ledc_timer_config(&ledc_timer);

  ledc_channel_config_t ch = {};
  ch.gpio_num = PIN_LED_R;
  ch.speed_mode = LEDC_MODE_LOCAL;
  ch.channel = LEDC_CHANNEL_0;
  ch.intr_type = LEDC_INTR_DISABLE;
  ch.timer_sel = LEDC_TIMER;
  ch.duty = 0;
  ch.hpoint = 0;
  ledc_channel_config(&ch);

  ch.gpio_num = PIN_LED_G;
  ch.channel = LEDC_CHANNEL_1;
  ledc_channel_config(&ch);

  ch.gpio_num = PIN_LED_B;
  ch.channel = LEDC_CHANNEL_2;
  ledc_channel_config(&ch);
}

static void led_set_rgb_raw(uint32_t r, uint32_t g, uint32_t b) {
  ledc_set_duty(LEDC_MODE_LOCAL, LEDC_CHANNEL_0, r);
  ledc_update_duty(LEDC_MODE_LOCAL, LEDC_CHANNEL_0);
  ledc_set_duty(LEDC_MODE_LOCAL, LEDC_CHANNEL_1, g);
  ledc_update_duty(LEDC_MODE_LOCAL, LEDC_CHANNEL_1);
  ledc_set_duty(LEDC_MODE_LOCAL, LEDC_CHANNEL_2, b);
  ledc_update_duty(LEDC_MODE_LOCAL, LEDC_CHANNEL_2);
}

static void led_set_state(led_state_e s) {
  if (s == LED_STATE_FLASH_OK || s == LED_STATE_FLASH_FAIL) {
    led_prev_state = led_state;
    led_state = s;
  } else {
    led_state = s;
  }
}

static void led_task(void *arg) {
  (void)arg;
  const uint32_t duty_max = (1 << LEDC_DUTY_RES) - 1;
  TickType_t last_wake = xTaskGetTickCount();

  for (;;) {
    TickType_t now = xTaskGetTickCount();
    uint32_t ms = (now * portTICK_PERIOD_MS);

    switch (led_state) {
      case LED_STATE_FASTBOOT: {
        uint32_t period = 1000;
        uint32_t t = ms % period;
        uint32_t val = (t < period/2) ? (t * 2 * duty_max / period) : ((period - t) * 2 * duty_max / period);
        led_set_rgb_raw(val, 0, 0);
        break;
      }
      case LED_STATE_FLASHING: {
        uint32_t t = ms % 250;
        if (t < 125) led_set_rgb_raw(duty_max, 0, 0);
        else led_set_rgb_raw(0, 0, 0);
        break;
      }
      case LED_STATE_FLASH_OK: {
        led_set_rgb_raw(0, duty_max, 0);
        vTaskDelay(pdMS_TO_TICKS(3000));
        led_state = led_prev_state ? led_prev_state : LED_STATE_FASTBOOT;
        break;
      }
      case LED_STATE_FLASH_FAIL: {
        led_set_rgb_raw(duty_max, duty_max, 0);
        vTaskDelay(pdMS_TO_TICKS(3000));
        led_state = led_prev_state ? led_prev_state : LED_STATE_FASTBOOT;
        break;
      }
      case LED_STATE_NORMAL_BOOT: {
        uint32_t period = 3000;
        uint32_t t = ms % period;
        float phase = (float)t / (float)period;
        uint32_t r = 0, g = 0, b = 0;
        if (phase < 0.3333f) {
          float p = phase / 0.3333f;
          g = duty_max;
          b = (uint32_t)(p * duty_max);
        } else if (phase < 0.6666f) {
          float p = (phase - 0.3333f) / 0.3333f;
          g = (uint32_t)((1.0f - p) * duty_max);
          b = duty_max;
        } else {
          float p = (phase - 0.6666f) / 0.3334f;
          g = (uint32_t)(p * duty_max);
          b = (uint32_t)((1.0f - p) * duty_max);
        }
        led_set_rgb_raw(r, g, b);
        break;
      }
      case LED_STATE_OFF:
      default:
        led_set_rgb_raw(0, 0, 0);
        break;
    }

    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));
  }
}

/* ---------- Fastboot vendor loop ---------- */

void fastboot_vendor_loop(void *arg) {
  (void)arg;
  char cmd[FASTBOOT_CMD_SIZE + 1];
  uint8_t data_buf[2048];

  for (;;) {
    int len = vendor_read_cmd_blocking(cmd, FASTBOOT_CMD_SIZE, 1000);
    if (len <= 0) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    ESP_LOGI(TAG, "fastboot cmd: %s", cmd);

    if (strncmp(cmd, "getvar:", 7) == 0) {
      char resp[64];
      snprintf(resp, sizeof(resp), FASTBOOT_RESP_OKAY "%s", "serial1234");
      vendor_write_resp(resp);
    } else if (strncmp(cmd, "erase:", 6) == 0) {
      const char *partname = cmd + 6;
      const esp_partition_t *part = esp_partition_find_first(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, partname);
      if (!part) { vendor_write_resp(FASTBOOT_RESP_FAIL); continue; }
      esp_err_t r = esp_partition_erase_range(part, 0, part->size);
      vendor_write_resp(r == ESP_OK ? FASTBOOT_RESP_OKAY : FASTBOOT_RESP_FAIL);
    } else if (strncmp(cmd, "flash:", 6) == 0) {
      const char *partname = cmd + 6;
      const esp_partition_t *part = esp_partition_find_first(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, partname);
      if (!part) { vendor_write_resp(FASTBOOT_RESP_FAIL); continue; }

      int hlen = vendor_read_cmd_blocking(cmd, FASTBOOT_CMD_SIZE, 5000);
      if (hlen <= 0 || strncmp(cmd, "DATA", 4) != 0) { vendor_write_resp(FASTBOOT_RESP_FAIL); continue; }
      uint32_t expected = (uint32_t) strtoul(cmd + 4, NULL, 10);
      vendor_write_resp(FASTBOOT_RESP_OKAY);

      uint32_t received = 0;
      uint32_t write_off = 0;
      led_set_state(LED_STATE_FLASHING);

      while (received < expected) {
        int r = tud_vendor_read(data_buf, sizeof(data_buf));
        if (r <= 0) {
          vTaskDelay(pdMS_TO_TICKS(5));
          continue;
        }
        esp_err_t w = esp_partition_write(part, write_off, data_buf, r);
        if (w != ESP_OK) {
          ESP_LOGE(TAG, "partition write err %s", esp_err_to_name(w));
          vendor_write_resp(FASTBOOT_RESP_FAIL);
          led_set_state(LED_STATE_FLASH_FAIL);
          break;
        }
        write_off += r;
        received += r;
      }
      if (received == expected) {
        vendor_write_resp(FASTBOOT_RESP_OKAY);
        led_set_state(LED_STATE_FLASH_OK);
      }
    } else if (strncmp(cmd, "set_active:", 11) == 0) {
      const char *slot = cmd + 11;
      const esp_partition_t *part = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_ANY, slot);
      if (!part) { vendor_write_resp(FASTBOOT_RESP_FAIL); continue; }
      esp_err_t rr = esp_ota_set_boot_partition(part);
      vendor_write_resp(rr == ESP_OK ? FASTBOOT_RESP_OKAY : FASTBOOT_RESP_FAIL);
    } else if (strncmp(cmd, "reboot", 6) == 0) {
      vendor_write_resp(FASTBOOT_RESP_OKAY);
      esp_restart();
    } else if (strncmp(cmd, "oem unlock", 10) == 0) {
      vendor_write_resp(FASTBOOT_RESP_OKAY);
      ESP_LOGW(TAG, "oem unlock requested (NOT IMPLEMENTED: use with caution)");
    } else if (strncmp(cmd, "continue", 8) == 0) {
      vendor_write_resp(FASTBOOT_RESP_OKAY);
      break;
    } else {
      vendor_write_resp(FASTBOOT_RESP_FAIL);
    }
  }

  vTaskDelete(NULL);
}

/* ---------- Boot logic ---------- */

bool check_firmware_signature(const esp_partition_t *p) {
  (void)p;
  // TODO: implement real signature/AVB check
  return true;
}

void boot_main_core(void) {
  ESP_LOGI(TAG, "Starting SBL core...");

  // init LED and LED task early so we can show states right away
  led_init();
  xTaskCreatePinnedToCore(led_task, "led", 2048, NULL, 6, NULL, tskNO_AFFINITY);

  init_lvgl_display();
  init_encoder();

  // SW pin
  gpio_set_direction((gpio_num_t)PIN_SW, GPIO_MODE_INPUT);
  gpio_pullup_en((gpio_num_t)PIN_SW);

  if (gpio_get_level((gpio_num_t)PIN_SW) == 0) {
    ESP_LOGI(TAG, "Recovery button pressed => entering Fastboot (Vendor USB)");
    init_tinyusb();

    led_set_state(LED_STATE_FASTBOOT);

    if (lv_display_get_default()) {
      lv_obj_t *label = lv_label_create(lv_screen_active());
      lv_label_set_text(label, "Recovery Mode: Connect USB");
      lv_obj_center(label);
    }

    xTaskCreatePinnedToCore(fastboot_vendor_loop, "fastboot", 8192, NULL, 5, NULL, tskNO_AFFINITY);
    while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    return;
  }

  led_set_state(LED_STATE_NORMAL_BOOT);

  const esp_partition_t *app_partition = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
  if (!app_partition) {
    ESP_LOGE(TAG, "No app partition found");
    esp_restart();
  }

  if (!check_firmware_signature(app_partition)) {
    ESP_LOGE(TAG, "Firmware signature invalid");
    esp_restart();
  }

  esp_err_t err = esp_ota_set_boot_partition(app_partition);
  if (err == ESP_OK) {
    ESP_LOGI(TAG, "Boot partition set, restarting...");
    esp_restart();
  } else {
    ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
    esp_restart();
  }
}

/* ---------- TinyUSB init ---------- */
void init_tinyusb(void) {
  TinyUSBDevice.begin();
  ESP_LOGI(TAG, "TinyUSB started");
  usb_serial.begin(115200);
}

/* ---------- Arduino entrypoints ---------- */
void setup() {
  Serial.begin(115200);
  delay(200);
  ESP_LOGI(TAG, "bootloader starting...");
  boot_main_core();
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
