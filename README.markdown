# Zephyr Watch 1

<div align="center">

**DIY смарт-часы в стиле Google Pixel Watch 3 на Rust**

[![Rust](https://img.shields.io/badge/Rust-000000?style=for-the-badge&logo=rust&logoColor=white)](https://www.rust-lang.org/)
[![ESP32-S3](https://img.shields.io/badge/ESP32--S3-E7352C?style=for-the-badge&logo=espressif&logoColor=white)](https://www.espressif.com/)
[![License](https://img.shields.io/badge/License-MIT-blue?style=for-the-badge)](LICENSE)

</div>

## О проекте

**Zephyr Watch 1** — это кастомные смарт-часы, созданные с нуля. Проект вдохновлён дизайном Google Pixel Watch 3 и работает на базе микроконтроллера **ESP32-S3-Zero-N4R2**.

### Ключевые особенности

- **Интерфейс Material You 3** — современный адаптивный дизайн с rounded cards и плавными анимациями
- **Поддержка двух дисплеев** — цветной ST7789 (240×240) и монохромный SSD1306 (128×64)
- **Гибкое управление** — ротационный энкодер с трещоткой + сенсорный экран (XPT2046)
- **Магнитная зарядка** — удобная зарядка через pogo-pin контакты
- **Долгое время работы** — энергоэффективный режим deep sleep (~0.01-0.05 мАч)
- **Расширяемое хранилище** — поддержка SD-карт до 32GB

---

## Аппаратная часть

### Компоненты

| Компонент | Спецификация | Примечание |
|-----------|--------------|------------|
| Микроконтроллер | ESP32-S3-Zero-N4R2 | 4MB Flash, 2MB PSRAM, WiFi/BLE |
| Основной дисплей | ST7789 | 240×240, SPI, цветной |
| Дополнительный дисплей | SSD1306 | 128×64, I2C, монохромный |
| Сенсор | XPT2046 | Резистивный тачскрин |
| Управление | KY-040 | Энкодер с кнопкой и трещоткой |
| Батарея | Li-ion | 400 мАч |
| Зарядка | TP4056 | С магнитными контактами |
| Хранилище | MicroSD | До 32GB (FAT32) |

### Схема подключения

```
Питание:
[USB Charger] → [Mag1:V+] → TP4056(IN+) → BAT+ → [Li-ion +] → 1N5819 → 5V [ESP32]
                                            ↓
[Mag2:GND] → IN- → BAT- → [Li-ion -] → GND [ESP32]
                ↓
                OUT+ → 1N5819 → 5V [ESP32] (load-sharing)

Дисплей ST7789 (SPI):
GPIO18 → SCLK
GPIO23 → MOSI
GPIO17 → DC
GPIO16 → CS
GPIO5  → RES

Сенсор XPT2046:
GPIO15 → TOUCH_CS

Энкодер KY-040:
GPIO4 → CLK
GPIO2 → DT
GPIO1 → SW

SD-карта (SPI):
GPIO9  → CS
GPIO11 → MOSI
GPIO12 → MISO
GPIO10 → SCK
```

---

## Программная часть

### Стек технологий

- **Язык**: [Rust](https://www.rust-lang.org/) (embedded-hal, esp-idf-hal)
- **Фреймворк**: [esp-idf-sys](https://github.com/esp-rs/esp-idf-sys) / [esp-idf-hal](https://github.com/esp-rs/esp-idf-hal)
- **UI**: [LVGL](https://lvgl.io/) v9+ (через [lvgl-rs](https://github.com/lvgl/lvgl-rs))
- **Сборка**: [PlatformIO](https://platformio.org/) или [cargo-espflash](https://github.com/esp-rs/espflash)

### Файловая система

Эмуляция структуры Wear OS 6:

```
/
├── system/          (~340KB)  UI, шрифты, watchfaces, конфигурации
│   ├── fonts/
│   ├── app/
│   └── lib/
├── boot/            (64KB)    Анимация загрузки, OTA-метаданные
├── tmp/             (~4KB)    Логи системы
├── vendor/          (~200KB)  Драйверы hardware
├── dev/                       Устройства
└── sdcard/          (32GB)    Пользовательские данные
```

### Структура проекта

```
.
├── bootloader/      # Загрузчик
├── firmware/        # Исходный код на Rust
│   ├── src/
│   │   ├── main.rs
│   │   ├── display/     # Драйверы дисплеев
│   │   ├── ui/          # Material You 3 UI
│   │   ├── input/       # Энкодер, тачскрин
│   │   ├── fs/          # Файловая система
│   │   └── power/       # Управление питанием
│   └── Cargo.toml
├── partitions.csv   # Таблица разделов
└── README.markdown  # Этот файл
```

---

## Быстрый старт

### Требования

- [Rust](https://rustup.rs/) (latest stable)
- [ESP-IDF](https://docs.espressif.com/projects/esp-idf/) v5.3+
- [cargo-espflash](https://github.com/esp-rs/espflash)
- [espup](https://github.com/esp-rs/espup) (для настройки toolchain)

### Установка toolchain

```bash
# Установка espup
cargo install espup

# Установка ESP-IDF toolchain
espup install

# Активация окружения
source $HOME/export-esp.sh
```

### Сборка и прошивка

```bash
# Клонирование репозитория
git clone https://github.com/yourusername/Zephyr-Watch-1.git
cd Zephyr-Watch-1/firmware

# Сборка
cargo build --release

# Прошивка (автоопределение порта)
cargo espflash flash --release --monitor

# Или с указанием порта
cargo espflash flash --release --port /dev/ttyUSB0 --monitor
```

---

## Функциональность

### Управление

- **Энкодер**:
  - Вращение — навигация по меню
  - Нажатие — выбор/подтверждение
  - Двойное нажатие — возврат назад

- **Сенсорный экран** (ST7789):
  - Тап — выбор элемента
  - Свайп — прокрутка/переключение экранов

### Экраны

- **Watchface** — главный циферблат с временем, датой, уведомлениями
- **Меню приложений** — список установленных приложений
- **Настройки** — яркость, звук, WiFi, Bluetooth
- **Уведомления** — входящие уведомления с телефона

### Энергосбережение

- Автоматический переход в deep sleep через 10 сек бездействия
- Пробуждение по:
  - Нажатию кнопки энкодера
  - Вращению энкодера
  - Касанию экрана

---

## Roadmap

- [x] Базовая инициализация hardware
- [x] Поддержка ST7789 и SSD1306
- [x] Интерфейс Material You 3
- [x] Управление энкодером
- [ ] Поддержка XPT2046 (тачскрин)
- [ ] Watchfaces с конфигурацией
- [ ] Уведомления через ESP-NOW
- [ ] OTA-обновления
- [ ] SDK для сторонних приложений

---

## Лицензия

Этот проект распространяется под лицензией MIT. См. [LICENSE](LICENSE) для подробностей.

---

<div align="center">

**Made with ❤️ and Rust**

</div>
