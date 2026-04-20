<div align="center">

<br/>

```
+------------------------------------------------------------------+
| ______          _                 __          __   _       _     |
||___  /         | |                \ \        / /  | |     | |    |
|   / / ___ _ __ | |__  _   _ _ __   \ \  /\  / /_ _| |_ ___| |__  |
|  / / / _ \ '_ \| '_ \| | | | '__|   \ \/  \/ / _` | __/ __| '_ \ |
| / /_|  __/ |_) | | | | |_| | |       \  /\  / (_| | || (__| | | ||
|/_____\___| .__/|_| |_|\__, |_|        \/  \/ \__,_|\__\___|_| |_||
|          | |           __/ |                                     |
|          |_|          |___/                                      |
+------------------------------------------------------------------+
```

# Zephyr Watch

### Кастомные смарт-часы на ESP32-S3 в стиле Google Pixel Watch

<br/>

![Rust](https://img.shields.io/badge/Rust-000000?style=for-the-badge&logo=rust&logoColor=white)
![ESP32-S3](https://img.shields.io/badge/ESP32--S3-E7352C?style=for-the-badge&logo=espressif&logoColor=white)
<!-- ![LVGL](https://img.shields.io/badge/LVGL-00B4D8?style=for-the-badge) -->
![MicroPython](https://img.shields.io/badge/MicroPython%20VM-2B5B84?style=for-the-badge&logo=python&logoColor=white)
![License](https://img.shields.io/badge/License-GPL--3.0-blue?style=for-the-badge)
![Status](https://img.shields.io/badge/Status-In%20Development-yellow?style=for-the-badge)

<br/>

> **DIY смарт-часы с Material You 3 интерфейсом, вдохновлённые дизайном Google Pixel Watch.**  
> Полностью с нуля. Полностью на Rust. Полностью открытый исходный код.

<br/>

---

</div>

## 📖 О проекте



## 🔧 Аппаратная часть

### Компоненты

| Компонент | Модель | Описание |
|-----------|--------|----------|
| Микроконтроллер | **ESP32-S3-Zero-N4R2** | 4MB Flash, 2MB PSRAM, WiFi, BLE 5.0 |
| Дисплей *(на выбор)* | **ST7789V3** | 240×280, SPI, цветной TFT(IPS) |
| Дисплей *(на выбор)* | **SSD1306** | 128×64, I2C, монохромный OLED |
| Тачскрин *(только со ST7789)* | **XPT2046** | Резистивный, SPI |
| Управление | **KY-040** | Ротационный энкодер с кнопкой |
| Батарея | **Li-ion** | 1000 мАч |
| Зарядка | **TP4056** | Магнитные pogo-pin контакты |
| Хранилище | **MicroSD** | До 32GB, FAT32 |
| Диод | **1N5819** | Защита от обратного тока (Schottky) |

### Схема подключения

```
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 ПИТАНИЕ
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

[USB] → [Mag+] → TP4056(IN+) → BAT+ → [Li-ion+] → 1N5819 → 5V [ESP32]
                                  ↓
[Mag−] → IN− → BAT− → [Li-ion−] → GND [ESP32]
                  ↓
                  OUT+ → 1N5819 → 5V [ESP32] (load-sharing)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

**TP4056**
- BAT[+] = батарея PIN "+"
- BAT[-] = батарея PIN "-"

- OUT[+] = ДИОД PIN " "
- OUT[-] = esp32s3_zero PIN "GND"

- IN[+] = магнит 1
- IN[1] = магнит 2

**Питания**
- 5V -> [
   ДИОД = PIN "|||"
 ]
- GND -> [
   TP4056 = PIN "-"
   ST7789 = PIN "GND"
   SSD1306 = PIN "GND"
   KY-040 = PIN "GND"
 ]
- 3V3 -> [
   ST7789 = PIN "VCC"
   SSD1306 = PIN "VCC"
 ]

**Дисплеи**

 ***ST7789v3***
- GND = GND
- VCC = 3V3
- GPIO[1] = BLK
- GPIO[2] = CS
- GPIO[3] = DC
- GPIO[4] = RES
- GPIO[5] = SDA
- GPIO[6] = SCL

 ***SSD1306***
- GND = GND
- VCC = 3V3
- GPIO[5] = SDA
- GPIO[6] = SCL

**Энкодер с кнопкой**
- GPIO[11] = PIN "RIGHT" # Энкодер
- GPIO[12] = PIN "LEFT" # Энкодер
- GPIO[13] = PIN "LEFT" # Кнопка

**SD модуль**
- GND = GND
- 3v3 = 3V3
- GPIO[7] = MISO
- GPIO[8] = CLK
- GPIO[9] = MOSI
- GPIO[10] = CS

**батарея**
- [-] = GND
- [T/ID] = PIN 14
- [+] = 5V

<br/>

---

## 📁 Файловая система (SD-карта)

> **Прошивка хранится и запускается с SD-карты.**  
> Bootloader на Flash неизменен — обновления делаются заменой файлов на карте.

```
/                              ← Корень SD-карты (FAT32)
│
├── firmware/                  ← Прошивка (MicroPython)
│   ├── main.py                ← Точка входа VM
│   ├── config.py              ← Тип дисплея, GPIO, параметры
│   ├── display/               ← Драйверы дисплеев
│   │   ├── st7789.py
│   │   └── ssd1306.py
│   ├── input/                 ← Драйверы ввода
│   │   ├── encoder.py
│   │   └── touch.py
│   ├── ui/                    ← Интерфейс Material You 3
│   │   ├── watchface.py
│   │   ├── menu.py
│   │   └── theme.py
│   └── power/                 ← Управление питанием
│       └── sleep.py
│
├── system/                    ← Системные ресурсы
│   ├── fonts/                 ← Шрифты для LVGL
│   └── lib/
│
├── watchfaces/                ← Пользовательские циферблаты
├── apps/                      ← Сторонние приложения (Python)
├── backup/                    ← Резервные копии настроек
└── logs/
    └── system.log             ← Системный лог (циклический буфер)
```

<br/>

---

## 🗂️ Структура репозитория

```
Android-For-ESP32S3/
│
├── .github/
│   └── workflows/             ← CI/CD: автосборка
│
├── Bootloader/                ← Загрузчик (Rust, прошивается 1 раз)
│   ├── src/
│   │   ├── main.rs            ← HAL init, монтирование SD, запуск VM
│   │   └── rtc.rs             ← Модуль RTC
│   └── Cargo.toml
│
├── Firmware/                  ← Прошивка (MicroPython, живёт на SD)
│   ├── main.py
│   ├── config.py
│   ├── display/
│   ├── input/
│   ├── ui/
│   └── power/
│
├── LICENSE                    ← GPL-3.0
└── README.md
```

<br/>

---

## 🎮 Управление

### Энкодер KY-040

| Действие | Результат |
|----------|-----------|
| Вращение влево/вправо | Навигация по меню |
| Одиночное нажатие | Выбор / подтверждение |
| Двойное нажатие | Назад |
| Долгое нажатие (2 секунды) + вращение | Меню выбора (домой, приложения, заблокировать, питание (вылючить, перезагрузить)) |

### Тачскрин XPT2046 *(только с ST7789)*

| Жест | Результат |
|------|-----------|
| Тап | Выбор элемента |
| Свайп влево/вправо | Переключение экранов |
| Свайп вниз | Шторка уведомлений |

<br/>

---

## 🔋 Энергопотребление

| Режим | Потребление | Описание |
|-------|-------------|----------|
| Активный (ST7789) | ~80–120 мА | Полный UI, WiFi активен |
| Активный (SSD1306) | ~20–35 мА | Монохромный UI |
| Режим ожидания | ~15–25 мА | Дисплей выкл., ядра активны |
| Deep Sleep | ~0.01–0.05 мА | Только RTC работает |
| **Срок работы (ST7789)** | **~1-2 дня** | 1000 мАч |
| **Срок работы (SSD1306)** | **~3 дня** | 1000 мАч |

> ⚡ Частота CPU снижена до **80 MHz** для уменьшения тепловыделения.

<br/>

---

## 🚀 Быстрый старт

### Требования

- [Rust](https://rustup.rs/) (latest stable)
- [espup](https://github.com/esp-rs/espup)
- [cargo-espflash](https://github.com/esp-rs/espflash)

### Установка toolchain

```bash
cargo install espup
espup install
source $HOME/export-esp.sh
```

### Сборка и прошивка

```bash
git clone --recurse-submodules https://github.com/DenBroLiik/Android-For-ESP32S3.git
cd Android-For-ESP32S3/Bootloader

cargo +esp build --release
cargo espflash flash --release --monitor
```

<br/>

---

## 🗺️ Roadmap

### ✅ Готово
- [x] Bootloader на Rust (HAL init, монтирование SD)
- [x] Модуль RTC (хранение времени после Deep Sleep)
- [x] Понижение частоты CPU до 80 MHz
- [x] Видеопамять за счёт SRAM
- [x] Драйвер распределения памяти SRAM + PSRAM

### 🔄 В разработке
- [ ] Инициализация компонентов — унифицированный init для дисплея, энкодера, SD
- [ ] Световая индикация — статусный LED: загрузка, ошибка, зарядка
- [ ] Подключение MicroPython VM — запуск прошивки с SD через интерпретатор
- [ ] Разметка карты памяти — стандартизированная структура FAT32

### 📋 Планируется
- [ ] Драйвер ST7789 (цветной дисплей)
- [ ] Драйвер SSD1306 (OLED)
- [ ] Material You 3 UI через LVGL
- [ ] Получение уведомлений через ESP-NOW
- [ ] OTA-обновления по WiFi
- [ ] SDK для watchfaces на Python
- [ ] Bluetooth синхронизация с Android

<br/>

---

## 📜 Changelog

### [19.02.2026]

**Changed**
- Понижение частоты процессора с MAX до **80 MHz** для снижения температуры чипа при длительной работе.

**Added**
- Видеопамять за счёт SRAM — расширение буфера кадра для рендеринга UI.
- Драйвер распределения оперативной памяти **SRAM + PSRAM** — динамическое управление доступными пулами памяти.

---

### [15.02.2026]

**Added**
- Базовый модуль **RTC** для загрузчика — хранение и восстановление системного времени после Deep Sleep и перезагрузки.

---

### [12.02.2026]

**Changed**
- Пересоздание проекта: **C++ → Rust** — полный переход на Rust для улучшения безопасности памяти, производительности и удобства сопровождения кода.

---

<br/>

## 📄 Лицензия

Этот проект распространяется под лицензией **GPL-3.0**.  
Подробнее см. файл [LICENSE](./LICENSE).

<br/>

---

<div align="center">

**Zephyr Watch** — Open Source Project

Made with ❤️ and Rust · [ESP32-S3](https://www.espressif.com/) · [LVGL](https://lvgl.io/) · [MicroPython](https://micropython.org/)

</div>