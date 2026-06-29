// xtensa_lx7_core_temperature.rs
// Обёртка над внутренним датчиком температуры ESP32-S3 (TSENS)
// no_std совместимо

use core::sync::atomic::{AtomicI32, Ordering};

// Порог тротлинга (в десятых долях градуса для целочисленной арифметики)
pub const THROTTLE_THRESHOLD_DECIDEGREE: i32 = 500; // 50.0°C
pub const CRITICAL_THRESHOLD_DECIDEGREE: i32 = 750; // 75.0°C
pub const HYSTERESIS_DECIDEGREE: i32 = 30;           // 3.0°C гистерезис

/// Уровень тротлинга
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub enum ThrottleLevel {
    /// Нормальная работа — 240 МГц
    Normal,
    /// Мягкий тротлинг — 160 МГц (50–60°C)
    Mild,
    /// Жёсткий тротлинг — 80 МГц (60–75°C)
    Heavy,
    /// Критическая температура — возможна остановка ядра (>75°C)
    Critical,
}

impl ThrottleLevel {
    /// Номинальная частота ядра для данного уровня (МГц)
    pub const fn target_mhz(self) -> u32 {
        match self {
            Self::Normal   => 240,
            Self::Mild     => 160,
            Self::Heavy    =>  80,
            Self::Critical =>  40,
        }
    }

    pub const fn from_decidegrees(d: i32) -> Self {
        if d >= CRITICAL_THRESHOLD_DECIDEGREE {
            Self::Critical
        } else if d >= 600 {
            Self::Heavy
        } else if d >= THROTTLE_THRESHOLD_DECIDEGREE {
            Self::Mild
        } else {
            Self::Normal
        }
    }
}

/// Калибровочный сдвиг датчика (в десятых долях °C).
/// Для ESP32-S3 даташит указывает погрешность ±2°C.
static CALIB_OFFSET_DECIDEGREE: AtomicI32 = AtomicI32::new(0);

/// Последнее прочитанное значение температуры (для ISR-safe доступа)
static LAST_TEMP_DECIDEGREE: AtomicI32 = AtomicI32::new(0);

// ---------------------------------------------------------------------------
// Низкоуровневое чтение TSENS через регистры ESP32-S3
// ---------------------------------------------------------------------------

/// Базовый адрес TSENS на ESP32-S3
const TSENS_BASE: u32 = 0x6000_8800;
const TSENS_CTRL_REG:    *mut u32 = (TSENS_BASE + 0x00) as *mut u32;
const TSENS_INT_RAW_REG: *mut u32 = (TSENS_BASE + 0x04) as *mut u32;
const TSENS_WAKEUP_CONF: *mut u32 = (TSENS_BASE + 0x0C) as *mut u32;

/// Включить тактирование TSENS и запустить измерение
unsafe fn tsens_power_on() {
    unsafe {
    // Бит 25 — tsens_power_up, бит 26 — tsens_dump_out (принудительный dump)
    let ctrl = TSENS_CTRL_REG.read_volatile();
    TSENS_CTRL_REG.write_volatile(ctrl | (1 << 25));

    // Дать датчику прогреться (~300 мкс)
    for _ in 0..10_000u32 {
        core::hint::spin_loop();
    }
    } // unsafe
}

/// Прочитать сырое значение TSENS (8-битное, беззнаковое)
unsafe fn tsens_read_raw() -> u8 {
    unsafe {
    // Биты [23:16] регистра TSENS_CTRL — raw value
    let ctrl = TSENS_CTRL_REG.read_volatile();
    ((ctrl >> 16) & 0xFF) as u8
    } // unsafe
}

unsafe fn tsens_power_off() {
    unsafe {
    let ctrl = TSENS_CTRL_REG.read_volatile();
    TSENS_CTRL_REG.write_volatile(ctrl & !(1 << 25));
    } // unsafe
}

/// Перевод сырого значения в десятые доли °C.
/// Линейная аппроксимация по даташиту ESP32-S3:
///   T(°C) = raw * 0.4386 − 27.88
/// Используем целочисленную арифметику: (raw * 4386 − 278800) / 10_000
fn raw_to_decidegrees(raw: u8) -> i32 {
    let v = raw as i32;
    (v * 4386 - 278_800) / 10_000
}

// ---------------------------------------------------------------------------
// Публичный API
// ---------------------------------------------------------------------------

/// Инициализировать датчик температуры (вызвать один раз при старте)
pub fn init() {
    unsafe { tsens_power_on() };
}

/// Прочитать температуру.
/// Возвращает значение в десятых долях °C (250 = 25.0°C).
pub fn read_decidegrees() -> i32 {
    let raw = unsafe { tsens_read_raw() };
    let calib = CALIB_OFFSET_DECIDEGREE.load(Ordering::Relaxed);
    let value = raw_to_decidegrees(raw) + calib;
    LAST_TEMP_DECIDEGREE.store(value, Ordering::Relaxed);
    value
}

/// Прочитать температуру в °C как f32.
/// Использует целочисленный путь внутри, конвертация только на выходе.
pub fn read_celsius() -> f32 {
    read_decidegrees() as f32 / 10.0
}

/// Последнее известное значение без нового измерения (ISR-safe)
pub fn last_celsius_cached() -> f32 {
    LAST_TEMP_DECIDEGREE.load(Ordering::Relaxed) as f32 / 10.0
}

/// Задать калибровочный сдвиг в десятых долях °C
pub fn set_calibration_offset(offset_decidegree: i32) {
    CALIB_OFFSET_DECIDEGREE.store(offset_decidegree, Ordering::Relaxed);
}

/// Определить уровень тротлинга по текущей температуре
pub fn current_throttle_level() -> ThrottleLevel {
    ThrottleLevel::from_decidegrees(read_decidegrees())
}

/// Нужен ли хотя бы мягкий тротлинг?
pub fn throttle_needed() -> bool {
    read_decidegrees() >= THROTTLE_THRESHOLD_DECIDEGREE
}

/// Критическая ли температура?
pub fn is_critical() -> bool {
    read_decidegrees() >= CRITICAL_THRESHOLD_DECIDEGREE
}

/// Обесточить датчик (для экономии ~60 мкА в deep sleep)
pub fn shutdown() {
    unsafe { tsens_power_off() };
}
