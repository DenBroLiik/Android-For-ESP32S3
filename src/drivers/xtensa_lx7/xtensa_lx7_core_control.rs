// xtensa_lx7_core_control.rs
// Контроль ядра Xtensa LX7: частота CPU, диапазон, тротлинг, watchdog

use crate::xtensa_lx7_core_temperature::{ThrottleLevel, current_throttle_level};

// Регистры управления тактированием ESP32-S3

const SYSTEM_BASE: u32            = 0x600C_0000;
const SYSTEM_CPU_PER_CONF_REG: *mut u32 = (SYSTEM_BASE + 0x08) as *mut u32;
const CPU_PERIOD_SEL_MASK: u32    = 0x3;

unsafe fn set_cpu_mhz_raw(mhz: u32) {
    unsafe { let sel: u32 = match mhz {
        240..=u32::MAX => 2, // 240 МГц
        160..=239      => 1, // 160 МГц
        _              => 0, // 80 МГц (минимум)
    };
    let old = SYSTEM_CPU_PER_CONF_REG.read_volatile();
    SYSTEM_CPU_PER_CONF_REG.write_volatile((old & !CPU_PERIOD_SEL_MASK) | sel); }
}

// CpuFreq

/// Поддерживаемые частоты CPU на ESP32-S3 через BBPLL.
/// Critical тротлинг (таргет 40 МГц) отображается на Mhz80 — аппаратный минимум.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub enum CpuFreq {
    Mhz80,   // 0
    Mhz160,  // 1
    Mhz240,  // 2
}

impl CpuFreq {
    /// MHz в число
    pub const fn as_mhz(self) -> u32 {
        match self {
            Self::Mhz80  =>  80,
            Self::Mhz160 => 160,
            Self::Mhz240 => 240,
        }
    }

    /// Число в CpuFreq (неподдерживаемые значения округляются вниз)
    pub const fn from_mhz(mhz: u32) -> Self {
        match mhz {
            240..=u32::MAX => Self::Mhz240,
            160..=239      => Self::Mhz160,
            _              => Self::Mhz80,
        }
    }

    /// Ограничить значение диапазоном [min, max]
    pub fn clamped(self, min: Self, max: Self) -> Self {
        self.max(min).min(max)
    }
}

// Watchdog RTC (RWDT)

const RTC_CNTL_BASE: u32             = 0x6000_8000;
const RTC_CNTL_WDTCONFIG0: *mut u32  = (RTC_CNTL_BASE + 0x98) as *mut u32;
const RTC_CNTL_WDTFEED:    *mut u32  = (RTC_CNTL_BASE + 0xAC) as *mut u32;
const RTC_CNTL_WDTWPROTECT:*mut u32  = (RTC_CNTL_BASE + 0xB0) as *mut u32;
const WDT_WRITE_KEY: u32             = 0x50D8_3AA1;

fn wdt_unlock() { unsafe { RTC_CNTL_WDTWPROTECT.write_volatile(WDT_WRITE_KEY) }; }
fn wdt_lock()   { unsafe { RTC_CNTL_WDTWPROTECT.write_volatile(0) }; }

/// Покормить RTC watchdog
pub fn wdt_feed() {
    wdt_unlock();
    unsafe { RTC_CNTL_WDTFEED.write_volatile(1) };
    wdt_lock();
}

/// Отключить RTC watchdog (для отладки)
pub fn wdt_disable() {
    wdt_unlock();
    unsafe {
        let cfg = RTC_CNTL_WDTCONFIG0.read_volatile();
        RTC_CNTL_WDTCONFIG0.write_volatile(cfg & !(1 << 31));
    }
    wdt_lock();
}

// CoreState 

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CoreState {
    pub throttle:        ThrottleLevel,
    pub current_freq:    CpuFreq,
    pub min_freq:        CpuFreq,
    pub max_freq:        CpuFreq,
    pub throttle_events: u32,
    pub critical_events: u32,
}

impl CoreState {
    const fn default() -> Self {
        Self {
            throttle:        ThrottleLevel::Normal,
            current_freq:    CpuFreq::Mhz240,
            min_freq:        CpuFreq::Mhz80,
            max_freq:        CpuFreq::Mhz240,
            throttle_events: 0,
            critical_events: 0,
        }
    }
}

static mut CORE_STATE: CoreState = CoreState::default();

// ── Внутренний хелпер ─────────────────────────────────────────────────────

unsafe fn apply_freq(freq: CpuFreq) {
    unsafe {
    set_cpu_mhz_raw(freq.as_mhz());
    CORE_STATE.current_freq = freq;
    }
}

// Публичный API

/// Инициализировать контроллер ядра. Вызвать один раз после `esp_hal::init()`.
pub fn init() {
    crate::xtensa_lx7_core_temperature::init();
    unsafe { apply_freq(CpuFreq::Mhz240) };
}

/// Главный цикл тротлинга — вызывать периодически (например, каждые 500 мс).
/// Применяет диапазон [min_freq, max_freq] и отслеживает события перегрева.
pub fn throttle_update() {
    let level = current_throttle_level();
    let target = unsafe {
        CpuFreq::from_mhz(level.target_mhz()).clamped(CORE_STATE.min_freq, CORE_STATE.max_freq)
    };
    if level != unsafe { CORE_STATE.throttle } {
        unsafe {
            match level {
                ThrottleLevel::Critical => {
                    CORE_STATE.critical_events = CORE_STATE.critical_events.saturating_add(1);
                }
                l if l > ThrottleLevel::Normal => {
                    CORE_STATE.throttle_events = CORE_STATE.throttle_events.saturating_add(1);
                }
                _ => {}
            }
            apply_freq(target);
            CORE_STATE.throttle = level;
        }
    }
    wdt_feed();
}

/// Установить минимальную частоту (тротлинг не опустится ниже).
/// Если текущая частота ниже нового минимума — поднимается сразу.
pub fn set_min_freq(freq: CpuFreq) {
    unsafe {
        CORE_STATE.min_freq = freq;
        if CORE_STATE.current_freq < freq { apply_freq(freq); }
    }
}

/// Установить максимальную частоту (например, для энергосбережения).
/// Если текущая частота выше нового максимума — снижается сразу.
pub fn set_max_freq(freq: CpuFreq) {
    unsafe {
        CORE_STATE.max_freq = freq;
        if CORE_STATE.current_freq > freq { apply_freq(freq); }
    }
}

/// Установить диапазон частот сразу.
pub fn set_freq_range(min: CpuFreq, max: CpuFreq) {
    set_max_freq(max); // сначала ограничиваем верхний порог
    set_min_freq(min); // потом поднимаем нижний
}

/// Принудительно установить частоту, минуя логику тротлинга.
/// Следующий вызов `throttle_update()` может переопределить.
pub fn force_cpu_freq(freq: CpuFreq) {
    unsafe {
        apply_freq(freq);
        CORE_STATE.throttle = ThrottleLevel::Normal;
    }
}

/// Текущее состояние ядра
pub fn state() -> CoreState { unsafe { CORE_STATE } }

/// Минимальная разрешённая частота
pub fn min_freq() -> CpuFreq { unsafe { CORE_STATE.min_freq } }

/// Максимальная разрешённая частота
pub fn max_freq() -> CpuFreq { unsafe { CORE_STATE.max_freq } }

/// Текущая частота CPU
pub fn current_freq() -> CpuFreq { unsafe { CORE_STATE.current_freq } }

/// Количество тротлинг-событий с старта
pub fn throttle_event_count() -> u32 { unsafe { CORE_STATE.throttle_events } }

/// Количество критических перегревов
pub fn critical_event_count() -> u32 { unsafe { CORE_STATE.critical_events } }