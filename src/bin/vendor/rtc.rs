use core::cell::RefCell;
use critical_section::Mutex;

// ── DateTime ───────────────────────────────────────────────────────────────

// Размер в памяти: 2 + 1 + 1 + 1 + 1 + 1 + 2 = 9 байт (вместо 28 байт с u32)
#[derive(Clone, Copy)]
struct DateTime {
    year:        u16,  // 0–65535
    month:       u8,   // 1–12
    day:         u8,   // 1–31
    hour:        u8,   // 0–23
    minute:      u8,   // 0–59
    second:      u8,   // 0–59
    millisecond: u16,  // 0–999
}

impl DateTime {
    const fn new(year: u16, month: u8, day: u8) -> Self {
        DateTime { year, month, day, hour: 0, minute: 0, second: 0, millisecond: 0 }
    }

    // Каждый уровень: total = current + delta; new = total % limit; carry = total / limit
    fn tick_ms(&mut self, ms: u32) {
        let total = self.millisecond as u32 + ms;
        self.millisecond = (total % 1000) as u16;
        let carry = total / 1000;
        if carry > 0 { self.tick_s(carry); }
    }

    fn tick_s(&mut self, s: u32) {
        let total = self.second as u32 + s;
        self.second = (total % 60) as u8;
        let carry = total / 60;
        if carry > 0 { self.tick_min(carry); }
    }

    fn tick_min(&mut self, m: u32) {
        let total = self.minute as u32 + m;
        self.minute = (total % 60) as u8;
        let carry = total / 60;
        if carry > 0 { self.tick_hour(carry); }
    }

    fn tick_hour(&mut self, h: u32) {
        let total = self.hour as u32 + h;
        self.hour = (total % 24) as u8;
        let carry = total / 24;
        if carry > 0 { self.tick_day(carry); }
    }

    fn tick_day(&mut self, mut days: u32) {
        while days > 0 {
            let left = (self.days_in_month() - self.day + 1) as u32;
            if days < left {
                self.day += days as u8;
                break;
            }
            days -= left;
            self.day = 1;
            self.month += 1;
            if self.month > 12 {
                self.month = 1;
                self.year = self.year.wrapping_add(1);
            }
        }
    }

    fn days_in_month(&self) -> u8 {
        match self.month {
            1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
            4 | 6 | 9 | 11               => 30,
            2 => if self.is_leap_year() { 29 } else { 28 },
            _                             => 30,
        }
    }

    fn is_leap_year(&self) -> bool {
        let y = self.year as u32;
        (y % 4 == 0 && y % 100 != 0) || y % 400 == 0
    }
}

// ── Глобальный RTC ─────────────────────────────────────────────────────

static DATETIME: Mutex<RefCell<DateTime>> =
    Mutex::new(RefCell::new(DateTime::new(2026, 1, 1)));

// ── Getters ──────────────────────────────────────────────────────────────────

/// Дата: (year, month, day)
pub fn get_date() -> (u16, u8, u8) {
    critical_section::with(|cs| {
        let dt = DATETIME.borrow_ref(cs);
        (dt.year, dt.month, dt.day)
    })
}

/// Время: (hour, minute, second, ms)
pub fn get_time() -> (u8, u8, u8) {
    critical_section::with(|cs| {
        let dt = DATETIME.borrow_ref(cs);
        (dt.hour, dt.minute, dt.second)
    })
}

/// Время + ms: (hour, minute, second, millisecond)
pub fn get_time_ms() -> (u8, u8, u8, u16) {
    critical_section::with(|cs| {
        let dt = DATETIME.borrow_ref(cs);
        (dt.hour, dt.minute, dt.second, dt.millisecond)
    })
}

/// Только миллисекунды (0–999)
pub fn get_ms() -> u16 {
    critical_section::with(|cs| DATETIME.borrow_ref(cs).millisecond)
}


// ── Setters ──────────────────────────────────────────────────────────────────

/// Установить дату
pub fn set_date(year: u16, month: u8, day: u8) {
    critical_section::with(|cs| {
        let mut dt = DATETIME.borrow_ref_mut(cs);
        dt.year = year; dt.month = month; dt.day = day;
    });
}

/// Установить время (ms сбрасывается в 0)
pub fn set_time(hour: u8, minute: u8, second: u8) {
    critical_section::with(|cs| {
        let mut dt = DATETIME.borrow_ref_mut(cs);
        dt.hour = hour; dt.minute = minute; dt.second = second; dt.millisecond = 0;
    });
}

/// Установить время + ms
pub fn set_time_ms(hour: u8, minute: u8, second: u8, millisecond: u16) {
    critical_section::with(|cs| {
        let mut dt = DATETIME.borrow_ref_mut(cs);
        dt.hour = hour; dt.minute = minute; dt.second = second;
        dt.millisecond = millisecond.min(999);
    });
}

// ── Tick ─────────────────────────────────────────────────────────────────────

/// Тик на `ms` миллисекунд — автоматически ролловер в сек/мин/ч/дни.
pub fn rtc_tick_ms(ms: u32) {
    critical_section::with(|cs| {
        DATETIME.borrow_ref_mut(cs).tick_ms(ms);
    });
}

/// Тик на 1 секунду (совместимость)
pub fn rtc_tick() {
    rtc_tick_ms(1000);
}