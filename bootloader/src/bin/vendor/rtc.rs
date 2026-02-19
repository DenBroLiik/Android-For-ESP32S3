use core::cell::RefCell;
use critical_section::Mutex;

// DateTime структура
#[derive(Clone, Copy)]
struct DateTime {
    year: u32,
    month: u32,
    day: u32,
    hour: u32,
    minute: u32,
    second: u32,
}

impl DateTime {
    const fn new(year: u32, month: u32, day: u32, hour: u32, minute: u32, second: u32) -> Self {
        DateTime {
            year,
            month,
            day,
            hour,
            minute,
            second,
        }
    }

    fn tick(&mut self) {
        self.second += 1;

        if self.second >= 60 {
            self.second = 0;
            self.minute += 1;
        }

        if self.minute >= 60 {
            self.minute = 0;
            self.hour += 1;
        }

        if self.hour >= 24 {
            self.hour = 0;
            self.day += 1;
        }

        let days_in_month = match self.month {
            1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
            4 | 6 | 9 | 11 => 30,
            2 => {
                if self.is_leap_year() {
                    29
                } else {
                    28
                }
            }
            _ => 30,
        };

        if self.day > days_in_month {
            self.day = 1;
            self.month += 1;
        }

        if self.month > 12 {
            self.month = 1;
            self.year += 1;
        }
    }

    fn is_leap_year(&self) -> bool {
        (self.year % 4 == 0 && self.year % 100 != 0) || (self.year % 400 == 0)
    }
}

// Глобальный RTC
static DATETIME: Mutex<RefCell<DateTime>> =
    Mutex::new(RefCell::new(DateTime::new(2026, 2, 12, 0, 0, 0)));

// Получить дату
pub fn get_date() -> (u32, u32, u32) {
    critical_section::with(|cs| {
        let dt = DATETIME.borrow_ref(cs);
        (dt.year, dt.month, dt.day)
    })
}

// Получить время
pub fn get_time() -> (u32, u32, u32) {
    critical_section::with(|cs| {
        let dt = DATETIME.borrow_ref(cs);
        (dt.hour, dt.minute, dt.second)
    })
}

// Установить дату (время остаётся тем же)
pub fn set_date(year: u32, month: u32, day: u32) {
    critical_section::with(|cs| {
        let mut dt = DATETIME.borrow_ref_mut(cs);
        dt.year = year;
        dt.month = month;
        dt.day = day;
    });
}

// Установить время (дата остаётся той же)
pub fn set_time(hour: u32, minute: u32, second: u32) {
    critical_section::with(|cs| {
        let mut dt = DATETIME.borrow_ref_mut(cs);
        dt.hour = hour;
        dt.minute = minute;
        dt.second = second;
    });
}

// Тик RTC
pub fn rtc_tick() {
    critical_section::with(|cs| {
        let mut dt = DATETIME.borrow_ref_mut(cs);
        dt.tick();
    });
}
