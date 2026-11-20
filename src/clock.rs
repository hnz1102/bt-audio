// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Hiroshi Nakajima

//! Local time from the SNTP clock and the `timezone` setting (a POSIX TZ
//! string such as `JST-9`), for syslog timestamps and the web UI.

use std::ffi::CString;
use std::time::{SystemTime, UNIX_EPOCH};

use esp_idf_svc::sys::*;
use log::warn;

struct LocalTime {
    year: i64,
    month: i64,
    day: i64,
    hour: i64,
    minute: i64,
    second: i64,
    // Seconds east of UTC.
    offset: i64,
}

/// Sets the time zone used by `localtime`. Takes effect immediately.
pub fn set_timezone(tz: &str) {
    let tz = if tz.trim().is_empty() { "UTC0" } else { tz.trim() };
    match CString::new(tz) {
        Ok(value) => unsafe {
            setenv(c"TZ".as_ptr(), value.as_ptr(), 1);
            tzset();
        },
        Err(_) => warn!("invalid time zone '{tz}'"),
    }
}

/// None until the clock has been set by SNTP.
fn now() -> Option<LocalTime> {
    let secs = SystemTime::now().duration_since(UNIX_EPOCH).ok()?.as_secs() as i64;
    // Before the first sync the clock is near 1970.
    if secs < 1_700_000_000 {
        return None;
    }
    let t = secs as time_t;
    let mut tm: tm = unsafe { core::mem::zeroed() };
    if unsafe { localtime_r(&t, &mut tm) }.is_null() {
        return None;
    }
    let (year, month, day) = (tm.tm_year as i64 + 1900, tm.tm_mon as i64 + 1, tm.tm_mday as i64);
    let (hour, minute, second) = (tm.tm_hour as i64, tm.tm_min as i64, tm.tm_sec as i64);
    let local = days_from_civil(year, month, day) * 86_400 + hour * 3600 + minute * 60 + second;
    Some(LocalTime { year, month, day, hour, minute, second, offset: local - secs })
}

/// RFC 3339 local time with its UTC offset (e.g. `2026-11-17T23:53:01+09:00`),
/// or None until the clock has been set.
pub fn rfc3339() -> Option<String> {
    let t = now()?;
    let sign = if t.offset < 0 { '-' } else { '+' };
    let off = t.offset.abs() / 60;
    Some(format!(
        "{:04}-{:02}-{:02}T{:02}:{:02}:{:02}{sign}{:02}:{:02}",
        t.year, t.month, t.day, t.hour, t.minute, t.second, off / 60, off % 60
    ))
}

/// Local time for display, e.g. `2026-11-17 23:53:01`, or None before SNTP.
pub fn display() -> Option<String> {
    let t = now()?;
    Some(format!(
        "{:04}-{:02}-{:02} {:02}:{:02}:{:02}",
        t.year, t.month, t.day, t.hour, t.minute, t.second
    ))
}

/// Days since 1970-01-01 for a civil date (Howard Hinnant's algorithm).
fn days_from_civil(year: i64, month: i64, day: i64) -> i64 {
    let y = if month <= 2 { year - 1 } else { year };
    let era = y.div_euclid(400);
    let yoe = y - era * 400;
    let doy = (153 * ((month + 9) % 12) + 2) / 5 + day - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    era * 146_097 + doe - 719_468
}
