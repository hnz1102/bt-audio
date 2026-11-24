// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Hiroshi Nakajima

//! Settings kept in NVS and edited through the web UI. A missing or empty key
//! falls back to its default, which comes from `cfg.toml` (section `[bt-audio]`)
//! at build time or, for a key not listed there, from the value below.

use std::ffi::CString;

use esp_idf_svc::sys::*;
use log::warn;

const NVS_NS: &str = "btaudio_cfg";

// Declares the settings once: the build-time defaults, the struct and the NVS
// key list. Every key is the field name, cut to the 15 characters NVS allows
// (see nvs_key).
macro_rules! settings {
    ($($name:ident = $default:literal),* $(,)?) => {
        // `toml_config` replaces each default with the value in cfg.toml.
        #[toml_cfg::toml_config]
        pub struct Defaults {
            $(#[default($default)] $name: &'static str,)*
        }

        #[derive(Clone, Debug)]
        pub struct Config {
            $(pub $name: String,)*
        }

        impl Config {
            pub fn defaults() -> Self {
                Self { $($name: DEFAULTS.$name.to_string(),)* }
            }

            /// Reads every setting from NVS, using the default where unset.
            pub fn load() -> Self {
                let d = Self::defaults();
                Self {
                    $($name: match nvs_get(stringify!($name)) {
                        Some(v) if !v.is_empty() => v,
                        _ => d.$name,
                    },)*
                }
            }

            fn pairs(&self) -> Vec<(&'static str, &str)> {
                vec![$((stringify!($name), self.$name.as_str()),)*]
            }
        }
    };
}

settings! {
    wifi_ssid = "",
    wifi_psk = "",
    wps_enable = "true",
    syslog_host_name = "bt-audio",
    syslog_app_name = "bt-audio",
    ip_mode = "dhcp",
    ip_address = "",
    subnet_mask = "255.255.255.0",
    gateway = "",
    dns = "",
    ntp_server1 = "time.aws.com",
    ntp_server2 = "time.google.com",
    ntp_server3 = "time.cloudflare.com",
    ntp_server4 = "ntp.nict.jp",
    timezone = "JST-9",
    syslog_enable = "false",
    syslog_server = "",
    web_user = "admin",
    web_password = "btpass",
    bt_name = "ESP32-Speaker",
    gain_q8 = "256",
    vol_range_db = "60",
    pd_max_mv = "24000",
    subsonic_hz = "30",
    crossover_hz = "150",
    bass_eq_hz = "60",
    bass_eq_db = "6",
}

impl Config {
    /// Software gain in Q8 fixed point (256 = 1.0x).
    pub fn gain_q8(&self) -> i32 {
        self.gain_q8.trim().parse().unwrap_or(256).clamp(0, 1024)
    }

    /// Span of the volume knob below 0 dB.
    pub fn vol_range_db(&self) -> i32 {
        self.vol_range_db.trim().parse().unwrap_or(60).clamp(6, 100)
    }

    /// Crossover frequency between the main amplifier and the woofer, Hz.
    pub fn crossover_hz(&self) -> u32 {
        self.crossover_hz.trim().parse().unwrap_or(150u32).clamp(50, 300)
    }

    /// Woofer bass EQ centre frequency, Hz.
    pub fn bass_eq_hz(&self) -> u32 {
        self.bass_eq_hz.trim().parse().unwrap_or(60u32).clamp(30, 150)
    }

    /// Woofer bass EQ boost, dB (0 = off).
    pub fn bass_eq_db(&self) -> u32 {
        self.bass_eq_db.trim().parse().unwrap_or(6u32).min(12)
    }

    /// Woofer subsonic high-pass frequency in Hz, 0 = off.
    pub fn subsonic_hz(&self) -> u32 {
        let hz = self.subsonic_hz.trim().parse().unwrap_or(30u32).min(100);
        if hz < 10 { 0 } else { hz }
    }

    /// Highest USB PD voltage to request; the TAS5754M allows up to 26.4 V.
    pub fn pd_max_mv(&self) -> u16 {
        self.pd_max_mv.trim().parse().unwrap_or(24_000).clamp(5_000, 24_000)
    }

    pub fn static_ip(&self) -> bool {
        self.ip_mode == "static"
    }

    /// Writes every setting to NVS.
    pub fn save(&self) -> bool {
        nvs_set(&self.pairs())
    }

    /// Writes only the WiFi credentials, e.g. after a WPS handshake.
    pub fn save_wifi(&self) -> bool {
        nvs_set(&[("wifi_ssid", &self.wifi_ssid), ("wifi_psk", &self.wifi_psk)])
    }
}

/// Reads a value kept outside the settings (e.g. a learned knob floor).
pub fn load_value(key: &str) -> Option<String> {
    nvs_get(key)
}

/// Stores a value kept outside the settings; erased by a factory reset.
pub fn save_value(key: &str, value: &str) -> bool {
    nvs_set(&[(key, value)])
}

/// Erases every stored setting; the defaults apply from the next boot.
pub fn erase() -> bool {
    let Ok(ns) = CString::new(NVS_NS) else { return false };
    unsafe {
        let mut handle: nvs_handle_t = 0;
        if nvs_open(ns.as_ptr(), nvs_open_mode_t_NVS_READWRITE, &mut handle) != ESP_OK {
            return false;
        }
        let ok = nvs_erase_all(handle) == ESP_OK;
        nvs_commit(handle);
        nvs_close(handle);
        ok
    }
}

fn nvs_get(key: &str) -> Option<String> {
    let ns = CString::new(NVS_NS).ok()?;
    let k = CString::new(nvs_key(key)).ok()?;
    unsafe {
        let mut handle: nvs_handle_t = 0;
        if nvs_open(ns.as_ptr(), nvs_open_mode_t_NVS_READONLY, &mut handle) != ESP_OK {
            return None;
        }
        // The first call returns the length including the terminating NUL.
        let mut len: usize = 0;
        if nvs_get_str(handle, k.as_ptr(), core::ptr::null_mut(), &mut len) != ESP_OK || len == 0 {
            nvs_close(handle);
            return None;
        }
        let mut buf = vec![0u8; len];
        let ret = nvs_get_str(handle, k.as_ptr(), buf.as_mut_ptr() as *mut _, &mut len);
        nvs_close(handle);
        if ret != ESP_OK {
            return None;
        }
        while buf.last() == Some(&0) {
            buf.pop();
        }
        String::from_utf8(buf).ok()
    }
}

/// NVS key for a setting: NVS keys are limited to 15 characters, so a longer
/// field name (`syslog_host_name`) is cut. The cut names are still unique.
fn nvs_key(name: &str) -> &str {
    &name[..name.len().min(15)]
}

fn nvs_set(pairs: &[(&str, &str)]) -> bool {
    let Ok(ns) = CString::new(NVS_NS) else { return false };
    unsafe {
        let mut handle: nvs_handle_t = 0;
        if nvs_open(ns.as_ptr(), nvs_open_mode_t_NVS_READWRITE, &mut handle) != ESP_OK {
            return false;
        }
        let mut ok = true;
        for (key, val) in pairs {
            match (CString::new(nvs_key(key)), CString::new(*val)) {
                (Ok(k), Ok(v)) => {
                    if nvs_set_str(handle, k.as_ptr(), v.as_ptr()) != ESP_OK {
                        warn!("NVS: failed to write '{key}'");
                        ok = false;
                    }
                }
                _ => {
                    warn!("NVS: invalid value for '{key}'");
                    ok = false;
                }
            }
        }
        ok &= nvs_commit(handle) == ESP_OK;
        nvs_close(handle);
        ok
    }
}
