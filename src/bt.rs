// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Hiroshi Nakajima

//! Bonded Bluetooth devices: reconnect at boot and the pairing window.
//!
//! Bluedroid keeps the bonds (link keys) in NVS by itself. This module adds
//! which device connected most recently and the device names, kept in
//! their own NVS namespace. At boot the speaker connects to the most recent
//! device; when that one is not reachable it tries the next one, and so on.
//!
//! New devices can only pair while the speaker is discoverable: for
//! PAIRING_WINDOW after "Pairing mode" in the web UI, and always while no
//! device is bonded. Being discoverable costs radio time (inquiry scan) and
//! made the A2DP stream drop packets, so it is off otherwise. Bonded devices can connect whenever no
//! other device is connected.

use std::ffi::{CStr, CString};
use std::sync::Mutex;
use std::time::{Duration, Instant};

use esp_idf_svc::sys::*;
use log::{info, warn};

pub type Bda = [u8; 6];

const NVS_NS: &str = "btaudio_bt";
// Bonded addresses, most recently connected first (blob of 6-byte entries).
const KEY_ORDER: &str = "order";
// How long the speaker stays discoverable after "Pairing mode".
pub const PAIRING_WINDOW: Duration = Duration::from_secs(180);
// Bonded devices tried at boot, at most.
const MAX_RECONNECT: usize = 8;

struct State {
    connected: Option<Bda>,
    // Devices still to try at boot, next first.
    reconnect: Vec<Bda>,
    // Device an outgoing connection is in progress to.
    attempt: Option<Bda>,
    pairing_until: Option<Instant>,
    // Scan mode last set: (connectable, discoverable).
    scan: Option<(bool, bool)>,
}

static STATE: Mutex<State> =
    Mutex::new(State { connected: None, reconnect: Vec::new(), attempt: None, pairing_until: None, scan: None });

// RAM copies of the NVS entries, so the web UI's polling never reads flash
// (a flash read suspends the cache and can stall the audio path).
static ORDER: Mutex<Option<Vec<Bda>>> = Mutex::new(None);
static NAMES: Mutex<Vec<(Bda, Option<String>)>> = Mutex::new(Vec::new());

pub fn format(bda: &Bda) -> String {
    bda.iter().map(|b| format!("{b:02x}")).collect::<Vec<_>>().join(":")
}

pub fn parse(s: &str) -> Option<Bda> {
    let mut bda = [0u8; 6];
    let mut parts = s.trim().split(':');
    for b in bda.iter_mut() {
        *b = u8::from_str_radix(parts.next()?, 16).ok()?;
    }
    parts.next().is_none().then_some(bda)
}

/// Bonded devices, most recently connected first.
pub fn devices() -> Vec<Bda> {
    let mut list = bonded();
    let order = load_order();
    list.sort_by_key(|b| order.iter().position(|o| o == b).unwrap_or(usize::MAX));
    list
}

pub fn connected() -> Option<Bda> {
    STATE.lock().unwrap().connected
}

/// Seconds left in the pairing window, None when closed.
pub fn pairing_remaining() -> Option<u64> {
    let until = STATE.lock().unwrap().pairing_until?;
    Some(until.saturating_duration_since(Instant::now()).as_secs())
}

/// Stored name of a bonded device.
pub fn name(bda: &Bda) -> Option<String> {
    let mut names = NAMES.lock().unwrap();
    if let Some((_, n)) = names.iter().find(|(b, _)| b == bda) {
        return n.clone();
    }
    let n = nvs_get_str_val(&name_key(bda));
    names.push((*bda, n.clone()));
    n
}

fn cache_name(bda: &Bda, n: Option<String>) {
    let mut names = NAMES.lock().unwrap();
    names.retain(|(b, _)| b != bda);
    names.push((*bda, n));
}

/// A2DP is ready (at boot): start reconnecting.
pub fn on_ready() {
    let mut st = STATE.lock().unwrap();
    st.reconnect = devices();
    st.reconnect.truncate(MAX_RECONNECT);
    apply_scan(&mut st);
    next_reconnect(&mut st);
}

pub fn on_connected(bda: Bda) {
    info!("connected: {} ({})", format(&bda), name(&bda).unwrap_or_default());
    let mut st = STATE.lock().unwrap();
    st.connected = Some(bda);
    st.attempt = None;
    st.reconnect.clear();
    apply_scan(&mut st);
    drop(st);
    let mut order = load_order();
    order.retain(|b| *b != bda);
    order.insert(0, bda);
    save_order(&order);
    if name(&bda).is_none() {
        let mut b = bda;
        unsafe { esp_bt_gap_read_remote_name(b.as_mut_ptr()) };
    }
}

pub fn on_disconnected(bda: Bda) {
    let mut st = STATE.lock().unwrap();
    if st.attempt == Some(bda) {
        info!("reconnect to {} failed", format(&bda));
        st.attempt = None;
        next_reconnect(&mut st);
    } else {
        info!("disconnected: {}", format(&bda));
    }
    if st.connected == Some(bda) {
        st.connected = None;
    }
    apply_scan(&mut st);
}

/// Pairing finished (or a bonded device authenticated again).
pub fn on_authenticated(bda: Bda, device_name: &str) {
    info!("authenticated: {} ({device_name})", format(&bda));
    save_name(&bda, device_name);
    // The first bond ends the always-discoverable state.
    apply_scan(&mut STATE.lock().unwrap());
}

pub fn on_remote_name(bda: Bda, device_name: &str) {
    save_name(&bda, device_name);
}

/// Opens the pairing window. A connected device is disconnected, because the
/// speaker cannot be found while connected.
pub fn start_pairing() {
    let mut st = STATE.lock().unwrap();
    st.pairing_until = Some(Instant::now() + PAIRING_WINDOW);
    st.reconnect.clear();
    info!("pairing mode for {} s", PAIRING_WINDOW.as_secs());
    if let Some(mut bda) = st.connected {
        unsafe { esp_a2d_sink_disconnect(bda.as_mut_ptr()) };
    }
    apply_scan(&mut st);
}

/// Forgets a bonded device; disconnects it first if connected.
pub fn remove(bda: Bda) -> bool {
    let mut st = STATE.lock().unwrap();
    st.reconnect.retain(|b| *b != bda);
    if st.connected == Some(bda) {
        let mut b = bda;
        unsafe { esp_a2d_sink_disconnect(b.as_mut_ptr()) };
    }
    let mut b = bda;
    let ok = unsafe { esp_bt_gap_remove_bond_device(b.as_mut_ptr()) } == ESP_OK;
    drop(st);
    let mut order = load_order();
    order.retain(|o| *o != bda);
    save_order(&order);
    nvs_erase(&name_key(&bda));
    cache_name(&bda, None);
    info!("removed bonded device {}", format(&bda));
    // With no bond left the speaker is discoverable again.
    apply_scan(&mut STATE.lock().unwrap());
    ok
}

/// Called from the controls loop: closes the pairing window when it expires.
pub fn tick() {
    let mut st = STATE.lock().unwrap();
    if st.pairing_until.is_some_and(|t| Instant::now() >= t) {
        st.pairing_until = None;
        info!("pairing window closed");
        apply_scan(&mut st);
    }
}

fn next_reconnect(st: &mut State) {
    if st.connected.is_some() {
        st.reconnect.clear();
        return;
    }
    while !st.reconnect.is_empty() {
        let mut bda = st.reconnect.remove(0);
        info!("reconnecting to {} ({})", format(&bda), name(&bda).unwrap_or_default());
        if unsafe { esp_a2d_sink_connect(bda.as_mut_ptr()) } == ESP_OK {
            st.attempt = Some(bda);
            return;
        }
        warn!("reconnect to {} not started", format(&bda));
    }
}

/// Connectable whenever nothing is connected; discoverable only in the
/// pairing window or while no device is bonded.
fn apply_scan(st: &mut State) {
    let mode = match st.connected {
        Some(_) => (false, false),
        None => (true, st.pairing_until.is_some() || bonded_count() == 0),
    };
    if st.scan == Some(mode) {
        return;
    }
    st.scan = Some(mode);
    unsafe {
        esp_bt_gap_set_scan_mode(
            if mode.0 { esp_bt_connection_mode_t_ESP_BT_CONNECTABLE } else { esp_bt_connection_mode_t_ESP_BT_NON_CONNECTABLE },
            if mode.1 {
                esp_bt_discovery_mode_t_ESP_BT_GENERAL_DISCOVERABLE
            } else {
                esp_bt_discovery_mode_t_ESP_BT_NON_DISCOVERABLE
            },
        );
    }
    if mode.0 {
        info!("{}", if mode.1 { "discoverable" } else { "connectable (bonded devices only)" });
    }
}

fn bonded_count() -> usize {
    unsafe { esp_bt_gap_get_bond_device_num() }.max(0) as usize
}

fn bonded() -> Vec<Bda> {
    let mut n = bonded_count() as i32;
    if n == 0 {
        return Vec::new();
    }
    let mut list = vec![[0u8; 6]; n as usize];
    if unsafe { esp_bt_gap_get_bond_device_list(&mut n, list.as_mut_ptr()) } != ESP_OK {
        return Vec::new();
    }
    list.truncate(n.max(0) as usize);
    list
}

// ─── NVS ─────────────────────────────────────────────────────────────────────

fn name_key(bda: &Bda) -> String {
    format!("n{}", bda.iter().map(|b| format!("{b:02x}")).collect::<String>())
}

fn save_name(bda: &Bda, device_name: &str) {
    let device_name = device_name.trim();
    if !device_name.is_empty() && name(bda).as_deref() != Some(device_name) {
        with_nvs(true, |h| {
            if let (Ok(k), Ok(v)) = (CString::new(name_key(bda)), CString::new(device_name)) {
                unsafe { nvs_set_str(h, k.as_ptr(), v.as_ptr()) };
            }
        });
        cache_name(bda, Some(device_name.to_string()));
    }
}

fn load_order() -> Vec<Bda> {
    let mut cache = ORDER.lock().unwrap();
    if let Some(order) = cache.as_ref() {
        return order.clone();
    }
    let order = read_order();
    *cache = Some(order.clone());
    order
}

fn read_order() -> Vec<Bda> {
    with_nvs(false, |h| {
        let key = CString::new(KEY_ORDER).unwrap();
        let mut len = 0usize;
        unsafe {
            if nvs_get_blob(h, key.as_ptr(), core::ptr::null_mut(), &mut len) != ESP_OK || len == 0 {
                return Vec::new();
            }
            let mut buf = vec![0u8; len];
            if nvs_get_blob(h, key.as_ptr(), buf.as_mut_ptr() as *mut _, &mut len) != ESP_OK {
                return Vec::new();
            }
            buf.chunks_exact(6).map(|c| c.try_into().unwrap()).collect()
        }
    })
    .unwrap_or_default()
}

fn save_order(order: &[Bda]) {
    if ORDER.lock().unwrap().as_deref() == Some(order) {
        return;
    }
    *ORDER.lock().unwrap() = Some(order.to_vec());
    let buf: Vec<u8> = order.iter().flatten().copied().collect();
    with_nvs(true, |h| {
        let key = CString::new(KEY_ORDER).unwrap();
        unsafe { nvs_set_blob(h, key.as_ptr(), buf.as_ptr() as *const _, buf.len()) };
    });
}

fn nvs_get_str_val(key: &str) -> Option<String> {
    with_nvs(false, |h| {
        let k = CString::new(key).ok()?;
        let mut len = 0usize;
        unsafe {
            if nvs_get_str(h, k.as_ptr(), core::ptr::null_mut(), &mut len) != ESP_OK || len == 0 {
                return None;
            }
            let mut buf = vec![0u8; len];
            if nvs_get_str(h, k.as_ptr(), buf.as_mut_ptr() as *mut _, &mut len) != ESP_OK {
                return None;
            }
            CStr::from_bytes_until_nul(&buf).ok().map(|s| s.to_string_lossy().into_owned())
        }
    })
    .flatten()
}

fn nvs_erase(key: &str) {
    with_nvs(true, |h| {
        if let Ok(k) = CString::new(key) {
            unsafe { nvs_erase_key(h, k.as_ptr()) };
        }
    });
}

/// Opens the namespace, runs `f`, commits (when writing) and closes it.
fn with_nvs<T>(write: bool, f: impl FnOnce(nvs_handle_t) -> T) -> Option<T> {
    let ns = CString::new(NVS_NS).ok()?;
    let mode = if write { nvs_open_mode_t_NVS_READWRITE } else { nvs_open_mode_t_NVS_READONLY };
    let mut h: nvs_handle_t = 0;
    if unsafe { nvs_open(ns.as_ptr(), mode, &mut h) } != ESP_OK {
        return None;
    }
    let r = f(h);
    if write {
        unsafe { nvs_commit(h) };
    }
    unsafe { nvs_close(h) };
    Some(r)
}
