
//! A2DP sink for the original ESP32: receives audio from a phone/PC over
//! Bluetooth Classic and plays it out through an I2S DAC (e.g. PCM5102A).
//!
//! BT data callback -> bounded channel -> I2S writer thread, so a slow I2S
//! write never stalls the Bluetooth stack.
// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Hiroshi Nakajima
#![allow(non_upper_case_globals)]

use std::ffi::CString;
use std::slice;
use std::sync::mpsc::{sync_channel, SyncSender};
use std::sync::OnceLock;
use std::thread;

use anyhow::Result;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::sys::bt_shim::*;
use esp_idf_svc::sys::*;
use log::{info, warn};

const DEVICE_NAME: &str = "ESP32-Speaker";
const DEFAULT_SAMPLE_RATE: u32 = 44_100;

// Software gain in Q8 fixed point (256 = 1.0x). Samples are clamped, so large
// values clip loud material.
const GAIN_Q8: i32 = 128; // 0.5x
// Queue depth between the BT task and the I2S writer, in A2DP packets.
const QUEUE_DEPTH: usize = 128;

// I2S pins (BCK / LRCK / DATA) for the DAC.
const PIN_BCK: i32 = 26;
const PIN_WS: i32 = 25;
const PIN_DOUT: i32 = 22;

enum Msg {
    Pcm(Vec<u8>),
    Rate(u32),
}

static AUDIO_TX: OnceLock<SyncSender<Msg>> = OnceLock::new();

fn main() -> Result<()> {
    link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    // Bluetooth stores pairing keys in NVS.
    let _nvs = EspDefaultNvsPartition::take()?;

    esp!(unsafe { bt_shim_i2s_start(DEFAULT_SAMPLE_RATE, PIN_BCK, PIN_WS, PIN_DOUT) })?;

    let (tx, rx) = sync_channel::<Msg>(QUEUE_DEPTH);
    AUDIO_TX.set(tx).ok();
    thread::Builder::new()
        .name("i2s-writer".into())
        .stack_size(4096)
        .spawn(move || {
            for msg in rx {
                match msg {
                    Msg::Pcm(mut data) => {
                        apply_gain(&mut data);
                        let mut off = 0;
                        while off < data.len() {
                            let n = unsafe { bt_shim_i2s_write(data[off..].as_ptr(), data.len() - off) };
                            if n == 0 {
                                break;
                            }
                            off += n;
                        }
                    }
                    Msg::Rate(rate) => {
                        if let Err(e) = esp!(unsafe { bt_shim_i2s_set_rate(rate) }) {
                            warn!("i2s set rate {rate} failed: {e}");
                        }
                    }
                }
            }
        })?;

    esp!(unsafe { bt_shim_controller_start() })?;
    esp!(unsafe { esp_bluedroid_init() })?;
    esp!(unsafe { esp_bluedroid_enable() })?;

    let name = CString::new(DEVICE_NAME)?;
    esp!(unsafe { esp_bt_dev_set_device_name(name.as_ptr()) })?;
    esp!(unsafe { esp_bt_gap_register_callback(Some(gap_cb)) })?;

    esp!(unsafe { esp_a2d_register_callback(Some(a2d_cb)) })?;
    esp!(unsafe { esp_a2d_sink_register_data_callback(Some(a2d_data_cb)) })?;
    // Completion is reported via ESP_A2D_PROF_STATE_EVT in a2d_cb.
    esp!(unsafe { esp_a2d_sink_init() })?;

    info!("A2DP sink started as \"{DEVICE_NAME}\"");
    Ok(())
}

/// Scales interleaved little-endian 16-bit samples by `GAIN_Q8`, saturating.
fn apply_gain(pcm: &mut [u8]) {
    for s in pcm.chunks_exact_mut(2) {
        let v = i16::from_le_bytes([s[0], s[1]]) as i32;
        let v = ((v * GAIN_Q8) >> 8).clamp(i16::MIN as i32, i16::MAX as i32) as i16;
        s.copy_from_slice(&v.to_le_bytes());
    }
}

unsafe extern "C" fn gap_cb(event: esp_bt_gap_cb_event_t, param: *mut esp_bt_gap_cb_param_t) {
    let param = &mut *param;
    match event {
        esp_bt_gap_cb_event_t_ESP_BT_GAP_AUTH_CMPL_EVT => {
            let auth = &param.auth_cmpl;
            if auth.stat == esp_bt_status_t_ESP_BT_STATUS_SUCCESS {
                let name = std::ffi::CStr::from_ptr(auth.device_name.as_ptr() as *const _);
                info!("authenticated: {}", name.to_string_lossy());
            } else {
                warn!("authentication failed, status {}", auth.stat);
            }
        }
        // Numeric comparison (SSP): accept automatically since we have no UI.
        esp_bt_gap_cb_event_t_ESP_BT_GAP_CFM_REQ_EVT => {
            info!("pairing request, passkey {}", param.cfm_req.num_val);
            esp_bt_gap_ssp_confirm_reply(param.cfm_req.bda.as_mut_ptr(), true);
        }
        // Legacy pairing fallback.
        esp_bt_gap_cb_event_t_ESP_BT_GAP_PIN_REQ_EVT => {
            let mut pin: esp_bt_pin_code_t = [b'1', b'2', b'3', b'4', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
            esp_bt_gap_set_pin(esp_bt_pin_type_t_ESP_BT_PIN_TYPE_FIXED, 4, pin.as_mut_ptr());
            esp_bt_gap_pin_reply(param.pin_req.bda.as_mut_ptr(), true, 4, pin.as_mut_ptr());
        }
        _ => {}
    }
}

unsafe extern "C" fn a2d_cb(event: esp_a2d_cb_event_t, param: *mut esp_a2d_cb_param_t) {
    let param = &*param;
    match event {
        esp_a2d_cb_event_t_ESP_A2D_PROF_STATE_EVT => {
            esp_bt_gap_set_scan_mode(
                esp_bt_connection_mode_t_ESP_BT_CONNECTABLE,
                esp_bt_discovery_mode_t_ESP_BT_GENERAL_DISCOVERABLE,
            );
            info!("discoverable");
        }
        esp_a2d_cb_event_t_ESP_A2D_CONNECTION_STATE_EVT => {
            let state = param.conn_stat.state;
            if state == esp_a2d_connection_state_t_ESP_A2D_CONNECTION_STATE_CONNECTED {
                info!("connected: {:02x?}", param.conn_stat.remote_bda);
                esp_bt_gap_set_scan_mode(
                    esp_bt_connection_mode_t_ESP_BT_NON_CONNECTABLE,
                    esp_bt_discovery_mode_t_ESP_BT_NON_DISCOVERABLE,
                );
            } else if state == esp_a2d_connection_state_t_ESP_A2D_CONNECTION_STATE_DISCONNECTED {
                info!("disconnected");
                esp_bt_gap_set_scan_mode(
                    esp_bt_connection_mode_t_ESP_BT_CONNECTABLE,
                    esp_bt_discovery_mode_t_ESP_BT_GENERAL_DISCOVERABLE,
                );
            }
        }
        esp_a2d_cb_event_t_ESP_A2D_AUDIO_STATE_EVT => {
            info!("audio state {}", param.audio_stat.state);
        }
        esp_a2d_cb_event_t_ESP_A2D_AUDIO_CFG_EVT => {
            let mcc = &param.audio_cfg.mcc;
            if mcc.type_ == ESP_A2D_MCT_SBC as _ {
                // First SBC codec-info octet: bits 4-7 are the sampling frequency.
                let oct0 = mcc.cie.sbc[0];
                let rate = if oct0 & (1 << 6) != 0 {
                    32_000
                } else if oct0 & (1 << 5) != 0 {
                    44_100
                } else if oct0 & (1 << 4) != 0 {
                    48_000
                } else {
                    16_000
                };
                info!("SBC sample rate {rate} Hz");
                if let Some(tx) = AUDIO_TX.get() {
                    let _ = tx.send(Msg::Rate(rate));
                }
            }
        }
        _ => {}
    }
}

/// Called with decoded 16-bit stereo PCM, runs on the Bluetooth task.
unsafe extern "C" fn a2d_data_cb(buf: *const u8, len: u32) {
    if let Some(tx) = AUDIO_TX.get() {
        // Drop the packet if the writer can't keep up rather than block the BT stack.
        let _ = tx.try_send(Msg::Pcm(slice::from_raw_parts(buf, len as usize).to_vec()));
    }
}
