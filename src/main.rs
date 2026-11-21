
//! A2DP sink for the original ESP32: receives audio from a phone/PC over
//! Bluetooth Classic and plays it out through two TAS5754M amplifiers (main
//! L/R and woofer) fed by the same I2S line.
//!
//! The 2.1ch crossover (150 Hz, LR4) runs on the ESP32. Each 32-bit I2S slot
//! carries two 16-bit samples: the high-passed main signal in the upper half
//! and the low-passed mono woofer signal in the lower half. The main amplifier
//! reads the upper half; the woofer amplifier skips 16 SCLKs (P0-R41 I2S
//! shift) and reads the lower half.
//!
//! BT data callback -> bounded channel -> I2S writer thread, so a slow I2S
//! write never stalls the Bluetooth stack. A second thread reads the two
//! volume knobs (master, and bass relative to it) and writes the amplifiers'
//! digital volume over I2C.
// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Hiroshi Nakajima
#![allow(non_upper_case_globals)]

use std::ffi::CString;
use std::slice;
use std::sync::atomic::{AtomicBool, AtomicI32, AtomicU32, Ordering};
use std::sync::mpsc::{sync_channel, SyncSender};
use std::sync::{Arc, Mutex, OnceLock};
use std::thread;
use std::time::{Duration, Instant};

use anyhow::Result;
use esp_idf_svc::hal::cpu::Core;
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::hal::task::thread::ThreadSpawnConfiguration;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::sys::bt_shim::*;
use esp_idf_svc::sys::*;
use log::{error, info, warn};

mod bt;
mod clock;
mod config;
mod crossover;
mod net;
mod syslog;
mod usbpd;
mod web;

const DEFAULT_SAMPLE_RATE: u32 = 44_100;
// Crossover frequency between the main speakers and the woofer, Hz. Set from
// the `crossover_hz` setting; the writer thread picks up changes.
static CROSSOVER_HZ: AtomicU32 = AtomicU32::new(150);

pub fn set_crossover_hz(hz: u32) {
    CROSSOVER_HZ.store(hz, Ordering::Relaxed);
}
// Subsonic high-pass on the woofer in Hz (0 = off): below this the woofer
// cannot play, and the content only makes the cone buzz at high volume. Set
// from the `subsonic_hz` setting; the writer thread picks up changes.
static SUBSONIC_HZ: AtomicU32 = AtomicU32::new(30);

// Woofer bass EQ (peaking boost): centre frequency in Hz and boost in dB
// (0 = off). Set from the `bass_eq_hz` / `bass_eq_db` settings.
static BASS_EQ_HZ: AtomicU32 = AtomicU32::new(60);
static BASS_EQ_DB: AtomicU32 = AtomicU32::new(6);

pub fn set_bass_eq(hz: u32, db: u32) {
    BASS_EQ_HZ.store(hz, Ordering::Relaxed);
    BASS_EQ_DB.store(db, Ordering::Relaxed);
}

pub fn set_subsonic_hz(hz: u32) {
    SUBSONIC_HZ.store(hz, Ordering::Relaxed);
}

// Software gain in Q8 fixed point (256 = 1.0x). Samples are clamped, so large
// values clip loud material. Set from the web UI settings.
static GAIN_Q8: AtomicI32 = AtomicI32::new(256);
// Queue depth between the BT task and the I2S writer, in A2DP packets (about
// 4 KB / 23 ms each at 44.1 kHz). The I2S DMA buffers hold another ~170 ms.
// Kept small: WiFi leaves only ~40 KB of heap, and a full queue of large
// packets would exhaust it.
const QUEUE_DEPTH: usize = 4;

// I2S pins for the amplifiers. MCLK (256 x fs) must be GPIO0, 1 or 3 on the ESP32.
const PIN_BCK: i32 = 26;
const PIN_WS: i32 = 25;
const PIN_DOUT: i32 = 22;
const PIN_MCLK: i32 = 0;

// I2C bus shared by both amplifiers, with external pull-ups.
const PIN_SDA: i32 = 21;
const PIN_SCL: i32 = 18;
// The AP33772S USB PD sink controller (0x52) shares this bus; it supplies PVDD.
// The highest voltage the source offers up to the `pd_max_mv` setting is
// requested; the TAS5754M is rated for PVDD up to 26.4 V.
// How often the measured USB PD output is logged (and so sent to syslog).
const PD_LOG_INTERVAL: Duration = Duration::from_secs(60);
// How often usbpd::poll measures the USB PD output (and retries a failed
// negotiation every minute).
const PD_POLL_INTERVAL: Duration = Duration::from_secs(5);
// How often the USB PD output current is sampled for the average / maximum in
// the periodic log (one 1-byte I2C read each).
const CURRENT_SAMPLE_INTERVAL: Duration = Duration::from_millis(200);
// 7-bit addresses, set by the ADR0/ADR1 straps (0x98 / 0x9A in 8-bit form).
const AMP_MAIN: u8 = 0x4C; // U400: main L/R
const AMP_WOOFER: u8 = 0x4D; // U401: woofer

// SPK_MUTE of both amplifiers: high = play, low = soft mute. A 100k pull-down
// keeps them muted until the firmware drives the pin.
const PIN_MUTE: i32 = 13;
// SPK_FAULT of both amplifiers, wired together, active low (external pull-up).
const PIN_FAULT: i32 = 34;

// Volume knobs (ADC1): 10k potentiometers between 3.3 V and GND.
const PIN_VOL_MAIN: i32 = 33;
const PIN_VOL_WOOFER: i32 = 35;
const ADC_SAMPLES: i32 = 16;
// After a failed I2C access to a fitted amplifier it is left alone this long,
// so it is not hammered with transfers that are never acknowledged.
const AMP_RETRY: Duration = Duration::from_secs(5);
// Volume knobs are 10k B-taper (linear) pots: fraction of the full voltage at
// half rotation (0.5 for a linear B pot, 0.15 for the common 15 % A curve).
const POT_TAPER_MID: f32 = 0.5;
// Readings below ADC_MIN mute; a muted knob unmutes only above ADC_UNMUTE, so
// noise around the threshold cannot switch the sound on and off. The ESP32
// ADC reads ~0 below about 0.1 V, which with a B-taper pot is the first
// ~3 % of the rotation (~20 % with an A-taper pot). The ADC saturates near ADC_MAX.
// Knob movement (fraction of the rotation) needed before the volume follows.
const KNOB_HYSTERESIS: f32 = 0.01;
// A volume register is rewritten only when the target moves this many 0.5 dB
// steps away from it.
const VOL_HYSTERESIS: f32 = 1.0;
// Smoothing of the knob ADC readings (one-pole, per 20 ms poll).
const KNOB_SMOOTHING: f32 = 0.15;
// ADC counts (of 4095) a smoothed reading must move before the knob counts as
// turned.
const KNOB_RAW_HYSTERESIS: f32 = 16.0;
const KNOB_SETTLE: u32 = 2;
// While tuning ADC_MIN / ADC_UNMUTE: log the knob readings this often (None
// turns the log off).
const KNOB_LOG_INTERVAL: Option<Duration> = None;
// Bass knob: woofer level relative to the master volume, +-12 dB around
// mid rotation, in 0.5 dB steps.
const BASS_RANGE_STEPS: i32 = 24;
// Most woofer boost (0.5 dB steps) applied digitally when the amplifier is
// already at the PVDD ceiling; set by the controls thread.
const WOOFER_BOOST_MAX_STEPS: i32 = 12;
// Digital volume (linear gain as f32 bits) for the main and woofer paths, set
// by the controls thread and ramped by the audio path. 0 = muted.
static MAIN_VOL_GAIN: AtomicU32 = AtomicU32::new(0);
static WOOFER_VOL_GAIN: AtomicU32 = AtomicU32::new(0);
const ADC_MIN: i32 = 10;
const ADC_UNMUTE: i32 = 25;
const ADC_MAX: i32 = 4000;

// TAS5754M registers (page 0). Volume is 0.5 dB per step: 0x30 = 0 dB.
const REG_PAGE: u8 = 0x00;
const REG_BOOK: u8 = 0x7F;
const REG_MUTE: u8 = 0x03;
const REG_FORMAT: u8 = 0x28;
const REG_I2S_SHIFT: u8 = 0x29;
const REG_VOL_LINK: u8 = 0x3C;
const REG_VOL_B: u8 = 0x3D;
const REG_CLOCK_STATUS: u8 = 0x5E;
const VOL_0DB: i32 = 0x30;
// SCLKs the woofer amplifier skips to reach the lower half of each slot.
const WOOFER_SHIFT: u8 = 16;
/// true: 32-bit I2S slots carrying a separate low-passed woofer word (read by
/// the woofer amplifier through R41), so the crossover runs on the ESP32.
/// false: plain 16-bit I2S (BCK 32 fs), both amplifiers play the same full-range
/// stereo signal and the crossover is off.
const SPLIT_WOOFER: bool = true;
// The master knob spans 0 dB (fully clockwise) down by the `vol_range_db`
// setting, in 0.5 dB steps.
static VOL_RANGE_STEPS: AtomicI32 = AtomicI32::new(120);
// Speaker amplifier gain set by the SPK_GAIN/FREQ divider (150k from GVDD_REG,
// 750k to GND: ~5.8 V = 26 dBV mode). At 26.4 dBV digital full scale would
// need a ~30 V peak, more than PVDD, so the amplifier output clips on loud
// passages (heard as crackling, mostly on the woofer).
const AMP_GAIN_DBV: f32 = 26.427;
// Fraction of PVDD the BTL output can swing to before clipping (output stage
// drop under load).
const PVDD_SWING: f32 = 0.9;
// Extra attenuation (0.5 dB steps) applied on top of the knob so full scale
// stays below PVDD; recalculated from the measured PVDD.
static VOL_CEIL_STEPS: AtomicI32 = AtomicI32::new(0);

pub fn set_gain_q8(gain: i32) {
    GAIN_Q8.store(gain, Ordering::Relaxed);
}

pub fn set_vol_range_db(db: i32) {
    VOL_RANGE_STEPS.store(db * 2, Ordering::Relaxed);
}

enum Msg {
    Pcm(Vec<u8>),
    Rate(u32),
}

static AUDIO_TX: OnceLock<SyncSender<Msg>> = OnceLock::new();

fn main() -> Result<()> {
    link_patches();
    syslog::init();

    // Bluetooth and the settings are stored in NVS.
    let _nvs = EspDefaultNvsPartition::take()?;
    let cfg = Arc::new(Mutex::new(config::Config::load()));
    let (device_name, pd_max_mv) = {
        let c = cfg.lock().unwrap();
        clock::set_timezone(&c.timezone);
        set_gain_q8(c.gain_q8());
        set_vol_range_db(c.vol_range_db());
        set_subsonic_hz(c.subsonic_hz());
        set_crossover_hz(c.crossover_hz());
        set_bass_eq(c.bass_eq_hz(), c.bass_eq_db());
        (c.bt_name.clone(), c.pd_max_mv())
    };

    // Stay muted until the amplifiers are configured and audio starts.
    init_gpio()?;

    // The I2C interrupt is allocated on the core that installs the driver.
    // Keep it off CPU0, where the Bluetooth controller and WiFi run: there it
    // stalled in its handler and tripped the interrupt watchdog.
    ThreadSpawnConfiguration { name: Some(b"i2c-init\0"), pin_to_core: Some(Core::Core1), ..Default::default() }
        .set()?;
    let i2c = thread::spawn(|| esp!(unsafe { bt_shim_i2c_start(PIN_SDA, PIN_SCL) }));
    ThreadSpawnConfiguration::default().set()?;
    i2c.join().map_err(|_| anyhow::anyhow!("I2C init thread panicked"))??;

    // Bring up the supply first so PVDD is present when the amplifiers start.
    match usbpd::enable_output(pd_max_mv) {
        Ok(mv) => info!("USB PD output on at {mv} mV"),
        Err(e) => error!("USB PD output not enabled: {e}"),
    }
    esp!(unsafe { bt_shim_adc_add(PIN_VOL_MAIN) })?;
    esp!(unsafe { bt_shim_adc_add(PIN_VOL_WOOFER) })?;
    esp!(unsafe { bt_shim_i2s_start(DEFAULT_SAMPLE_RATE, PIN_BCK, PIN_WS, PIN_DOUT, PIN_MCLK, if SPLIT_WOOFER { 32 } else { 16 }) })?;

    // The amplifiers detect their clocks from MCLK / BCK / LRCK, so configure
    // them once the I2S clocks are running.
    thread::sleep(Duration::from_millis(100));
    // An amplifier that does not answer here is treated as not fitted and is
    // left alone until the next boot; the other one runs on its own.
    for (i, addr) in [AMP_MAIN, AMP_WOOFER].into_iter().enumerate() {
        match amp_init(addr) {
            Ok(()) => AMP_PRESENT[i].store(true, Ordering::Relaxed),
            Err(e) => warn!("amp {addr:#04x} not answering ({e}), running without it"),
        }
    }
    thread::Builder::new()
        .name("controls".into())
        .stack_size(4096)
        .spawn(controls_loop)?;

    let (tx, rx) = sync_channel::<Msg>(QUEUE_DEPTH);
    AUDIO_TX.set(tx).ok();
    thread::Builder::new()
        .name("i2s-writer".into())
        .stack_size(4096)
        .spawn(move || {
            // Without the woofer amplifier the main one plays the full range.
            let woofer = SPLIT_WOOFER && AMP_PRESENT[1].load(Ordering::Relaxed);
            if !SPLIT_WOOFER {
                info!("16-bit I2S: crossover off, both amplifiers play full range");
            } else if !woofer {
                warn!("woofer amplifier missing: crossover off, main amplifier plays full range");
            }
            let mut xover = crossover::Crossover::new(CROSSOVER_HZ.load(Ordering::Relaxed), SUBSONIC_HZ.load(Ordering::Relaxed), DEFAULT_SAMPLE_RATE);
            let mut limit_main = crossover::Limiter::new(DEFAULT_SAMPLE_RATE, crossover::CEILING);
            let mut limit_woofer = crossover::Limiter::new(DEFAULT_SAMPLE_RATE, crossover::CEILING);
            let mut out = Vec::new();
            // Gains applied at the end of the previous packet (main, woofer boost).
            let mut state = RenderState::new();
            for msg in rx {
                match msg {
                    Msg::Pcm(data) => {
                        xover.set_freq(CROSSOVER_HZ.load(Ordering::Relaxed));
                        xover.set_subsonic(SUBSONIC_HZ.load(Ordering::Relaxed));
                        xover.set_eq(BASS_EQ_HZ.load(Ordering::Relaxed), BASS_EQ_DB.load(Ordering::Relaxed));
                        out.clear();
                        if out.try_reserve(data.len() * 2).is_err() {
                            DROPPED_PACKETS.fetch_add(1, Ordering::Relaxed);
                            continue;
                        }
                        render(&data, &mut out, woofer.then_some(&mut xover), &mut state, &mut limit_main, &mut limit_woofer);
                        LIMITED[0].fetch_add(std::mem::take(&mut limit_main.engaged), Ordering::Relaxed);
                        LIMITED[1].fetch_add(std::mem::take(&mut limit_woofer.engaged), Ordering::Relaxed);
                        drop(data);
                        let mut off = 0;
                        while off < out.len() {
                            let n = unsafe { bt_shim_i2s_write(out[off..].as_ptr(), out.len() - off) };
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
                        xover.set_rate(rate);
                        limit_main.set_rate(rate);
                        limit_woofer.set_rate(rate);
                    }
                }
            }
        })?;

    esp!(unsafe { bt_shim_controller_start() })?;
    esp!(unsafe { esp_bluedroid_init() })?;
    esp!(unsafe { esp_bluedroid_enable() })?;

    let name = CString::new(device_name.as_str())?;
    esp!(unsafe { esp_bt_dev_set_device_name(name.as_ptr()) })?;
    esp!(unsafe { esp_bt_gap_register_callback(Some(gap_cb)) })?;

    esp!(unsafe { esp_a2d_register_callback(Some(a2d_cb)) })?;
    esp!(unsafe { esp_a2d_sink_register_data_callback(Some(a2d_data_cb)) })?;
    // Completion is reported via ESP_A2D_PROF_STATE_EVT in a2d_cb.
    esp!(unsafe { esp_a2d_sink_init() })?;

    info!("A2DP sink started as \"{device_name}\"");

    // Last: WiFi and the web UI take their memory after Bluetooth and I2S.
    net::start(cfg, Peripherals::take()?.modem)?;
    Ok(())
}

pub static PLAYING: AtomicBool = AtomicBool::new(false);
// Status for the web UI.
pub static BT_CONNECTED: AtomicBool = AtomicBool::new(false);
pub static SAMPLE_RATE: AtomicU32 = AtomicU32::new(0);
/// A2DP packets dropped because the queue was full or memory ran out.
pub static DROPPED_PACKETS: AtomicU32 = AtomicU32::new(0);
/// Samples at which the main / woofer limiter turned the level down.
static LIMITED: [AtomicU32; 2] = [AtomicU32::new(0), AtomicU32::new(0)];

/// Volume register of the main and woofer amplifier; VOLUME_MUTED at the
/// knob's minimum, VOLUME_UNKNOWN until written.
pub static VOLUME: [AtomicI32; 2] = [AtomicI32::new(VOLUME_UNKNOWN), AtomicI32::new(VOLUME_UNKNOWN)];
pub const VOLUME_MUTED: i32 = -1;
/// Main and woofer amplifier answered at boot.
pub static AMP_PRESENT: [AtomicBool; 2] = [AtomicBool::new(false), AtomicBool::new(false)];
pub const VOLUME_UNKNOWN: i32 = -2;

/// Volume in dB for a register value, None while muted or unknown.
pub fn volume_db(reg: i32) -> Option<f32> {
    (reg >= 0).then(|| (VOL_0DB - reg) as f32 / 2.0)
}

/// True while an amplifier pulls SPK_FAULT low.
pub fn amp_fault() -> bool {
    unsafe { gpio_get_level(PIN_FAULT) == 0 }
}

/// Sets up SPK_MUTE as an output driven low (muted) and SPK_FAULT as an input,
/// both without internal pulls (the board has its own). `gpio_config` also
/// routes the pads to the GPIO matrix: GPIO13 starts out as the JTAG MTCK
/// pin, where setting the level alone does not reach the pad.
fn init_gpio() -> Result<(), EspError> {
    let pin = |mode, gpio: i32| gpio_config_t {
        pin_bit_mask: 1u64 << gpio,
        mode,
        pull_up_en: gpio_pullup_t_GPIO_PULLUP_DISABLE,
        pull_down_en: gpio_pulldown_t_GPIO_PULLDOWN_DISABLE,
        intr_type: gpio_int_type_t_GPIO_INTR_DISABLE,
    };
    unsafe {
        // Low before the output is enabled, so the amplifiers never see a
        // high pulse.
        esp!(gpio_set_level(PIN_MUTE, 0))?;
        esp!(gpio_config(&pin(gpio_mode_t_GPIO_MODE_OUTPUT, PIN_MUTE)))?;
        esp!(gpio_config(&pin(gpio_mode_t_GPIO_MODE_INPUT, PIN_FAULT)))
    }
}

/// Drives SPK_MUTE of both amplifiers: low soft-mutes them, high plays.
fn set_amp_mute(mute: bool) -> Result<(), EspError> {
    esp!(unsafe { gpio_set_level(PIN_MUTE, if mute { 0 } else { 1 }) })
}

fn amp_write(addr: u8, reg: u8, val: u8) -> Result<(), EspError> {
    esp!(unsafe { bt_shim_i2c_write_reg(addr, reg, val) })
}

fn amp_read(addr: u8, reg: u8) -> Result<u8, EspError> {
    let mut val = 0u8;
    esp!(unsafe { bt_shim_i2c_read_reg(addr, reg, &mut val) })?;
    Ok(val)
}

/// Sets I2S 16-bit input and links both channels to one volume register. The
/// volume starts at -60 dB until the knob has been read.
fn amp_init(addr: u8) -> Result<(), EspError> {
    amp_write(addr, REG_PAGE, 0x00)?;
    amp_write(addr, REG_BOOK, 0x00)?;
    amp_write(addr, REG_FORMAT, 0x00)?; // I2S, 16-bit words (in 32-bit slots)
    amp_write(addr, REG_I2S_SHIFT, if SPLIT_WOOFER && addr == AMP_WOOFER { WOOFER_SHIFT } else { 0 })?;
    amp_write(addr, REG_VOL_LINK, 0x01)?; // channel A follows channel B
    amp_write(addr, REG_VOL_B, (VOL_0DB + VOL_RANGE_STEPS.load(Ordering::Relaxed)) as u8)?;
    let status = amp_read(addr, REG_CLOCK_STATUS)?;
    info!("amp {addr:#04x} ready, clock status {status:#04x}");
    Ok(())
}

/// Knob rotation (0..1) from a pot reading. For a B-taper (linear) pot it is
/// the reading itself. For an A-taper (audio, log) pot the
/// wiper voltage follows v = (A^x - 1) / (A - 1), where A is chosen so the
/// voltage is POT_TAPER_MID at half rotation; this inverts it. The volume in
/// dB is then linear in rotation, instead of crowding into the last half.
fn knob_rotation(raw: i32) -> f32 {
    let v = (raw as f32 / ADC_MAX as f32).clamp(0.0, 1.0);
    let m = POT_TAPER_MID;
    if (m - 0.5).abs() < 0.01 {
        return v; // linear (B) taper
    }
    // v(0.5) = 1 / (sqrt(A) + 1) = m  =>  A = ((1 - m) / m)^2
    let a = ((1.0 - m) / m).powi(2);
    (1.0 + v * (a - 1.0)).ln() / a.ln()
}

/// Knob position 0..1, counted from the unmute point (between ADC_MIN and
/// ADC_UNMUTE it stays 0).
fn knob_position(raw: i32) -> f32 {
    let low = knob_rotation(ADC_UNMUTE);
    ((knob_rotation(raw) - low) / (1.0 - low)).clamp(0.0, 1.0)
}

/// Sets the volume ceiling for a measured PVDD so the amplifier output
/// cannot clip. A one-step change upward is ignored so measurement noise does
/// not keep moving it.
fn update_volume_ceiling(pvdd_mv: u16) {
    if pvdd_mv < 1000 {
        return;
    }
    let full_scale_peak = 10f32.powf(AMP_GAIN_DBV / 20.0) * std::f32::consts::SQRT_2;
    let db = 20.0 * (PVDD_SWING * pvdd_mv as f32 / 1000.0 / full_scale_peak).log10();
    let steps = ((-db * 2.0).ceil() as i32).max(0);
    let old = VOL_CEIL_STEPS.load(Ordering::Relaxed);
    if steps > old || steps < old - 1 {
        VOL_CEIL_STEPS.store(steps, Ordering::Relaxed);
        info!("PVDD {pvdd_mv} mV: maximum volume {:.1} dB", -steps as f32 / 2.0);
    }
}

/// A volume knob, read with hysteresis so ADC noise does not move it.
struct Knob {
    pin: i32,
    // Position 0..1, None in the mute zone; None until first read.
    pos: Option<f32>,
    read: bool,
    // Smoothed ADC reading.
    filtered: Option<f32>,
    // Reading in use, changed only past KNOB_RAW_HYSTERESIS.
    held: Option<i32>,
    // Consecutive polls the smoothed reading has been past the hysteresis.
    away: u32,
    // Last ADC reading (trimmed mean), for the knob log.
    last_raw: i32,
}

impl Knob {
    fn new(pin: i32) -> Self {
        Self { pin, pos: None, read: false, filtered: None, held: None, away: 0, last_raw: -1 }
    }

    /// Reads the ADC; returns the (filtered) position, or None when muted.
    fn update(&mut self) -> Option<f32> {
        let raw = unsafe { bt_shim_adc_read_avg(self.pin, ADC_SAMPLES) };
        self.last_raw = raw;
        if raw >= 0 {
            let raw = match self.filtered {
                Some(f) => f + (raw as f32 - f) * KNOB_SMOOTHING,
                None => raw as f32,
            };
            self.filtered = Some(raw);
            // Hold the reading until it moves KNOB_RAW_HYSTERESIS away, so ADC
            // noise never reaches the volume (with an A-taper pot the inversion
            // multiplies it near the bottom of the knob).
            // A move must also last KNOB_SETTLE polls in a row, so a single
            // noise burst is ignored.
            let raw = match self.held {
                Some(h) if (raw - h as f32).abs() < KNOB_RAW_HYSTERESIS => {
                    self.away = 0;
                    h
                }
                Some(h) if self.away + 1 < KNOB_SETTLE => {
                    self.away += 1;
                    h
                }
                _ => {
                    self.away = 0;
                    raw.round() as i32
                }
            };
            self.held = Some(raw);
            // Mute below ADC_MIN; once muted, stay muted up to ADC_UNMUTE.
            let muted = raw < ADC_MIN || (self.read && self.pos.is_none() && raw < ADC_UNMUTE);
            let pos = (!muted).then(|| knob_position(raw));
            let moved = match (pos, self.pos) {
                (Some(p), Some(l)) => (p - l).abs() >= KNOB_HYSTERESIS || (p == 0.0 || p == 1.0) && p != l,
                (None, None) => false,
                _ => true,
            };
            if moved || !self.read {
                self.pos = pos;
                self.read = true;
            }
        }
        self.pos
    }

    /// Current readings for the knob log: raw ADC, smoothed, held, position.
    fn describe(&self) -> String {
        let pos = self.pos.map_or("mute".to_string(), |p| format!("{:.3}", p));
        format!(
            "raw {} filt {:.0} held {} pos {pos}",
            self.last_raw,
            self.filtered.unwrap_or(-1.0),
            self.held.unwrap_or(-1)
        )
    }
}

/// Writes one amplifier's volume register when its target changes.
struct AmpOut {
    amp: u8,
    // Last volume register written, None while muted.
    last: Option<u8>,
    known: bool,
    // No I2C access before this instant, after a failure.
    retry_at: Option<Instant>,
    error_logged: bool,
}

impl AmpOut {
    fn new(amp: u8) -> Self {
        Self { amp, last: None, known: false, retry_at: None, error_logged: false }
    }

    /// `target` is the volume register as a fraction; it is rounded and written
    /// only once it is VOL_HYSTERESIS away from the register, so knob noise
    /// around a step boundary does not rewrite the register every cycle.
    fn apply(&mut self, target: Option<f32>) {
        if self.retry_at.is_some_and(|t| Instant::now() < t) {
            return;
        }
        let vol = match (target, self.last) {
            (Some(t), Some(l)) if self.known && (t - l as f32).abs() < VOL_HYSTERESIS => return,
            // Move one 0.5 dB step per poll (25 dB/s): a jump of several dB
            // at once clicks audibly while the knob turns.
            (Some(t), Some(l)) if self.known => Some(if t > l as f32 { l + 1 } else { l - 1 }),
            (None, None) if self.known => return,
            (t, _) => t.map(|t| t.round() as u8),
        };
        let result = match vol {
            Some(v) if self.known && self.last.is_some() => amp_write(self.amp, REG_VOL_B, v),
            Some(v) => amp_write(self.amp, REG_VOL_B, v).and_then(|_| amp_write(self.amp, REG_MUTE, 0x00)),
            None => amp_write(self.amp, REG_MUTE, 0x11),
        };
        match result {
            Ok(()) => {
                self.retry_at = None;
                self.known = true;
                self.last = vol;
                if self.error_logged {
                    info!("amp {:#04x} answering again", self.amp);
                    self.error_logged = false;
                }
            }
            Err(e) => {
                self.retry_at = Some(Instant::now() + AMP_RETRY);
                if !self.error_logged {
                    warn!("amp {:#04x} not answering ({e}), retrying every {} s", self.amp, AMP_RETRY.as_secs());
                    self.error_logged = true;
                }
            }
        }
    }
}

/// Volume for both amplifiers. The main knob is the master volume for both.
/// The bass knob sets the woofer level against it:
/// - lower half: from -12 dB (BASS_RANGE_STEPS) below the master up to the
///   master level at mid rotation (fully down mutes the woofer);
/// - upper half: from the master level up to the woofer's full output at the
///   top, whatever the master is, so the woofer can play at full even with the
///   main speakers quiet. Full output is the PVDD ceiling plus up to
///   WOOFER_BOOST_MAX_STEPS of digital boost (the woofer limiter holds peaks).
///
/// The volume itself is digital (MAIN_VOL_GAIN / WOOFER_VOL_GAIN, ramped per
/// sample by the audio path) so turning a knob never clicks; the amplifier
/// registers stay at the PVDD ceiling and only mute. Returns the register
/// targets for (main, woofer), None = mute.
fn volume_targets(master: Option<f32>, bass: Option<f32>) -> (Option<f32>, Option<f32>) {
    let top = (VOL_0DB + VOL_CEIL_STEPS.load(Ordering::Relaxed)) as f32;
    let range = VOL_RANGE_STEPS.load(Ordering::Relaxed) as f32;
    let set = |gain: &AtomicU32, vol: &AtomicI32, steps: Option<f32>| {
        // 0.5 dB per step below the ceiling register.
        let g = steps.map_or(0.0, |s| 10f32.powf(-(s - top) / 40.0));
        gain.store(g.to_bits(), Ordering::Relaxed);
        vol.store(steps.map_or(VOLUME_MUTED, |s| s.round() as i32), Ordering::Relaxed);
    };
    let main = master.map(|m| top + (1.0 - m) * range);
    let full = top - WOOFER_BOOST_MAX_STEPS as f32;
    let woofer = main.zip(bass).map(|(main, b)| {
        let w = if b < 0.5 {
            main + (0.5 - b) * 2.0 * BASS_RANGE_STEPS as f32
        } else {
            main + (full - main) * (b - 0.5) * 2.0
        };
        w.clamp(full, 254.0)
    });
    if SPLIT_WOOFER {
        set(&MAIN_VOL_GAIN, &VOLUME[0], main);
        set(&WOOFER_VOL_GAIN, &VOLUME[1], woofer);
        (main.map(|_| top), woofer.map(|_| top))
    } else {
        // Both amplifiers get the same data: the main volume is digital, the
        // woofer register sets the woofer's level relative to it.
        set(&MAIN_VOL_GAIN, &VOLUME[0], main);
        set(&WOOFER_VOL_GAIN, &VOLUME[1], woofer);
        let rel = main.zip(woofer).map(|(m, w)| (top + w - m).clamp(top, 254.0));
        (main.map(|_| top), rel)
    }
}

/// Polls the volume knobs and SPK_FAULT, and logs the USB PD output every
/// PD_LOG_INTERVAL. A latched fault is cleared by toggling SPK_MUTE.
fn controls_loop() {
    let mut master = Knob::new(PIN_VOL_MAIN);
    let mut bass = Knob::new(PIN_VOL_WOOFER);
    // Only the amplifiers that answered at boot are written.
    let mut outs = [(AMP_MAIN, 0), (AMP_WOOFER, 1)]
        .map(|(amp, i)| AMP_PRESENT[i].load(Ordering::Relaxed).then(|| AmpOut::new(amp)));
    let mut last_clear: Option<Instant> = None;
    let mut next_pd_log = Instant::now() + PD_LOG_INTERVAL;
    // Measure PVDD right away so the volume ceiling is set before playback.
    let mut next_pd_poll = Instant::now();
    let mut next_current = Instant::now();
    let mut next_knob_log = Instant::now();
    loop {
        bt::tick();
        if Instant::now() >= next_pd_poll {
            usbpd::poll();
            if let Some(r) = usbpd::measure() {
                update_volume_ceiling(r.voltage_mv);
            }
            // A retried negotiation can take seconds; do not catch up after it.
            next_pd_poll = Instant::now() + PD_POLL_INTERVAL;
        }
        if Instant::now() >= next_current {
            usbpd::sample_current();
            next_current = Instant::now() + CURRENT_SAMPLE_INTERVAL;
        }
        if Instant::now() >= next_pd_log {
            usbpd::log_output();
            let dropped = DROPPED_PACKETS.swap(0, Ordering::Relaxed);
            if dropped > 0 {
                warn!("audio: {dropped} packets dropped in the last {} s", PD_LOG_INTERVAL.as_secs());
            }
            let main = LIMITED[0].swap(0, Ordering::Relaxed);
            let woofer = LIMITED[1].swap(0, Ordering::Relaxed);
            if main > 0 || woofer > 0 {
                info!(
                    "audio: limiter engaged on {main} main / {woofer} woofer samples in the last {} s",
                    PD_LOG_INTERVAL.as_secs()
                );
            }
            // The DMA also underruns while idle; count only during playback.
            let underruns = unsafe { bt_shim_i2s_underruns() };
            if underruns > 0 && PLAYING.load(Ordering::Relaxed) {
                warn!("audio: {underruns} I2S underruns in the last {} s", PD_LOG_INTERVAL.as_secs());
            }
            next_pd_log += PD_LOG_INTERVAL;
        }
        let (main_vol, woofer_vol) = volume_targets(master.update(), bass.update());
        if let Some(interval) = KNOB_LOG_INTERVAL {
            if Instant::now() >= next_knob_log {
                info!("knobs: main {}, woofer {}", master.describe(), bass.describe());
                next_knob_log = Instant::now() + interval;
            }
        }
        for (out, vol) in outs.iter_mut().zip([main_vol, woofer_vol]) {
            if let Some(out) = out {
                out.apply(vol);
            }
        }
        if amp_fault() {
            let due = last_clear.map_or(true, |t| t.elapsed() >= Duration::from_secs(1));
            if due {
                warn!("amplifier fault (SPK_FAULT low), toggling SPK_MUTE");
                let _ = set_amp_mute(true);
                thread::sleep(Duration::from_millis(20));
                let _ = set_amp_mute(!PLAYING.load(Ordering::Relaxed));
                last_clear = Some(Instant::now());
            }
        } else {
            last_clear = None;
        }
        thread::sleep(Duration::from_millis(20));
    }
}

/// State the audio path keeps from one packet to the next.
struct RenderState {
    // Gains reached at the end of the previous packet: input gain, main volume,
    // woofer volume.
    gains: [f32; 3],
    // Dither noise generator (xorshift32).
    rng: u32,
}

impl RenderState {
    fn new() -> Self {
        Self { gains: [GAIN_Q8.load(Ordering::Relaxed) as f32 / 256.0, 0.0, 0.0], rng: 0x1234_5678 }
    }

    /// TPDF dither, -1..+1 LSB.
    #[inline]
    fn dither(&mut self) -> f32 {
        let mut next = || {
            self.rng ^= self.rng << 13;
            self.rng ^= self.rng >> 17;
            self.rng ^= self.rng << 5;
            (self.rng >> 8) as f32 * (1.0 / 16_777_216.0)
        };
        next() - next()
    }
}

/// Converts interleaved little-endian 16-bit stereo PCM into I2S slots:
/// input gain, crossover, digital volume (main / woofer), limiters, then
/// dithered rounding to 16 bits. With SPLIT_WOOFER each 32-bit slot carries
/// the main sample in the upper and the woofer sample in the lower half;
/// otherwise plain 16-bit stereo.
fn render(
    pcm: &[u8],
    out: &mut Vec<u8>,
    mut xover: Option<&mut crossover::Crossover>,
    st: &mut RenderState,
    limit_main: &mut crossover::Limiter,
    limit_woofer: &mut crossover::Limiter,
) {
    // Gain and volume changes ramp linearly across the packet instead of
    // stepping at its start, which would put a step into the waveform.
    let target = [
        GAIN_Q8.load(Ordering::Relaxed) as f32 / 256.0,
        f32::from_bits(MAIN_VOL_GAIN.load(Ordering::Relaxed)),
        f32::from_bits(WOOFER_VOL_GAIN.load(Ordering::Relaxed)),
    ];
    let start = std::mem::replace(&mut st.gains, target);
    let frames = (pcm.len() / 4).max(1) as f32;
    let step = [0, 1, 2].map(|k| (target[k] - start[k]) / frames);
    // Round to the nearest 16-bit value (floor of x + 0.5, kept positive).
    let q = |v: f32| ((v.clamp(-32768.0, 32767.0) + 32768.5) as i32 - 32768).min(32767);
    for (i, f) in pcm.chunks_exact(4).enumerate() {
        let n = (i + 1) as f32;
        let [gain, vol_main, vol_woofer] = [0, 1, 2].map(|k| start[k] + step[k] * n);
        let (li, ri) = (i16::from_le_bytes([f[0], f[1]]) as i32, i16::from_le_bytes([f[2], f[3]]) as i32);
        let (l, r) = (li as f32 * gain, ri as f32 * gain);
        let (ml, mr, w) = match xover.as_deref_mut() {
            Some(x) => x.run(l, r),
            None => (l, r, 0.0),
        };
        let (ml, mr, w) = (ml * vol_main, mr * vol_main, w * vol_woofer);
        let gm = limit_main.gain(ml.abs().max(mr.abs()));
        let gw = limit_woofer.gain(w.abs());
        let mli = q(ml * gm + st.dither());
        let mri = q(mr * gm + st.dither());
        let wi = q(w * gw + st.dither());
        let word = |v: i32| v as i16 as u16 as u32;
        if SPLIT_WOOFER {
            out.extend_from_slice(&((word(mli) << 16) | word(wi)).to_le_bytes());
            out.extend_from_slice(&((word(mri) << 16) | word(wi)).to_le_bytes());
        } else {
            out.extend_from_slice(&(mli as i16).to_le_bytes());
            out.extend_from_slice(&(mri as i16).to_le_bytes());
        }
    }
}

unsafe extern "C" fn gap_cb(event: esp_bt_gap_cb_event_t, param: *mut esp_bt_gap_cb_param_t) {
    let param = &mut *param;
    match event {
        esp_bt_gap_cb_event_t_ESP_BT_GAP_AUTH_CMPL_EVT => {
            let auth = &param.auth_cmpl;
            if auth.stat == esp_bt_status_t_ESP_BT_STATUS_SUCCESS {
                let name = std::ffi::CStr::from_ptr(auth.device_name.as_ptr() as *const _);
                bt::on_authenticated(auth.bda, &name.to_string_lossy());
            } else {
                warn!("authentication failed, status {}", auth.stat);
            }
        }
        esp_bt_gap_cb_event_t_ESP_BT_GAP_READ_REMOTE_NAME_EVT => {
            let r = &param.read_rmt_name;
            if r.stat == esp_bt_status_t_ESP_BT_STATUS_SUCCESS {
                let name = std::ffi::CStr::from_ptr(r.rmt_name.as_ptr() as *const _);
                bt::on_remote_name(r.bda, &name.to_string_lossy());
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
            if param.a2d_prof_stat.init_state == esp_a2d_init_state_t_ESP_A2D_INIT_SUCCESS {
                bt::on_ready();
            }
        }
        esp_a2d_cb_event_t_ESP_A2D_CONNECTION_STATE_EVT => {
            let state = param.conn_stat.state;
            let bda = param.conn_stat.remote_bda;
            if state == esp_a2d_connection_state_t_ESP_A2D_CONNECTION_STATE_CONNECTED {
                BT_CONNECTED.store(true, Ordering::Relaxed);
                bt::on_connected(bda);
            } else if state == esp_a2d_connection_state_t_ESP_A2D_CONNECTION_STATE_DISCONNECTED {
                BT_CONNECTED.store(false, Ordering::Relaxed);
                PLAYING.store(false, Ordering::Relaxed);
                let _ = set_amp_mute(true);
                bt::on_disconnected(bda);
            }
        }
        esp_a2d_cb_event_t_ESP_A2D_AUDIO_STATE_EVT => {
            let started = param.audio_stat.state == esp_a2d_audio_state_t_ESP_A2D_AUDIO_STATE_STARTED;
            info!("audio state {}", param.audio_stat.state);
            PLAYING.store(started, Ordering::Relaxed);
            if let Err(e) = set_amp_mute(!started) {
                warn!("amp mute failed: {e}");
            }
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
                SAMPLE_RATE.store(rate, Ordering::Relaxed);
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
    let Some(tx) = AUDIO_TX.get() else { return };
    // Drop the packet rather than block the BT stack when the writer can't keep
    // up, and rather than abort when the heap is short.
    let data = slice::from_raw_parts(buf, len as usize);
    let mut pcm = Vec::new();
    if pcm.try_reserve_exact(data.len()).is_err() {
        DROPPED_PACKETS.fetch_add(1, Ordering::Relaxed);
        return;
    }
    pcm.extend_from_slice(data);
    if tx.try_send(Msg::Pcm(pcm)).is_err() {
        DROPPED_PACKETS.fetch_add(1, Ordering::Relaxed);
    }
}
