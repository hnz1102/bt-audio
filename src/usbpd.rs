// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Hiroshi Nakajima

//! AP33772S USB PD sink controller on the shared I2C bus. The sequence follows
//! the wrapper in uv-controller: read the source PDOs, enable the protections,
//! request the voltage, and switch VOUT on once the negotiation succeeded.
//!
//! After start-up only the `controls` thread talks to the controller, through
//! `poll`: it caches the measured output for the log and the web UI and
//! retries a failed negotiation. Keeping the accesses few and on one thread
//! matters: the controller does not answer on I2C while it negotiates. A PPS /
//! AVS contract needs no keep-alive from here: the controller repeats the
//! request itself (21 V stayed up well past the source's 15 s PPS timeout).

use std::sync::atomic::{AtomicU16, AtomicU32, Ordering};
use std::sync::Mutex;
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{anyhow, bail, Result};
use ap33772s_driver::{PDOInfo, PDStatus, AP33772S, AP33772S_ADDR, REG_CURRENT, REG_PD_REQMSG, STATUS_READY};
use embedded_hal::i2c::{Error, ErrorKind, ErrorType, I2c, Operation};
use esp_idf_svc::sys::bt_shim::*;
use esp_idf_svc::sys::{esp, EspError};
use log::{info, warn};

static OUTPUT_MV: AtomicU16 = AtomicU16::new(0);
static LIMIT_MV: AtomicU16 = AtomicU16::new(0);
static POLLS: AtomicU32 = AtomicU32::new(0);
// How long the source may take to settle on the requested voltage.
const CONTRACT_TIMEOUT: Duration = Duration::from_secs(5);
// How long to wait for the source to start USB PD at boot, and on a retry.
const PD_WAIT_BOOT: Duration = Duration::from_secs(5);
const PD_WAIT_RETRY: Duration = Duration::from_secs(1);
// A failed negotiation is retried every this many polls (one minute).
const RETRY_POLLS: u32 = 12;
// OPMODE bit set while the source talks USB PD (0x01: legacy Type-C source).
const OPMODE_PD: u8 = 0x02;
// Current samples (CURRENT register, 24 mA per LSB) since the last log line,
// taken by `sample_current` several times a second so peaks during playback
// are not missed.
static CUR_SUM: AtomicU32 = AtomicU32::new(0);
static CUR_COUNT: AtomicU32 = AtomicU32::new(0);
static CUR_MAX: AtomicU32 = AtomicU32::new(0);
const CURRENT_LSB_MA: u32 = 24;
/// Latest measurement, refreshed by `poll`.
static READING: Mutex<Option<Reading>> = Mutex::new(None);
/// The controller after `enable_output`, kept to read its status later.
static PD: Mutex<Option<AP33772S>> = Mutex::new(None);
/// How the last negotiation ended, for `log_status` (the console log of the
/// negotiation itself is not forwarded to syslog).
static NOTE: Mutex<String> = Mutex::new(String::new());

/// Voltage requested from the source in mV, or 0 while VOUT is off.
pub fn output_mv() -> u16 {
    OUTPUT_MV.load(Ordering::Relaxed)
}

// The error is only reported through Debug.
#[derive(Debug)]
struct I2cError(#[allow(dead_code)] EspError);

impl Error for I2cError {
    fn kind(&self) -> ErrorKind {
        ErrorKind::Other
    }
}

/// The bus started by `bt_shim_i2c_start`, seen through embedded-hal.
struct ShimI2c;

impl ErrorType for ShimI2c {
    type Error = I2cError;
}

impl I2c for ShimI2c {
    fn transaction(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<(), I2cError> {
        for op in operations {
            match op {
                Operation::Write(bytes) => esp!(unsafe { bt_shim_i2c_write(address, bytes.as_ptr(), bytes.len()) }),
                Operation::Read(buf) => esp!(unsafe { bt_shim_i2c_read(address, buf.as_mut_ptr(), buf.len()) }),
            }
            .map_err(I2cError)?;
        }
        Ok(())
    }
}

/// The PDO giving the highest voltage in mV, not above `limit_mv`: a fixed
/// PDO gives its own voltage, a variable (PPS / AVS) PDO any voltage up to its
/// maximum. On a tie the fixed PDO wins.
fn best_pdo(pdos: &[PDOInfo], limit_mv: u16) -> Option<(PDOInfo, u16)> {
    pdos.iter()
        .map(|pdo| (*pdo, if pdo.is_fixed { pdo.voltage_mv } else { pdo.voltage_mv.min(limit_mv) }))
        .filter(|&(_, mv)| mv <= limit_mv)
        .max_by_key(|&(pdo, mv)| (mv, pdo.is_fixed))
}

/// PD_REQMSG word: PDO index, maximum current, and for a variable PDO the
/// voltage (100 mV units for SPR PPS, 200 mV for EPR AVS), as in the driver.
fn request_word(pdo: &PDOInfo, voltage_mv: u16) -> u16 {
    let voltage = match (pdo.is_fixed, pdo.pdo_index) {
        (true, _) => 0xFF,
        (false, 1..=7) => (voltage_mv / 100).min(0xFF),
        (false, _) => (voltage_mv / 200).min(0xFF),
    };
    ((pdo.pdo_index as u16 & 0x0F) << 12) | (0x0F << 8) | voltage
}

fn write_request(word: u16) -> Result<(), EspError> {
    let [lsb, msb] = word.to_le_bytes();
    let buf = [REG_PD_REQMSG, lsb, msb];
    esp!(unsafe { bt_shim_i2c_write(AP33772S_ADDR, buf.as_ptr(), buf.len()) })
}

/// Call every PD_POLL_INTERVAL (5 s) from one thread. Reads the output into the
/// cache, and retries a failed negotiation every RETRY_POLLS polls.
pub fn poll() {
    let n = POLLS.fetch_add(1, Ordering::Relaxed);
    let mut guard = PD.lock().unwrap();
    let Some(pd) = guard.as_mut() else { return };
    if OUTPUT_MV.load(Ordering::Relaxed) == 0 && n % RETRY_POLLS == RETRY_POLLS - 1 {
        info!("PD: retrying the negotiation");
        if let Err(e) = negotiate(pd, LIMIT_MV.load(Ordering::Relaxed), PD_WAIT_RETRY) {
            *NOTE.lock().unwrap() = format!("failed: {e}");
            warn!("PD: {e}");
        }
        return;
    }
    let reading = pd.get_status(&mut ShimI2c).ok().map(|s| Reading {
        voltage_mv: s.voltage_mv,
        current_ma: s.current_ma,
        fault: s.has_fault.then(|| format!("{:?}", s.fault_type)),
    });
    *READING.lock().unwrap() = reading;
}

/// Reads only the output current, for the average / maximum in the log. Call
/// often (a few times a second) from the same thread as `poll`.
pub fn sample_current() {
    let guard = PD.lock().unwrap();
    if guard.is_none() {
        return;
    }
    let mut raw = 0u8;
    if esp!(unsafe { bt_shim_i2c_read_reg(AP33772S_ADDR, REG_CURRENT, &mut raw) }).is_ok() {
        let ma = raw as u32 * CURRENT_LSB_MA;
        CUR_SUM.fetch_add(ma, Ordering::Relaxed);
        CUR_COUNT.fetch_add(1, Ordering::Relaxed);
        CUR_MAX.fetch_max(ma, Ordering::Relaxed);
    }
}

/// Negotiates the highest voltage the source offers up to `max_voltage_mv` and
/// turns VOUT on, returning the requested voltage in mV. VOUT is left off when
/// no voltage is available or the request fails.
pub fn enable_output(max_voltage_mv: u16) -> Result<u16> {
    LIMIT_MV.store(max_voltage_mv, Ordering::Relaxed);
    let mut pd = AP33772S::new();
    let result = negotiate(&mut pd, max_voltage_mv, PD_WAIT_BOOT);
    if let Err(e) = &result {
        *NOTE.lock().unwrap() = format!("failed: {e}");
    }
    *PD.lock().unwrap() = Some(pd);
    result
}

/// Logs the latest measured output and how the last negotiation ended.
pub fn log_status() {
    log_output();
    let note = NOTE.lock().unwrap();
    if !note.is_empty() {
        info!("PD negotiation: {note}");
    }
}

/// Measured output of the controller: voltage (mV), current (mA) and the
/// protection fault, if any.
#[derive(Clone)]
pub struct Reading {
    pub voltage_mv: u16,
    pub current_ma: u16,
    pub fault: Option<String>,
}

/// The latest measurement (at most a few seconds old); None before the first
/// one or when the controller did not answer.
pub fn measure() -> Option<Reading> {
    READING.lock().unwrap().clone()
}

/// Logs the latest measured voltage and the average / maximum current since
/// the previous call (falls back to the latest single reading when no samples
/// were taken), as a warning when a protection fault is flagged. Silent before
/// the first measurement.
pub fn log_output() {
    let count = CUR_COUNT.swap(0, Ordering::Relaxed);
    let sum = CUR_SUM.swap(0, Ordering::Relaxed);
    let max = CUR_MAX.swap(0, Ordering::Relaxed);
    let Some(r) = measure() else { return };
    let current = if count > 0 {
        format!("{} mA avg / {max} mA max ({count} samples)", sum / count)
    } else {
        format!("{} mA", r.current_ma)
    };
    match r.fault {
        Some(fault) => warn!("PD {} mV, {current}, fault {fault}", r.voltage_mv),
        None => info!("PD {} mV, {current}", r.voltage_mv),
    }
}

/// The measured output and the raw status registers; the voltage and current
/// readings have a resolution of 80 mV and 24 mA.
fn describe(s: &PDStatus) -> String {
    let fault = if s.has_fault { format!(", fault {:?}", s.fault_type) } else { String::new() };
    format!(
        "{} mV, {} mA, {} C (requested {} mV, source limit {} W; status {:#04x}, opmode {:#04x}, cc {:#04x}{fault})",
        s.voltage_mv, s.current_ma, s.temperature, s.requested_voltage_mv, s.pdp_limit_w,
        s.status, s.op_mode, s.cc_status
    )
}

/// Waits for VOUT to settle and checks that it reached at least 85 % of the
/// requested voltage, appending what was seen to `note`.
fn vout_rose(pd: &AP33772S, i2c: &mut ShimI2c, target_mv: u16, mode: &str, note: &mut String) -> bool {
    thread::sleep(Duration::from_millis(300));
    match pd.get_status(i2c) {
        Ok(s) => {
            let line = describe(&s);
            note.push_str(&format!(" | after {mode}: {line}"));
            let rose = s.voltage_mv as u32 * 100 >= target_mv as u32 * 85;
            if rose {
                info!("PD output {line}");
            } else {
                warn!("PD output below {target_mv} mV after {mode}: {line}");
            }
            rose
        }
        Err(e) => {
            note.push_str(&format!(" | after {mode}: status not read ({e:?})"));
            warn!("AP33772S status not read: {e:?}");
            false
        }
    }
}

/// Reads the source's PDOs once the source talks USB PD; a source that has
/// just been connected (or reset) may still be in its Type-C 5 V state.
fn wait_for_pd(pd: &mut AP33772S, i2c: &mut ShimI2c, wait: Duration) -> Result<()> {
    let start = Instant::now();
    loop {
        let state = match pd.init(i2c) {
            Ok(()) => match pd.get_status(i2c) {
                Ok(s) if s.op_mode & OPMODE_PD != 0 && !pd.get_pdo_list().is_empty() => return Ok(()),
                Ok(s) => format!("opmode {:#04x}, {} PDOs", s.op_mode, pd.get_pdo_list().len()),
                Err(e) => format!("status not read ({e:?})"),
            },
            Err(e) => format!("init failed ({e:?})"),
        };
        if start.elapsed() >= wait {
            bail!("the source does not offer USB PD ({state})");
        }
        thread::sleep(Duration::from_millis(500));
    }
}

fn negotiate(pd: &mut AP33772S, max_voltage_mv: u16, pd_wait: Duration) -> Result<u16> {
    let mut i2c = ShimI2c;

    wait_for_pd(pd, &mut i2c, pd_wait)?;
    for pdo in pd.get_pdo_list() {
        info!(
            "PDO {}: {} mV, {} mA, {} mW, {}",
            pdo.pdo_index,
            pdo.voltage_mv,
            pdo.current_ma,
            pdo.max_power_mw,
            if pdo.is_fixed { "fixed" } else { "variable" }
        );
    }
    let (pdo, voltage_mv) = best_pdo(pd.get_pdo_list(), max_voltage_mv)
        .ok_or_else(|| anyhow!("the source offers no voltage up to {max_voltage_mv} mV"))?;
    info!(
        "selected {voltage_mv} mV from PDO {} ({}), limit {max_voltage_mv} mV",
        pdo.pdo_index,
        if pdo.is_fixed { "fixed" } else { "variable" }
    );

    // UVP, OVP and OCP on; OTP and data role swap off (no NTC fitted).
    if let Err(e) = pd.configure_protections(&mut i2c, true, true, true, false, false) {
        warn!("AP33772S protections not configured: {e:?}");
    }
    OUTPUT_MV.store(0, Ordering::Relaxed);
    pd.force_vout_off(&mut i2c).map_err(|e| anyhow!("VOUT off failed: {e:?}"))?;
    let mut note = format!("requested {voltage_mv} mV");
    // The request is written directly instead of through the driver, whose
    // wait reads the controller while it is negotiating (it does not answer on
    // I2C then) and also fails on a UVP flag raised while VOUT is off.
    let word = request_word(&pdo, voltage_mv);
    write_request(word).map_err(|e| anyhow!("{voltage_mv} mV request failed: {e}"))?;
    // Poll the controller's own state until it reports the contract.
    let start = Instant::now();
    thread::sleep(Duration::from_millis(200));
    let mut last = String::from("status not read");
    let mut contracted = false;
    while !contracted && start.elapsed() < CONTRACT_TIMEOUT {
        thread::sleep(Duration::from_millis(100));
        match pd.get_status(&mut i2c) {
            Ok(s) => {
                contracted = s.status & STATUS_READY != 0 && s.requested_voltage_mv.abs_diff(voltage_mv) <= 500;
                last = describe(&s);
            }
            Err(e) => last = format!("status not read ({e:?})"),
        }
    }
    note.push_str(&format!(" | contract after {} ms: {last}", start.elapsed().as_millis()));
    if !contracted {
        bail!("no {voltage_mv} mV contract: {note}");
    }
    info!("PD contract {last}");
    pd.force_vout_on(&mut i2c).map_err(|e| anyhow!("VOUT on failed: {e:?}"))?;

    let mut rose = vout_rose(pd, &mut i2c, voltage_mv, "VOUT on", &mut note);
    if !rose {
        // Let the controller switch VOUT itself now that the contract is in place.
        match pd.set_vout_auto_control(&mut i2c) {
            Ok(()) => rose = vout_rose(pd, &mut i2c, voltage_mv, "auto control", &mut note),
            Err(e) => note.push_str(&format!(" | auto control failed ({e:?})")),
        }
    }
    *NOTE.lock().unwrap() = note.clone();
    if !rose {
        let _ = pd.force_vout_off(&mut i2c);
        bail!("VOUT did not rise to {voltage_mv} mV: {note}");
    }
    OUTPUT_MV.store(voltage_mv, Ordering::Relaxed);
    Ok(voltage_mv)
}
