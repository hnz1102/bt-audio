// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Hiroshi Nakajima

//! Logger that writes to the ESP console and, once enabled, also forwards every
//! record to a remote syslog server over UDP (RFC 5424), stamped with the
//! local time and its UTC offset ("-" until SNTP has set the clock).
//!
//! `log` is called from tasks with small stacks (the Bluetooth callbacks), so
//! it only formats the message and queues it; a separate thread sends it.

use std::net::{ToSocketAddrs, UdpSocket};
use std::sync::mpsc::{sync_channel, SyncSender};
use std::sync::{Mutex, OnceLock};
use std::thread;

use esp_idf_svc::log::EspLogger;
use log::{Level, Log, Metadata, Record};

use crate::config::Config;

const QUEUE_DEPTH: usize = 32;
const MAX_MESSAGE: usize = 480;
// Facility "user" (1).
const FACILITY: u8 = 1 << 3;

static CONSOLE: EspLogger = EspLogger::new();
static LOGGER: Logger = Logger;

/// Host name and application name sent in each message; None while
/// forwarding is off.
static IDENT: Mutex<Option<(String, String)>> = Mutex::new(None);
static SOCKET: Mutex<Option<(UdpSocket, std::net::SocketAddr)>> = Mutex::new(None);
static QUEUE: OnceLock<SyncSender<String>> = OnceLock::new();

struct Logger;

impl Log for Logger {
    fn enabled(&self, metadata: &Metadata) -> bool {
        metadata.level() <= log::max_level()
    }

    fn log(&self, record: &Record) {
        if !self.enabled(record.metadata()) {
            return;
        }
        CONSOLE.log(record);
        let Some((host, app)) = IDENT.lock().unwrap().clone() else { return };
        let Some(queue) = QUEUE.get() else { return };
        let severity = match record.level() {
            Level::Error => 3,
            Level::Warn => 4,
            Level::Info => 6,
            Level::Debug | Level::Trace => 7,
        };
        let mut msg = format!(
            "<{}>1 {} {} {} - - - [{}] {}",
            FACILITY | severity,
            crate::clock::rfc3339().as_deref().unwrap_or("-"),
            host,
            app,
            record.target(),
            record.args()
        );
        if msg.len() > MAX_MESSAGE {
            let mut end = MAX_MESSAGE;
            while !msg.is_char_boundary(end) {
                end -= 1;
            }
            msg.truncate(end);
        }
        // Drop the message when the sender cannot keep up.
        let _ = queue.try_send(msg);
    }

    fn flush(&self) {}
}

/// Installs the logger. Call once, before any logging.
pub fn init() {
    if log::set_logger(&LOGGER).is_ok() {
        CONSOLE.initialize();
    }
}

/// Starts or stops forwarding according to the settings. `syslog_server` is
/// `host[:port]` (port 514 by default). Takes effect immediately.
pub fn apply(cfg: &Config) {
    if cfg.syslog_enable != "true" || cfg.syslog_server.trim().is_empty() {
        *IDENT.lock().unwrap() = None;
        *SOCKET.lock().unwrap() = None;
        return;
    }
    let server = cfg.syslog_server.trim();
    let target = if server.contains(':') { server.to_string() } else { format!("{server}:514") };
    let addr = match target.to_socket_addrs().ok().and_then(|mut a| a.next()) {
        Some(a) => a,
        None => {
            log::warn!("syslog: cannot resolve {target}");
            return;
        }
    };
    let socket = match UdpSocket::bind("0.0.0.0:0") {
        Ok(s) => s,
        Err(e) => {
            log::warn!("syslog: socket failed: {e}");
            return;
        }
    };
    let _ = socket.set_nonblocking(true);
    let previous = SOCKET.lock().unwrap().replace((socket, addr)).map(|(_, a)| a);
    QUEUE.get_or_init(|| {
        let (tx, rx) = sync_channel::<String>(QUEUE_DEPTH);
        thread::Builder::new()
            .name("syslog".into())
            .stack_size(4096)
            .spawn(move || {
                for msg in rx {
                    if let Some((socket, addr)) = SOCKET.lock().unwrap().as_ref() {
                        let _ = socket.send_to(msg.as_bytes(), addr);
                    }
                }
            })
            .expect("spawn syslog");
        tx
    });
    *IDENT.lock().unwrap() = Some((cfg.syslog_host_name.clone(), cfg.syslog_app_name.clone()));
    log::info!("syslog: forwarding to {addr}");
    // The messages from before forwarding began never reached the server, so
    // report the supply output now that it can.
    if previous != Some(addr) {
        crate::usbpd::log_status();
    }
}
