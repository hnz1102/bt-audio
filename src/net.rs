// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Hiroshi Nakajima

//! WiFi station with WPS push-button setup, plus the services that need the
//! network: SNTP, remote syslog and the web UI. The WPS handling follows the
//! one in eew-display.
//!
//! Everything runs on one thread that owns the WiFi driver, so the audio path
//! never waits on the network. The radio is shared with Bluetooth: the WiFi
//! modem sleeps between beacons to leave airtime for the A2DP stream.

use std::ffi::{c_void, CString};
use std::net::Ipv4Addr;
use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use anyhow::{anyhow, bail, Result};
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::handle::RawHandle;
use esp_idf_svc::hal::modem::Modem;
use esp_idf_svc::http::server::EspHttpServer;
use esp_idf_svc::sntp::{EspSntp, SntpConf};
use esp_idf_svc::sys::*;
use esp_idf_svc::wifi::{AuthMethod, ClientConfiguration, Configuration, EspWifi};
use log::{error, info, warn};

use crate::config::Config;

const WPS_TIMEOUT_S: u32 = 120;
const RECONNECT_INTERVAL: Duration = Duration::from_secs(30);

// WPS handshake state, written by the event handler.
const WPS_RUNNING: u8 = 0;
const WPS_OK: u8 = 1;
const WPS_FAILED: u8 = 2;
static WPS_STATE: AtomicU8 = AtomicU8::new(WPS_RUNNING);
static WPS_CREDS: Mutex<Option<(String, String)>> = Mutex::new(None);
static WPS_REQUEST: AtomicBool = AtomicBool::new(false);

/// True while the station has an IP address.
static NET_UP: AtomicBool = AtomicBool::new(false);
/// With a static IP the address is usable as soon as the station associates.
static STATIC_IP: AtomicBool = AtomicBool::new(false);

pub fn net_up() -> bool {
    NET_UP.load(Ordering::Relaxed)
}

/// Asks the network thread to run a WPS push-button handshake.
pub fn request_wps() {
    WPS_REQUEST.store(true, Ordering::SeqCst);
}

pub fn rssi() -> Option<i32> {
    if !net_up() {
        return None;
    }
    let mut rssi = 0i32;
    (unsafe { esp_wifi_sta_get_rssi(&mut rssi) } == ESP_OK).then_some(rssi)
}

pub fn ip_address() -> Option<Ipv4Addr> {
    unsafe {
        let netif = esp_netif_get_handle_from_ifkey(c"WIFI_STA_DEF".as_ptr());
        if netif.is_null() {
            return None;
        }
        let mut info: esp_netif_ip_info_t = core::mem::zeroed();
        if esp_netif_get_ip_info(netif, &mut info) != ESP_OK || info.ip.addr == 0 {
            return None;
        }
        Some(Ipv4Addr::from(info.ip.addr.to_ne_bytes()))
    }
}

unsafe extern "C" fn net_event_handler(
    _arg: *mut c_void,
    base: esp_event_base_t,
    event_id: i32,
    _event_data: *mut c_void,
) {
    let id = event_id as u32;
    if base == IP_EVENT {
        if id == ip_event_t_IP_EVENT_STA_GOT_IP {
            NET_UP.store(true, Ordering::Relaxed);
        } else if id == ip_event_t_IP_EVENT_STA_LOST_IP {
            NET_UP.store(false, Ordering::Relaxed);
        }
    } else if base == WIFI_EVENT {
        if id == wifi_event_t_WIFI_EVENT_STA_CONNECTED {
            if STATIC_IP.load(Ordering::Relaxed) {
                NET_UP.store(true, Ordering::Relaxed);
            }
        } else if id == wifi_event_t_WIFI_EVENT_STA_DISCONNECTED || id == wifi_event_t_WIFI_EVENT_STA_STOP {
            NET_UP.store(false, Ordering::Relaxed);
        }
    }
}

fn register_net_events() {
    unsafe {
        esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, Some(net_event_handler), core::ptr::null_mut());
        esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, Some(net_event_handler), core::ptr::null_mut());
    }
}

fn cstr(bytes: &[u8]) -> String {
    let len = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
    String::from_utf8_lossy(&bytes[..len]).into_owned()
}

/// Two success paths exist: with credentials in the event data, or with the
/// driver having connected on its own (then they are in the STA config).
unsafe extern "C" fn wps_event_handler(
    _arg: *mut c_void,
    _base: esp_event_base_t,
    event_id: i32,
    event_data: *mut c_void,
) {
    let id = event_id as u32;
    if id == wifi_event_t_WIFI_EVENT_STA_WPS_ER_SUCCESS {
        if !event_data.is_null() {
            let evt = &*(event_data as *const wifi_event_sta_wps_er_success_t);
            if evt.ap_cred_cnt > 0 {
                let cred = &evt.ap_cred[0];
                *WPS_CREDS.lock().unwrap() = Some((cstr(&cred.ssid), cstr(&cred.passphrase)));
            }
        }
        WPS_STATE.store(WPS_OK, Ordering::Release);
    } else if id == wifi_event_t_WIFI_EVENT_STA_WPS_ER_FAILED || id == wifi_event_t_WIFI_EVENT_STA_WPS_ER_TIMEOUT {
        WPS_STATE.store(WPS_FAILED, Ordering::Release);
    }
}

/// Stops WPS and removes its event handler, however the handshake ends.
struct WpsGuard;

impl Drop for WpsGuard {
    fn drop(&mut self) {
        unsafe {
            esp_wifi_wps_disable();
            esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, Some(wps_event_handler));
        }
    }
}

/// Runs a WPS push-button handshake (press the button on the router) and
/// returns the SSID and passphrase it provides.
fn run_wps(wifi: &mut EspWifi<'static>) -> Result<(String, String)> {
    let _ = wifi.disconnect();
    wifi.set_configuration(&Configuration::Client(ClientConfiguration::default()))?;
    if !wifi.is_started()? {
        wifi.start()?;
    }

    WPS_STATE.store(WPS_RUNNING, Ordering::Release);
    *WPS_CREDS.lock().unwrap() = None;
    let guard = WpsGuard;
    unsafe {
        esp!(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, Some(wps_event_handler), core::ptr::null_mut()))?;
        let cfg = esp_wps_config_t { wps_type: wps_type_WPS_TYPE_PBC, ..Default::default() };
        esp!(esp_wifi_wps_enable(&cfg))?;
        esp!(esp_wifi_wps_start(0))?;
    }
    info!("WPS: press the WPS button on the router within {WPS_TIMEOUT_S} s");

    for _ in 0..WPS_TIMEOUT_S {
        thread::sleep(Duration::from_secs(1));
        match WPS_STATE.load(Ordering::Acquire) {
            WPS_FAILED => bail!("WPS failed or timed out"),
            WPS_OK => {
                let creds = WPS_CREDS.lock().unwrap().take();
                let (ssid, psk) = match creds {
                    Some(c) => c,
                    None => unsafe {
                        let mut conf: wifi_config_t = core::mem::zeroed();
                        esp!(esp_wifi_get_config(wifi_interface_t_WIFI_IF_STA, &mut conf))?;
                        (cstr(&conf.sta.ssid), cstr(&conf.sta.password))
                    },
                };
                drop(guard);
                return Ok((ssid, psk));
            }
            _ => {}
        }
    }
    bail!("WPS timed out after {WPS_TIMEOUT_S} s")
}

fn apply_static_ip(wifi: &EspWifi<'static>, cfg: &Config) -> Result<()> {
    let parse = |name: &str, v: &str| v.trim().parse::<Ipv4Addr>().map_err(|e| anyhow!("invalid {name} '{v}': {e}"));
    let ip = parse("IP address", &cfg.ip_address)?;
    let mask = parse("subnet mask", &cfg.subnet_mask)?;
    let gw = parse("gateway", &cfg.gateway)?;
    let handle = wifi.sta_netif().handle();
    unsafe {
        esp_netif_dhcpc_stop(handle);
        let info = esp_netif_ip_info_t {
            ip: esp_ip4_addr_t { addr: u32::from_ne_bytes(ip.octets()) },
            netmask: esp_ip4_addr_t { addr: u32::from_ne_bytes(mask.octets()) },
            gw: esp_ip4_addr_t { addr: u32::from_ne_bytes(gw.octets()) },
        };
        esp!(esp_netif_set_ip_info(handle, &info))?;
        if let Ok(dns) = cfg.dns.trim().parse::<Ipv4Addr>() {
            let mut dns_info: esp_netif_dns_info_t = core::mem::zeroed();
            dns_info.ip.u_addr.ip4.addr = u32::from_ne_bytes(dns.octets());
            esp_netif_set_dns_info(handle, esp_netif_dns_type_t_ESP_NETIF_DNS_MAIN, &mut dns_info);
        }
    }
    Ok(())
}

/// Applies the station settings and starts connecting.
fn configure_and_connect(wifi: &mut EspWifi<'static>, cfg: &Config) -> Result<()> {
    let too_long = |_| anyhow!("SSID or password too long");
    wifi.set_configuration(&Configuration::Client(ClientConfiguration {
        ssid: cfg.wifi_ssid.as_str().try_into().map_err(too_long)?,
        password: cfg.wifi_psk.as_str().try_into().map_err(too_long)?,
        auth_method: if cfg.wifi_psk.is_empty() { AuthMethod::None } else { AuthMethod::WPA2Personal },
        ..Default::default()
    }))?;
    if !wifi.is_started()? {
        wifi.start()?;
    }
    if let Ok(name) = CString::new(cfg.syslog_host_name.as_str()) {
        unsafe { esp_netif_set_hostname(wifi.sta_netif().handle(), name.as_ptr()) };
    }
    STATIC_IP.store(cfg.static_ip(), Ordering::Relaxed);
    if cfg.static_ip() {
        if let Err(e) = apply_static_ip(wifi, cfg) {
            warn!("static IP not applied, using DHCP: {e}");
            STATIC_IP.store(false, Ordering::Relaxed);
        }
    }
    // Modem sleep between beacons leaves airtime for the Bluetooth stream.
    esp!(unsafe { esp_wifi_set_ps(wifi_ps_type_t_WIFI_PS_MIN_MODEM) })?;
    wifi.connect()?;
    Ok(())
}

/// What runs once the network is up; dropping it stops the services.
struct Services {
    _sntp: Option<EspSntp<'static>>,
    _http: Option<EspHttpServer<'static>>,
}

fn start_services(cfg: &Arc<Mutex<Config>>) -> Services {
    let conf = cfg.lock().unwrap().clone();
    match ip_address() {
        Some(ip) => info!("WiFi up, IP address {ip}"),
        None => info!("WiFi up"),
    }

    let mut sntp_conf = SntpConf::default();
    let servers = [&conf.ntp_server1, &conf.ntp_server2, &conf.ntp_server3, &conf.ntp_server4];
    let mut count = 0;
    for server in servers.iter().map(|s| s.trim()).filter(|s| !s.is_empty()) {
        if count < sntp_conf.servers.len() {
            sntp_conf.servers[count] = server;
            count += 1;
        }
    }
    let sntp = match EspSntp::new(&sntp_conf) {
        Ok(s) => Some(s),
        Err(e) => {
            warn!("SNTP not started: {e}");
            None
        }
    };
    crate::syslog::apply(&conf);
    let http = match crate::web::start(cfg.clone()) {
        Ok(s) => {
            info!("web UI on port 80");
            Some(s)
        }
        Err(e) => {
            error!("web UI not started: {e}");
            None
        }
    };
    unsafe {
        info!(
            "heap free {} bytes, minimum {} bytes",
            esp_get_free_heap_size(),
            esp_get_minimum_free_heap_size()
        );
    }
    Services { _sntp: sntp, _http: http }
}

fn run(cfg: Arc<Mutex<Config>>, modem: Modem) -> Result<()> {
    let mut wifi = EspWifi::new(modem, EspSystemEventLoop::take()?, None)?;
    register_net_events();

    let mut conf = cfg.lock().unwrap().clone();
    if conf.wifi_ssid.is_empty() {
        if conf.wps_enable == "true" {
            request_wps();
        } else {
            warn!("no WiFi network configured and WPS is disabled");
        }
    }

    let mut services: Option<Services> = None;
    let mut next_connect = Instant::now();
    loop {
        if WPS_REQUEST.swap(false, Ordering::SeqCst) {
            match run_wps(&mut wifi) {
                Ok((ssid, psk)) => {
                    info!("WPS: received the credentials for \"{ssid}\"");
                    let mut c = cfg.lock().unwrap();
                    c.wifi_ssid = ssid;
                    c.wifi_psk = psk;
                    if !c.save_wifi() {
                        warn!("WPS: credentials not saved to NVS");
                    }
                    conf = c.clone();
                }
                Err(e) => warn!("WPS: {e}"),
            }
            next_connect = Instant::now();
        }

        if !conf.wifi_ssid.is_empty() && !net_up() && Instant::now() >= next_connect {
            let _ = wifi.disconnect();
            match configure_and_connect(&mut wifi, &conf) {
                Ok(()) => info!("connecting to \"{}\"", conf.wifi_ssid),
                Err(e) => warn!("WiFi connect failed: {e}"),
            }
            next_connect = Instant::now() + RECONNECT_INTERVAL;
        }

        if net_up() && services.is_none() {
            services = Some(start_services(&cfg));
        }
        thread::sleep(Duration::from_secs(1));
    }
}

/// Starts the network thread; `cfg` is shared with the web UI.
pub fn start(cfg: Arc<Mutex<Config>>, modem: Modem) -> std::io::Result<()> {
    thread::Builder::new().name("net".into()).stack_size(8192).spawn(move || {
        if let Err(e) = run(cfg, modem) {
            error!("network thread stopped: {e}");
        }
    })?;
    Ok(())
}
