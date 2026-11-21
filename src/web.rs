// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Hiroshi Nakajima

//! Web UI for the settings. The pages are static; the browser loads and saves
//! the settings over a small JSON / form API, so a request needs little heap.
//! Access is protected by a login with a session cookie. The server speaks
//! plain HTTP, so use it on a trusted network only.

use std::collections::HashMap;
use std::sync::atomic::Ordering;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use embedded_svc::http::Headers;
use esp_idf_svc::http::server::{Configuration, EspHttpServer};
use esp_idf_svc::http::Method;
use esp_idf_svc::io::{Read, Write};
use esp_idf_svc::sys::*;
use log::{info, warn};

use crate::config::{self, Config};

const MAX_BODY: usize = 2048;

const LOGIN_HTML: &str = r#"<!DOCTYPE html><html><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>BT Audio</title>
<style>body{font-family:sans-serif;background:#10151f;color:#e2e8f0;display:flex;justify-content:center;padding-top:15vh}
form{background:#1a2233;padding:28px;border-radius:10px;width:280px}h1{font-size:1.2em;margin:0 0 16px}
label{display:block;margin:10px 0 4px;font-size:.9em}input{width:100%;box-sizing:border-box;padding:8px;border-radius:6px;border:1px solid #33415c;background:#0f1626;color:inherit}
button{margin-top:16px;width:100%;padding:10px;border:0;border-radius:6px;background:#3b82f6;color:#fff;font-weight:bold}
#e{color:#f87171;font-size:.9em;display:none;margin-top:10px}</style></head><body>
<form method="POST" action="/login"><h1>BT Audio</h1>
<label>User</label><input name="username" autocomplete="username" required>
<label>Password</label><input type="password" name="password" autocomplete="current-password" required>
<button>Log in</button><div id="e">Wrong user or password</div></form>
<script>if(location.search.includes('error'))document.getElementById('e').style.display='block'</script>
</body></html>"#;

const PAGE_HTML: &str = r#"<!DOCTYPE html><html><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>BT Audio</title>
<style>body{font-family:sans-serif;background:#10151f;color:#e2e8f0;max-width:560px;margin:0 auto;padding:12px}
h1{font-size:1.3em}h2{font-size:1em;margin:22px 0 6px;color:#93c5fd;border-bottom:1px solid #33415c}
label{display:block;margin:8px 0 2px;font-size:.85em}input,select{width:100%;box-sizing:border-box;padding:8px;border-radius:6px;border:1px solid #33415c;background:#0f1626;color:inherit}
button{margin:10px 8px 0 0;padding:9px 14px;border:0;border-radius:6px;background:#3b82f6;color:#fff;font-weight:bold}
button.w{background:#b45309}button.d{background:#b91c1c}a{color:#93c5fd;float:right;font-size:.85em}
#st{display:grid;grid-template-columns:auto 1fr;gap:3px 12px;font-size:.85em;background:#1a2233;padding:10px;border-radius:8px}
#st dt{color:#94a3b8}#st dd{margin:0}.bad{color:#f87171}
input[type=range]{padding:0;border:0;background:none;accent-color:#3b82f6}output{float:right;color:#93c5fd}</style></head><body>
<h1>BT Audio <a href="/logout">Log out</a></h1>
<h2>Status</h2><dl id="st"><dt>Status</dt><dd>loading...</dd></dl>
<h2>Bluetooth devices</h2>
<div id="bt" style="font-size:.85em;background:#1a2233;padding:10px;border-radius:8px">loading...</div>
<div id="pair" style="font-size:.8em;color:#94a3b8;margin-top:6px"></div>
<button type="button" class="w" onclick="pair()">Pairing mode (3 min)</button>
<div style="font-size:.8em;color:#94a3b8;margin-top:6px">At boot the speaker connects to the most recent device, then to the next one if that is not reachable. New devices can pair only in pairing mode (or while no device is paired); a connected device is disconnected for it.</div>
<form id="f">
<h2>Audio</h2>
<label>Software gain <output id="gain_q8_v"></output></label><input type="range" name="gain_q8" min="0" max="1024" step="4" data-live>
<label>Crossover frequency <output id="crossover_hz_v"></output></label><input type="range" name="crossover_hz" min="50" max="300" step="5" data-live>
<label>Woofer subsonic high-pass <output id="subsonic_hz_v"></output></label><input type="range" name="subsonic_hz" min="0" max="100" step="1" data-live>
<label>Bass EQ boost <output id="bass_eq_db_v"></output></label><input type="range" name="bass_eq_db" min="0" max="12" step="1" data-live>
<label>Bass EQ frequency <output id="bass_eq_hz_v"></output></label><input type="range" name="bass_eq_hz" min="30" max="150" step="5" data-live>
<div style="font-size:.8em;color:#94a3b8;margin-top:6px">The sliders apply while moving; press Save to keep them after a reboot.</div>
<label>Volume knob range (dB, 6-100)</label><input type="number" name="vol_range_db" min="6" max="100">
<label>Bluetooth name</label><input name="bt_name">
<label>USB PD maximum voltage (mV, 5000-24000)</label><input type="number" name="pd_max_mv" min="5000" max="24000">
<button>Save</button>
<h2>WiFi</h2>
<label>SSID</label><input name="wifi_ssid">
<label>Password</label><input type="password" name="wifi_psk" placeholder="(leave empty to keep)" autocomplete="new-password">
<label>WPS at boot when no SSID is set</label><select name="wps_enable"><option value="true">Enabled</option><option value="false">Disabled</option></select>
<button type="button" class="w" onclick="act('/api/wps','Press the WPS button on the router within 120 s.')">Start WPS now</button>
<h2>Network</h2>
<label>Host name (DHCP and syslog)</label><input name="syslog_host_name">
<label>IP mode</label><select name="ip_mode"><option value="dhcp">DHCP</option><option value="static">Static</option></select>
<label>IP address</label><input name="ip_address"><label>Subnet mask</label><input name="subnet_mask">
<label>Gateway</label><input name="gateway"><label>DNS</label><input name="dns">
<label>NTP servers</label><input name="ntp_server1"><input name="ntp_server2"><input name="ntp_server3"><input name="ntp_server4">
<label>Time zone (POSIX TZ, e.g. JST-9, UTC0, CET-1CEST,M3.5.0,M10.5.0/3)</label><input name="timezone">
<h2>Syslog</h2>
<label>Forward logs</label><select name="syslog_enable"><option value="false">Off</option><option value="true">On</option></select>
<label>Server (host[:port], UDP, default port 514)</label><input name="syslog_server">
<label>Application name</label><input name="syslog_app_name">
<h2>Account</h2>
<label>User</label><input name="web_user">
<label>New password</label><input type="password" name="web_password" placeholder="(leave empty to keep)" autocomplete="new-password">
<button>Save</button>
<button type="button" class="w" onclick="act('/api/reboot','Rebooting...')">Reboot</button>
<button type="button" class="d" onclick="confirm('Erase all settings?')&&act('/api/factory_reset','Settings erased, rebooting...')">Factory reset</button>
</form>
<script>
const $=s=>document.querySelector(s);
async function post(u,b){const r=await fetch(u,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:b||''});if(r.status==401)location='/login';return r.text()}
async function act(u,m){alert(m);await post(u)}
async function load(){const r=await fetch('/api/config');if(r.status==401){location='/login';return}
const c=await r.json();for(const k in c){const e=document.querySelector('[name='+k+']');if(e)e.value=c[k]}show()}
const fmt={gain_q8:v=>v==0?'mute':(v/256).toFixed(2)+'x ('+(20*Math.log10(v/256)).toFixed(1)+' dB)',crossover_hz:v=>v+' Hz',subsonic_hz:v=>v<10?'off':v+' Hz',bass_eq_db:v=>v==0?'off':'+'+v+' dB',bass_eq_hz:v=>v+' Hz'};
function show(){for(const k in fmt)$('#'+k+'_v').textContent=fmt[k](+$('[name='+k+']').value)}
// Live apply: one request in flight, the latest value is sent when it returns.
let busy=false,dirty=false;
async function live(){if(busy){dirty=true;return}busy=true;
try{do{dirty=false;await post('/api/live',Object.keys(fmt).map(k=>k+'='+$('[name='+k+']').value).join('&'))}while(dirty)}catch(e){}busy=false}
document.querySelectorAll('[data-live]').forEach(e=>e.oninput=()=>{show();live()});
const h=t=>String(t).replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));
const na=v=>v===null?'--':h(v);
const db=v=>v===null?'--':v==='mute'?'mute':v==='absent'?'not connected':v+' dB';
const up=t=>{const d=Math.floor(t/86400),h=Math.floor(t%86400/3600),m=Math.floor(t%3600/60);return(d?d+'d ':'')+h+'h '+m+'m'};
async function stat(){try{const r=await fetch('/api/status');if(r.status==401){location='/login';return}if(!r.ok)return;const s=await r.json();
const rows=[['Time',na(s.time)],['Uptime',up(s.uptime_s)],
['WiFi',na(s.ip)+' / '+h(s.ssid)+' / '+na(s.rssi)+' dBm'],
['Bluetooth',s.bt_connected?(s.playing?'playing':'connected'):'not connected'],
['Sample rate',s.rate?s.rate+' Hz':'--'],
['Volume',db(s.vol_main)+' main / '+db(s.vol_woofer)+' woofer'],
['Amplifier',s.amp_fault?'<span class="bad">FAULT</span>':'OK'],
['USB PD',s.pd_mv===null?'--':(s.pd_mv/1000).toFixed(2)+' V, '+(s.pd_ma/1000).toFixed(3)+' A (set '+(s.pd_set_mv/1000).toFixed(1)+' V)'+(s.pd_fault?' <span class="bad">'+h(s.pd_fault)+'</span>':'')],
['Heap',s.heap_free+' bytes free (min '+s.heap_min+')'],['Firmware','v'+s.version]];
$('#st').innerHTML=rows.map(([k,v])=>'<dt>'+k+'</dt><dd>'+v+'</dd>').join('')}catch(e){}}
async function bt(){try{const r=await fetch('/api/bt');if(!r.ok)return;const b=await r.json();
$('#bt').innerHTML=b.devices.length?b.devices.map((d,i)=>'<div style="display:flex;align-items:center;gap:8px;margin:4px 0"><span style="flex:1">'+(d.name?h(d.name):'(unknown)')+' <span style="color:#94a3b8">'+d.addr+'</span>'+(d.connected?' <b style="color:#4ade80">connected</b>':'')+(i==0?' <span style="color:#93c5fd">last</span>':'')+'</span><button type="button" class="d" style="margin:0;padding:5px 10px" onclick="unpair(\''+d.addr+'\')">Remove</button></div>').join(''):'No paired devices';
$('#pair').textContent=b.pairing_s===null?(b.devices.length?'Not discoverable (paired devices can connect).':'Discoverable (no paired device yet).'):'Discoverable for pairing: '+b.pairing_s+' s left'}catch(e){}}
async function pair(){await post('/api/bt/pair');bt()}
async function unpair(a){if(confirm('Remove '+a+'?')){await post('/api/bt/remove','addr='+a);bt()}}
$('#f').onsubmit=async e=>{e.preventDefault();alert(await post('/api/config',new URLSearchParams(new FormData(e.target)).toString()))};
load();stat();bt();setInterval(()=>{stat();bt()},5000);
</script></body></html>"#;

/// Entity tag of the main page: FNV-1a hash of its text.
fn page_etag() -> String {
    let hash = PAGE_HTML.bytes().fold(0x811c_9dc5u32, |h, b| (h ^ b as u32).wrapping_mul(0x0100_0193));
    format!("\"{hash:08x}\"")
}

// ─── Session ──────────────────────────────────────────────────────────────────

type Session = Arc<Mutex<Option<String>>>;

fn new_token() -> String {
    (0..4).map(|_| format!("{:08x}", unsafe { esp_random() })).collect()
}

fn is_auth(cookie: Option<&str>, session: &Session) -> bool {
    let guard = session.lock().unwrap();
    let Some(token) = guard.as_ref() else { return false };
    cookie
        .and_then(|h| h.split(';').find_map(|p| p.trim().strip_prefix("session=")))
        .map_or(false, |v| v == token)
}

// ─── Request helpers ─────────────────────────────────────────────────────────

fn url_decode(s: &str) -> String {
    let b = s.as_bytes();
    let mut out = Vec::with_capacity(b.len());
    let mut i = 0;
    while i < b.len() {
        match b[i] {
            b'+' => out.push(b' '),
            b'%' if i + 2 < b.len() => {
                match u8::from_str_radix(core::str::from_utf8(&b[i + 1..i + 3]).unwrap_or(""), 16) {
                    Ok(v) => {
                        out.push(v);
                        i += 2;
                    }
                    Err(_) => out.push(b'%'),
                }
            }
            c => out.push(c),
        }
        i += 1;
    }
    String::from_utf8_lossy(&out).into_owned()
}

fn parse_form(body: &str) -> HashMap<String, String> {
    body.split('&')
        .filter_map(|pair| pair.split_once('='))
        .map(|(k, v)| (url_decode(k), url_decode(v)))
        .collect()
}

fn json_escape(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 2);
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '<' => out.push_str("\\u003c"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out
}

fn config_json(cfg: &Config) -> String {
    // The secrets are never sent back to the browser.
    let fields: Vec<String> = [
        ("wifi_ssid", &cfg.wifi_ssid), ("wps_enable", &cfg.wps_enable), ("syslog_host_name", &cfg.syslog_host_name),
        ("ip_mode", &cfg.ip_mode), ("ip_address", &cfg.ip_address), ("subnet_mask", &cfg.subnet_mask),
        ("gateway", &cfg.gateway), ("dns", &cfg.dns), ("ntp_server1", &cfg.ntp_server1),
        ("ntp_server2", &cfg.ntp_server2), ("ntp_server3", &cfg.ntp_server3), ("ntp_server4", &cfg.ntp_server4), ("timezone", &cfg.timezone),
        ("syslog_enable", &cfg.syslog_enable), ("syslog_server", &cfg.syslog_server),
        ("syslog_app_name", &cfg.syslog_app_name),
        ("web_user", &cfg.web_user), ("bt_name", &cfg.bt_name), ("gain_q8", &cfg.gain_q8),
        ("vol_range_db", &cfg.vol_range_db), ("pd_max_mv", &cfg.pd_max_mv),
        ("subsonic_hz", &cfg.subsonic_hz), ("crossover_hz", &cfg.crossover_hz),
        ("bass_eq_hz", &cfg.bass_eq_hz), ("bass_eq_db", &cfg.bass_eq_db),
    ]
    .iter()
    .map(|(k, v)| format!("\"{k}\":\"{}\"", json_escape(v)))
    .collect();
    format!("{{{}}}", fields.join(","))
}

fn status_json(cfg: &Config) -> String {
    let opt = |v: Option<String>| v.map_or("null".to_string(), |v| format!("\"{}\"", json_escape(&v)));
    let num = |v: Option<f32>| v.map_or("null".to_string(), |v| format!("{v:.1}"));
    let ip = crate::net::ip_address().map(|ip| ip.to_string());
    let rssi = crate::net::rssi().map(|r| r as f32);
    let uptime = unsafe { esp_timer_get_time() } / 1_000_000;
    let (free, min) = unsafe { (esp_get_free_heap_size(), esp_get_minimum_free_heap_size()) };
    let pd = crate::usbpd::measure();
    let volume = |i: usize| match crate::VOLUME[i].load(Ordering::Relaxed) {
        _ if !crate::AMP_PRESENT[i].load(Ordering::Relaxed) => "\"absent\"".to_string(),
        crate::VOLUME_MUTED => "\"mute\"".to_string(),
        reg => num(crate::volume_db(reg)),
    };
    let rate = crate::SAMPLE_RATE.load(Ordering::Relaxed);
    format!(
        concat!(
            "{{\"time\":{},\"uptime_s\":{},\"heap_free\":{},\"heap_min\":{},\"version\":\"{}\",",
            "\"ip\":{},\"ssid\":\"{}\",\"rssi\":{},",
            "\"bt_connected\":{},\"playing\":{},\"rate\":{},\"vol_main\":{},\"vol_woofer\":{},\"amp_fault\":{},",
            "\"pd_set_mv\":{},\"pd_mv\":{},\"pd_ma\":{},\"pd_fault\":{}}}"
        ),
        opt(crate::clock::display()), uptime, free, min, env!("CARGO_PKG_VERSION"),
        opt(ip), json_escape(&cfg.wifi_ssid), num(rssi),
        crate::BT_CONNECTED.load(Ordering::Relaxed), crate::PLAYING.load(Ordering::Relaxed),
        if rate == 0 { "null".to_string() } else { rate.to_string() },
        volume(0), volume(1), crate::amp_fault(),
        crate::usbpd::output_mv(),
        pd.as_ref().map_or("null".to_string(), |r| r.voltage_mv.to_string()),
        pd.as_ref().map_or("null".to_string(), |r| r.current_ma.to_string()),
        opt(pd.and_then(|r| r.fault)),
    )
}

/// Bonded devices (most recent first) and the pairing window.
fn bt_json() -> String {
    let connected = crate::bt::connected();
    let devices: Vec<String> = crate::bt::devices()
        .iter()
        .map(|b| {
            format!(
                "{{\"addr\":\"{}\",\"name\":\"{}\",\"connected\":{}}}",
                crate::bt::format(b),
                json_escape(&crate::bt::name(b).unwrap_or_default()),
                connected == Some(*b)
            )
        })
        .collect();
    let pairing = crate::bt::pairing_remaining().map_or("null".to_string(), |s| s.to_string());
    format!("{{\"pairing_s\":{pairing},\"devices\":[{}]}}", devices.join(","))
}

/// Keeps a numeric setting inside its range so a typo cannot store nonsense.
fn clamped(value: &str, min: i64, max: i64, fallback: i64) -> String {
    value.trim().parse::<i64>().unwrap_or(fallback).clamp(min, max).to_string()
}

fn restart_soon() {
    thread::spawn(|| {
        thread::sleep(Duration::from_millis(700));
        unsafe { esp_restart() }
    });
}

// ─── Server ───────────────────────────────────────────────────────────────────

pub fn start(cfg: Arc<Mutex<Config>>) -> anyhow::Result<EspHttpServer<'static>> {
    let session: Session = Arc::new(Mutex::new(None));
    let conf = Configuration { max_uri_handlers: 16, max_open_sockets: 2, ..Default::default() };
    let mut server = EspHttpServer::new(&conf)?;

    // Browsers ask for it on every page; answer without a body instead of a 404.
    server.fn_handler("/favicon.ico", Method::Get, |request| {
        request.into_status_response(204)?;
        Ok::<(), anyhow::Error>(())
    })?;

    server.fn_handler("/login", Method::Get, |request| {
        request.into_ok_response()?.write_all(LOGIN_HTML.as_bytes())?;
        Ok::<(), anyhow::Error>(())
    })?;

    let (cfg_c, session_c) = (cfg.clone(), session.clone());
    server.fn_handler("/login", Method::Post, move |mut request| {
        let len = (request.content_len().unwrap_or(0) as usize).min(512);
        let mut buf = vec![0u8; len];
        request.read_exact(&mut buf).unwrap_or(());
        let form = parse_form(&String::from_utf8_lossy(&buf));
        let valid = {
            let c = cfg_c.lock().unwrap();
            form.get("username") == Some(&c.web_user) && form.get("password") == Some(&c.web_password)
        };
        if valid {
            let token = new_token();
            let cookie = format!("session={token}; Path=/; HttpOnly; SameSite=Strict");
            *session_c.lock().unwrap() = Some(token);
            request.into_response(302, Some("Found"), &[("Set-Cookie", cookie.as_str()), ("Location", "/")])?;
        } else {
            warn!("web: failed login");
            request.into_response(302, Some("Found"), &[("Location", "/login?error=1")])?;
        }
        Ok::<(), anyhow::Error>(())
    })?;

    let session_c = session.clone();
    server.fn_handler("/logout", Method::Get, move |request| {
        *session_c.lock().unwrap() = None;
        request.into_response(302, Some("Found"), &[("Set-Cookie", "session=; Path=/; Max-Age=0"), ("Location", "/login")])?;
        Ok::<(), anyhow::Error>(())
    })?;

    let session_c = session.clone();
    server.fn_handler("/", Method::Get, move |request| {
        if !is_auth(request.header("Cookie"), &session_c) {
            request.into_response(302, Some("Found"), &[("Location", "/login")])?;
            return Ok::<(), anyhow::Error>(());
        }
        // A reload only revalidates the page: sending it again over WiFi
        // made the A2DP stream drop packets.
        let etag = page_etag();
        if request.header("If-None-Match") == Some(etag.as_str()) {
            request.into_response(304, Some("Not Modified"), &[("ETag", etag.as_str())])?;
            return Ok(());
        }
        let headers = [("Content-Type", "text/html; charset=UTF-8"), ("ETag", etag.as_str()), ("Cache-Control", "no-cache")];
        let mut response = request.into_response(200, Some("OK"), &headers)?;
        response.write_all(PAGE_HTML.as_bytes())?;
        Ok(())
    })?;

    let (cfg_c, session_c) = (cfg.clone(), session.clone());
    server.fn_handler("/api/config", Method::Get, move |request| {
        if !is_auth(request.header("Cookie"), &session_c) {
            request.into_status_response(401)?.write_all(b"Unauthorized")?;
            return Ok::<(), anyhow::Error>(());
        }
        let json = config_json(&cfg_c.lock().unwrap());
        request
            .into_response(200, Some("OK"), &[("Content-Type", "application/json")])?
            .write_all(json.as_bytes())?;
        Ok(())
    })?;

    let (cfg_c, session_c) = (cfg.clone(), session.clone());
    server.fn_handler("/api/config", Method::Post, move |mut request| {
        if !is_auth(request.header("Cookie"), &session_c) {
            request.into_status_response(401)?.write_all(b"Unauthorized")?;
            return Ok::<(), anyhow::Error>(());
        }
        let len = (request.content_len().unwrap_or(0) as usize).min(MAX_BODY);
        let mut buf = vec![0u8; len];
        request.read_exact(&mut buf).unwrap_or(());
        let form = parse_form(&String::from_utf8_lossy(&buf));

        let (ok, snapshot) = {
            let mut c = cfg_c.lock().unwrap();
            macro_rules! text {
                ($($field:ident),*) => {$(
                    if let Some(v) = form.get(stringify!($field)) { c.$field = v.trim().to_string(); }
                )*};
            }
            text!(wifi_ssid, wps_enable, syslog_host_name, ip_mode, ip_address, subnet_mask, gateway, dns,
                  ntp_server1, ntp_server2, ntp_server3, ntp_server4, timezone, syslog_enable, syslog_server,
                  syslog_app_name, web_user, bt_name);
            // The secrets are only replaced when something was typed.
            if let Some(v) = form.get("wifi_psk").filter(|v| !v.is_empty()) {
                c.wifi_psk = v.clone();
            }
            if let Some(v) = form.get("web_password").filter(|v| !v.is_empty()) {
                c.web_password = v.clone();
            }
            if let Some(v) = form.get("gain_q8") { c.gain_q8 = clamped(v, 0, 1024, 256); }
            if let Some(v) = form.get("vol_range_db") { c.vol_range_db = clamped(v, 6, 100, 60); }
            if let Some(v) = form.get("pd_max_mv") { c.pd_max_mv = clamped(v, 5000, 24000, 24000); }
            if let Some(v) = form.get("subsonic_hz") { c.subsonic_hz = clamped(v, 0, 100, 30); }
            if let Some(v) = form.get("crossover_hz") { c.crossover_hz = clamped(v, 50, 300, 150); }
            if let Some(v) = form.get("bass_eq_hz") { c.bass_eq_hz = clamped(v, 30, 150, 60); }
            if let Some(v) = form.get("bass_eq_db") { c.bass_eq_db = clamped(v, 0, 12, 6); }
            (c.save(), c.clone())
        };

        // These apply at once; the rest needs a reboot.
        crate::clock::set_timezone(&snapshot.timezone);
        crate::set_gain_q8(snapshot.gain_q8());
        crate::set_vol_range_db(snapshot.vol_range_db());
        crate::set_subsonic_hz(snapshot.subsonic_hz());
        crate::set_crossover_hz(snapshot.crossover_hz());
        crate::set_bass_eq(snapshot.bass_eq_hz(), snapshot.bass_eq_db());
        crate::syslog::apply(&snapshot);

        if ok {
            info!("web: settings saved");
            request
                .into_response(200, Some("OK"), &[("Content-Type", "text/plain; charset=UTF-8")])?
                .write_all(b"Saved. Gain, volume range, crossover, subsonic filter, bass EQ, time zone and syslog are applied now; WiFi, network, name, USB PD and account need a reboot.")?;
        } else {
            warn!("web: settings not saved to NVS");
            request.into_status_response(500)?.write_all(b"NVS write error")?;
        }
        Ok(())
    })?;

    let (cfg_c, session_c) = (cfg.clone(), session.clone());
    server.fn_handler("/api/status", Method::Get, move |request| {
        if !is_auth(request.header("Cookie"), &session_c) {
            request.into_status_response(401)?.write_all(b"Unauthorized")?;
            return Ok::<(), anyhow::Error>(());
        }
        let json = status_json(&cfg_c.lock().unwrap());
        request
            .into_response(200, Some("OK"), &[("Content-Type", "application/json")])?
            .write_all(json.as_bytes())?;
        Ok(())
    })?;

    // Slider values applied at once without writing NVS (Save stores them).
    let session_c = session.clone();
    server.fn_handler("/api/live", Method::Post, move |mut request| {
        if !is_auth(request.header("Cookie"), &session_c) {
            request.into_status_response(401)?.write_all(b"Unauthorized")?;
            return Ok::<(), anyhow::Error>(());
        }
        let len = (request.content_len().unwrap_or(0) as usize).min(256);
        let mut buf = vec![0u8; len];
        request.read_exact(&mut buf).unwrap_or(());
        let form = parse_form(&String::from_utf8_lossy(&buf));
        let num = |k: &str, max: i64| form.get(k).and_then(|v| v.trim().parse::<i64>().ok()).map(|v| v.clamp(0, max));
        if let Some(v) = num("gain_q8", 1024) {
            crate::set_gain_q8(v as i32);
        }
        if let Some(v) = num("crossover_hz", 300) {
            crate::set_crossover_hz(v.max(50) as u32);
        }
        if let (Some(hz), Some(db)) = (num("bass_eq_hz", 150), num("bass_eq_db", 12)) {
            crate::set_bass_eq(hz.max(30) as u32, db as u32);
        }
        if let Some(v) = num("subsonic_hz", 100) {
            crate::set_subsonic_hz(if v < 10 { 0 } else { v as u32 });
        }
        request.into_ok_response()?.write_all(b"OK")?;
        Ok(())
    })?;

    let session_c = session.clone();
    server.fn_handler("/api/bt", Method::Get, move |request| {
        if !is_auth(request.header("Cookie"), &session_c) {
            request.into_status_response(401)?.write_all(b"Unauthorized")?;
            return Ok::<(), anyhow::Error>(());
        }
        let json = bt_json();
        request
            .into_response(200, Some("OK"), &[("Content-Type", "application/json")])?
            .write_all(json.as_bytes())?;
        Ok(())
    })?;

    let session_c = session.clone();
    server.fn_handler("/api/bt/pair", Method::Post, move |request| {
        if !is_auth(request.header("Cookie"), &session_c) {
            request.into_status_response(401)?.write_all(b"Unauthorized")?;
            return Ok::<(), anyhow::Error>(());
        }
        crate::bt::start_pairing();
        request.into_ok_response()?.write_all(b"Pairing mode")?;
        Ok(())
    })?;

    let session_c = session.clone();
    server.fn_handler("/api/bt/remove", Method::Post, move |mut request| {
        if !is_auth(request.header("Cookie"), &session_c) {
            request.into_status_response(401)?.write_all(b"Unauthorized")?;
            return Ok::<(), anyhow::Error>(());
        }
        let len = (request.content_len().unwrap_or(0) as usize).min(128);
        let mut buf = vec![0u8; len];
        request.read_exact(&mut buf).unwrap_or(());
        let form = parse_form(&String::from_utf8_lossy(&buf));
        match form.get("addr").and_then(|a| crate::bt::parse(a)) {
            Some(bda) if crate::bt::remove(bda) => request.into_ok_response()?.write_all(b"Removed")?,
            Some(_) => request.into_status_response(500)?.write_all(b"Remove failed")?,
            None => request.into_status_response(400)?.write_all(b"Bad address")?,
        }
        Ok(())
    })?;

    let session_c = session.clone();
    server.fn_handler("/api/wps", Method::Post, move |request| {
        if !is_auth(request.header("Cookie"), &session_c) {
            request.into_status_response(401)?.write_all(b"Unauthorized")?;
            return Ok::<(), anyhow::Error>(());
        }
        crate::net::request_wps();
        request.into_ok_response()?.write_all(b"WPS started")?;
        Ok(())
    })?;

    let session_c = session.clone();
    server.fn_handler("/api/reboot", Method::Post, move |request| {
        if !is_auth(request.header("Cookie"), &session_c) {
            request.into_status_response(401)?.write_all(b"Unauthorized")?;
            return Ok::<(), anyhow::Error>(());
        }
        request.into_ok_response()?.write_all(b"Rebooting")?;
        restart_soon();
        Ok(())
    })?;

    let session_c = session.clone();
    server.fn_handler("/api/factory_reset", Method::Post, move |request| {
        if !is_auth(request.header("Cookie"), &session_c) {
            request.into_status_response(401)?.write_all(b"Unauthorized")?;
            return Ok::<(), anyhow::Error>(());
        }
        let ok = config::erase();
        request.into_ok_response()?.write_all(if ok { b"Erased" } else { b"Erase failed" })?;
        if ok {
            restart_soon();
        }
        Ok(())
    })?;

    Ok(server)
}
