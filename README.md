# bt-audio

A Bluetooth A2DP sink for the original ESP32, written in Rust. It receives
audio from a phone or PC over Bluetooth Classic and plays it through a
MAX98357A I2S DAC and amplifier.

## How it works

```
Bluetooth (A2DP/SBC) -> Bluedroid decoder -> bounded channel -> I2S writer thread -> DAC
```

- The ESP32 advertises itself as `ESP32-Speaker` and accepts pairing
  automatically (SSP numeric comparison, with a legacy PIN of `1234` as
  fallback).
- The Bluedroid stack decodes SBC and hands 16-bit stereo PCM to a callback
  running on the Bluetooth task. The callback copies each packet into a
  bounded queue (128 packets) and drops it if the queue is full, so a slow
  I2S write never stalls the Bluetooth stack.
- A separate `i2s-writer` thread applies a software gain, then writes to the
  I2S channel.
- The I2S sample rate follows the rate negotiated by the source (16, 32,
  44.1 or 48 kHz).
- The I2S clocks are BCK and WS only. MCLK is not driven, because the
  MAX98357A derives its clock from BCLK and LRCLK.

The Rust side lives in [src/main.rs](src/main.rs). A small C component,
[components/bt_shim](components/bt_shim), wraps the Bluetooth controller
start-up and the I2S driver, whose configuration macros are awkward to use
from Rust.

## Hardware

- An original ESP32 (classic Bluetooth is not available on the S2, S3, C3 and
  later chips).
- A MAX98357A I2S DAC / class-D amplifier board and a speaker.
- 16 MB flash (the partition table and `sdkconfig.defaults` assume it; adjust
  `CONFIG_ESPTOOLPY_FLASHSIZE_*` for other sizes).

### Wiring

| Signal | ESP32 GPIO | MAX98357A pin |
|--------|-----------|---------------|
| BCK    | 26        | BCLK          |
| WS / LRCK | 25     | LRC           |
| DATA (DOUT) | 22   | DIN           |
| SD (mute) | 13 | SD (630 kΩ pull-up to 3.3 V) |

The pins are constants at the top of [src/main.rs](src/main.rs). Power the
MAX98357A (`VIN`) and connect `GND`. The remaining pins set the behavior in
hardware:

- `SD`: shutdown and channel select, connected to GPIO13 and pulled up to 3.3 V
  through 630 kΩ. With the MAX98357A's internal pull-down of about 100 kΩ
  this sets the pin to roughly 0.45 V, which selects the (L+R)/2 mix of the
  stereo stream (the datasheet gives about 0.16-0.77 V for the mix). The
  firmware uses the pin as a mute: it drives GPIO13 low (shutdown) at boot, on
  disconnect and whenever the stream is not running, and releases it to
  high impedance (internal pulls off, so the divider stays at 0.45 V) when
  `audio state 1` (stream started) is reported. This avoids hiss and pops
  while idle.
- `GAIN`: sets a fixed amplifier gain of 3 to 15 dB. This project leaves it
  open, which selects 9 dB. It adds to the software gain (`GAIN_Q8`).

> **Keep GPIO6-11 clear.** These pins connect the module to its internal SPI
> flash. If they are wired to anything external, noise can corrupt
> instruction fetches and cause random `IllegalInstruction` panics, typically
> when Bluetooth audio starts.

## Building and flashing

### 1. Set up the environment

On Ubuntu / Debian (including WSL2), run the setup script once:

```sh
bash setup_env.sh
```

It installs the system packages, Rust (rustup), `ldproxy`, `espup` and
`cargo-espflash`, the Xtensa toolchain for the ESP32 (`espup install
--targets esp32`), and adds your user to the `dialout` group for serial port
access (log out and back in afterwards).

In every new terminal, load the Rust and ESP environments before building:

```sh
source "$HOME/.cargo/env"
source "$HOME/export-esp.sh"
```

ESP-IDF v5.2.2 is downloaded and built automatically by `esp-idf-sys` on the
first build, which takes a while.

### 2. Build and flash

```sh
cargo build --release
cargo espflash flash --release --monitor --port /dev/ttyUSB0
```

The target, ESP-IDF version and `build-std` are configured in
[.cargo/config.toml](.cargo/config.toml). The partition table is
[partitions.csv](partitions.csv): a single 3 MB factory app, because the
default 1 MB partition is too small with Bluedroid.

Press `Ctrl+R` in the monitor to reset the chip. This works when the
USB-serial chip's DTR/RTS lines are wired to EN and GPIO0, as on most
development boards.

## Configuration

Constants in [src/main.rs](src/main.rs):

| Constant | Default | Meaning |
|----------|---------|---------|
| `DEVICE_NAME` | `ESP32-Speaker` | Bluetooth name shown to sources |
| `DEFAULT_SAMPLE_RATE` | `44_100` | I2S rate before the source negotiates one |
| `GAIN_Q8` | `256` | Software gain in Q8 fixed point (256 = 1.0x). Samples are clamped, so values above 256 clip loud material |
| `QUEUE_DEPTH` | `128` | Packets buffered between the Bluetooth task and the I2S writer |
| `PIN_BCK`, `PIN_WS`, `PIN_DOUT` | 26, 25, 22 | I2S pins |
| `PIN_SD` | 13 | MAX98357A SD pin, used as mute |

Build-time settings are in [sdkconfig.defaults](sdkconfig.defaults): classic
Bluetooth only (BR/EDR), Bluedroid with A2DP and SSP, a custom partition
table, and 16 MB flash.

## Troubleshooting

- **Random `IllegalInstruction` panics or garbled backtraces**: check the
  flash pins (GPIO6-11) and keep wiring away from the antenna. Lowering the
  flash clock with `CONFIG_ESPTOOLPY_FLASHFREQ_20M=y` (and flashing with
  `-f 20mhz`) can help as a diagnostic, but removing the noise source is the
  real fix.
- **Distorted sound**: set `GAIN_Q8` to `256` or lower; higher values clip.
  Also check the source volume, the MAX98357A `GAIN` pin (open = 9 dB; tie
  it to VDD for 6 dB, or through 100 kΩ to VDD for 3 dB, if it distorts) and its power supply and ground.
- **No sound but connected**: check the log for `audio state 1` and verify the
  I2S wiring and the MAX98357A `SD` pin (about 0.45 V gives the mix; 0 V is shutdown, which is where the firmware holds it until a stream starts).

## Limitations

- SBC only; no AAC or other codecs.
- No AVRCP (volume and playback control from the sink side).
- One connection at a time; the device stops being discoverable while
  connected.

## License

MIT License. Copyright (c) 2026 Hiroshi Nakajima.

The license is declared in the header of [src/main.rs](src/main.rs)
(`SPDX-License-Identifier: MIT`) and in [Cargo.toml](Cargo.toml)
(`license = "MIT"`). 