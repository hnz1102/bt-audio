#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

/* Thin C wrappers around IDF APIs that are only reachable through C macros
 * (BT_CONTROLLER_INIT_CONFIG_DEFAULT, I2S_*_DEFAULT_CONFIG) and so are not
 * visible in the generated Rust bindings. */

/* Init + enable the controller in Classic BT mode. */
esp_err_t bt_shim_controller_start(void);

/* Create the I2S TX channel (stereo, Philips, 32-bit slots, BCK = 64 x fs)
 * and enable it. MCLK (256 x fs) is output on `mclk`, which on the ESP32 must be GPIO0, 1 or 3.
 * The APLL is used as the clock source when it can produce the rate, so the
 * clocks are exact and low-jitter; otherwise the 160 MHz PLL is used. */
/* slot_bits: 16 (plain 16-bit stereo) or 32 (main + woofer words per slot). */
esp_err_t bt_shim_i2s_start(uint32_t sample_rate, int bck, int ws, int dout, int mclk, int slot_bits);

/* Change the sample rate of the running channel. */
esp_err_t bt_shim_i2s_set_rate(uint32_t sample_rate);

/* Number of I2S DMA underruns since the last call. */
uint32_t bt_shim_i2s_underruns(void);

/* Blocking write of interleaved 32-bit stereo slots. Returns bytes written. */
size_t bt_shim_i2s_write(const uint8_t *data, size_t len);

/* I2C master on port 0 at 100 kHz, shared by the amplifiers and the USB PD
 * controller. */
esp_err_t bt_shim_i2c_start(int sda, int scl);
/* Plain write / read of `len` bytes (a STOP ends each, no repeated start). */
esp_err_t bt_shim_i2c_write(uint8_t addr, const uint8_t *buf, size_t len);
esp_err_t bt_shim_i2c_read(uint8_t addr, uint8_t *buf, size_t len);
esp_err_t bt_shim_i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val);
esp_err_t bt_shim_i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *val);

/* One-shot ADC on ADC1 (12 bit, ~0-3.1 V). `gpio` must be an ADC1 pin. */
esp_err_t bt_shim_adc_add(int gpio);
/* Average of `samples` readings (0..4095), or a negative esp_err_t on error. */
int bt_shim_adc_read_avg(int gpio, int samples);
