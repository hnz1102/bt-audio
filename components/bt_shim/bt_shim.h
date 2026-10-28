#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

/* Thin C wrappers around IDF APIs that are only reachable through C macros
 * (BT_CONTROLLER_INIT_CONFIG_DEFAULT, I2S_*_DEFAULT_CONFIG) and so are not
 * visible in the generated Rust bindings. */

/* Init + enable the controller in Classic BT mode. */
esp_err_t bt_shim_controller_start(void);

/* Create the I2S TX channel (16-bit stereo, Philips) and enable it. */
esp_err_t bt_shim_i2s_start(uint32_t sample_rate, int bck, int ws, int dout);

/* Change the sample rate of the running channel. */
esp_err_t bt_shim_i2s_set_rate(uint32_t sample_rate);

/* Blocking write of interleaved 16-bit stereo PCM. Returns bytes written. */
size_t bt_shim_i2s_write(const uint8_t *data, size_t len);
