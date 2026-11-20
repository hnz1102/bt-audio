#include "bt_shim.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_bt.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/i2s_std.h"

static const char *TAG = "bt_shim";

#define AMP_I2C_PORT        I2C_NUM_0   /* shared bus: amplifiers + USB PD */
#define AMP_I2C_TIMEOUT_MS  100

static i2s_chan_handle_t s_tx;
static i2s_std_config_t s_cfg;
static bool s_apll;
/* DMA buffers sent before new data arrived (underruns; auto_clear sends 0s). */
static volatile uint32_t s_underruns;

static IRAM_ATTR bool on_send_q_ovf(i2s_chan_handle_t handle, i2s_event_data_t *event, void *ctx)
{
    s_underruns++;
    return false;
}

static adc_oneshot_unit_handle_t s_adc;

esp_err_t bt_shim_controller_start(void)
{
    esp_err_t err = esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
    if (err != ESP_OK) {
        return err;
    }
    esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    err = esp_bt_controller_init(&cfg);
    if (err != ESP_OK) {
        return err;
    }
    return esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
}

static i2s_std_clk_config_t make_clk(uint32_t sample_rate, bool apll)
{
    i2s_std_clk_config_t clk = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate);
    if (apll) {
        clk.clk_src = I2S_CLK_SRC_APLL;
    }
    return clk;
}

esp_err_t bt_shim_i2s_start(uint32_t sample_rate, int bck, int ws, int dout, int mclk, int slot_bits)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    /* 8 x 480 frames x 8 bytes (2 x 32-bit slots) = 30 KB, ~87 ms at 44.1 kHz. */
    chan_cfg.dma_desc_num = 8;
    chan_cfg.dma_frame_num = 480;
    esp_err_t err = i2s_new_channel(&chan_cfg, &s_tx, NULL);
    if (err != ESP_OK) {
        return err;
    }

    s_cfg = (i2s_std_config_t){
        .clk_cfg = make_clk(sample_rate, true),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(slot_bits == 32 ? I2S_DATA_BIT_WIDTH_32BIT : I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = mclk,
            .bclk = bck,
            .ws = ws,
            .dout = dout,
            .din = I2S_GPIO_UNUSED,
        },
    };
    s_apll = true;
    err = i2s_channel_init_std_mode(s_tx, &s_cfg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "APLL clock for %u Hz failed (%s), using PLL_160M",
                 (unsigned)sample_rate, esp_err_to_name(err));
        s_apll = false;
        s_cfg.clk_cfg = make_clk(sample_rate, false);
        err = i2s_channel_init_std_mode(s_tx, &s_cfg);
        if (err != ESP_OK) {
            return err;
        }
    }
    i2s_event_callbacks_t cbs = { .on_send_q_ovf = on_send_q_ovf };
    err = i2s_channel_register_event_callback(s_tx, &cbs, NULL);
    if (err != ESP_OK) {
        return err;
    }
    /* Strongest drive on MCLK: GPIO0 also carries the BOOT switch and the
       auto-reset transistor, whose capacitance rounds 11.3 MHz into a
       triangle. */
    if (mclk >= 0) {
        gpio_set_drive_capability(mclk, GPIO_DRIVE_CAP_3);
    }
    return i2s_channel_enable(s_tx);
}

uint32_t bt_shim_i2s_underruns(void)
{
    uint32_t n = s_underruns;
    s_underruns -= n;
    return n;
}

esp_err_t bt_shim_i2s_set_rate(uint32_t sample_rate)
{
    esp_err_t err = i2s_channel_disable(s_tx);
    if (err != ESP_OK) {
        return err;
    }
    i2s_std_clk_config_t clk = make_clk(sample_rate, s_apll);
    err = i2s_channel_reconfig_std_clock(s_tx, &clk);
    if (err != ESP_OK && s_apll) {
        ESP_LOGW(TAG, "APLL clock for %u Hz failed (%s), using PLL_160M",
                 (unsigned)sample_rate, esp_err_to_name(err));
        s_apll = false;
        clk = make_clk(sample_rate, false);
        err = i2s_channel_reconfig_std_clock(s_tx, &clk);
    }
    if (err != ESP_OK) {
        return err;
    }
    return i2s_channel_enable(s_tx);
}

size_t bt_shim_i2s_write(const uint8_t *data, size_t len)
{
    size_t written = 0;
    i2s_channel_write(s_tx, data, len, &written, 200);
    return written;
}

esp_err_t bt_shim_i2c_start(int sda, int scl)
{
    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    esp_err_t err = i2c_param_config(AMP_I2C_PORT, &cfg);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(AMP_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
}

esp_err_t bt_shim_i2c_write(uint8_t addr, const uint8_t *buf, size_t len)
{
    return i2c_master_write_to_device(AMP_I2C_PORT, addr, buf, len,
                                      pdMS_TO_TICKS(AMP_I2C_TIMEOUT_MS));
}

esp_err_t bt_shim_i2c_read(uint8_t addr, uint8_t *buf, size_t len)
{
    return i2c_master_read_from_device(AMP_I2C_PORT, addr, buf, len,
                                       pdMS_TO_TICKS(AMP_I2C_TIMEOUT_MS));
}

esp_err_t bt_shim_i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val)
{
    const uint8_t buf[2] = { reg, val };
    return i2c_master_write_to_device(AMP_I2C_PORT, addr, buf, sizeof(buf),
                                      pdMS_TO_TICKS(AMP_I2C_TIMEOUT_MS));
}

esp_err_t bt_shim_i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *val)
{
    return i2c_master_write_read_device(AMP_I2C_PORT, addr, &reg, 1, val, 1,
                                        pdMS_TO_TICKS(AMP_I2C_TIMEOUT_MS));
}

esp_err_t bt_shim_adc_add(int gpio)
{
    adc_unit_t unit;
    adc_channel_t chan;
    esp_err_t err = adc_oneshot_io_to_channel(gpio, &unit, &chan);
    if (err != ESP_OK) {
        return err;
    }
    if (unit != ADC_UNIT_1) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_adc == NULL) {
        adc_oneshot_unit_init_cfg_t unit_cfg = {
            .unit_id = ADC_UNIT_1,
            .ulp_mode = ADC_ULP_MODE_DISABLE,
        };
        err = adc_oneshot_new_unit(&unit_cfg, &s_adc);
        if (err != ESP_OK) {
            return err;
        }
    }
    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    return adc_oneshot_config_channel(s_adc, chan, &chan_cfg);
}

int bt_shim_adc_read_avg(int gpio, int samples)
{
    adc_unit_t unit;
    adc_channel_t chan;
    esp_err_t err = adc_oneshot_io_to_channel(gpio, &unit, &chan);
    if (err != ESP_OK) {
        return -(int)err;
    }
    /* Average without the lowest and highest sample (ADC spikes). */
    int sum = 0, lo = 4095, hi = 0;
    for (int i = 0; i < samples; i++) {
        int raw;
        err = adc_oneshot_read(s_adc, chan, &raw);
        if (err != ESP_OK) {
            return -(int)err;
        }
        sum += raw;
        lo = raw < lo ? raw : lo;
        hi = raw > hi ? raw : hi;
    }
    if (samples > 2) {
        return (sum - lo - hi) / (samples - 2);
    }
    return sum / samples;
}
