#include "bt_shim.h"

#include "esp_bt.h"
#include "driver/i2s_std.h"

static i2s_chan_handle_t s_tx;
static i2s_std_config_t s_cfg;

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

esp_err_t bt_shim_i2s_start(uint32_t sample_rate, int bck, int ws, int dout)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    chan_cfg.dma_desc_num = 16;
    chan_cfg.dma_frame_num = 480;
    esp_err_t err = i2s_new_channel(&chan_cfg, &s_tx, NULL);
    if (err != ESP_OK) {
        return err;
    }

    s_cfg = (i2s_std_config_t){
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = bck,
            .ws = ws,
            .dout = dout,
            .din = I2S_GPIO_UNUSED,
        },
    };
    err = i2s_channel_init_std_mode(s_tx, &s_cfg);
    if (err != ESP_OK) {
        return err;
    }
    return i2s_channel_enable(s_tx);
}

esp_err_t bt_shim_i2s_set_rate(uint32_t sample_rate)
{
    i2s_std_clk_config_t clk = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate);
    esp_err_t err = i2s_channel_disable(s_tx);
    if (err != ESP_OK) {
        return err;
    }
    err = i2s_channel_reconfig_std_clock(s_tx, &clk);
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
