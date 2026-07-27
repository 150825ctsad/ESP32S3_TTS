/**
 * @brief ESP32-S3 + MAX98357A board implementation
 *
 * MAX98357A is a pure I2S slave amplifier with no I2C control.
 * Audio output goes through I2S TX -> MAX98357A -> speaker.
 * No microphone / ADC path on this board.
 */

#include "string.h"
#include "bsp_board.h"
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
#include "driver/i2s_std.h"
#else
#include "driver/i2s.h"
#endif
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"

#define ADC_I2S_CHANNEL 0

static const char *TAG = "MAX98357A";
static int s_play_sample_rate = 16000;
static int s_play_channel_format = 1;
static int s_bits_per_chan = 16;

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static i2s_chan_handle_t tx_handle = NULL;
#endif

static audio_codec_data_if_t *play_data_if = NULL;
static audio_codec_if_t      *play_codec_if = NULL;
static esp_codec_dev_handle_t play_dev = NULL;

/* ------------------------------------------------------------------ */
/*  I2S init (TX only)                                                */
/* ------------------------------------------------------------------ */

static esp_err_t bsp_i2s_init(i2s_port_t i2s_num, uint32_t sample_rate,
                              int channel_format, int bits_per_chan)
{
    esp_err_t ret_val = ESP_OK;

    /* MAX98357A requires BCLK/LRCK ratio ≥ 32.
     * In MONO mode with 16-bit slots: ratio = 16 → amp won't lock.
     * Force STEREO slot mode: ratio = 16*2 = 32 (the minimum). */
    i2s_slot_mode_t slot_mode = I2S_SLOT_MODE_STEREO;
    if (channel_format != 1 && channel_format != 2) {
        ESP_LOGW(TAG, "Unsupported channel_format %d, fallback to stereo", channel_format);
    }

    if (bits_per_chan != 16 && bits_per_chan != 24 && bits_per_chan != 32) {
        ESP_LOGW(TAG, "Unsupported bits_per_chan %d, fallback to 16", bits_per_chan);
        bits_per_chan = 16;
    }

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(i2s_num, I2S_ROLE_MASTER);
    ret_val |= i2s_new_channel(&chan_cfg, &tx_handle, NULL);  // TX only

    i2s_std_config_t std_cfg = I2S_CONFIG_DEFAULT(sample_rate, slot_mode, bits_per_chan);
    ret_val |= i2s_channel_init_std_mode(tx_handle, &std_cfg);
    ret_val |= i2s_channel_enable(tx_handle);

    if (ret_val != ESP_OK) {
        ESP_LOGE(TAG, "I2S init failed");
    }
    return ret_val;
}

static esp_err_t bsp_i2s_deinit(i2s_port_t i2s_num)
{
    esp_err_t ret_val = ESP_OK;
    if (tx_handle) {
        ret_val |= i2s_channel_disable(tx_handle);
        ret_val |= i2s_del_channel(tx_handle);
        tx_handle = NULL;
    }
    return ret_val;
}

/* ------------------------------------------------------------------ */
/*  DAC init (dummy codec + I2S data interface)                       */
/* ------------------------------------------------------------------ */

static esp_err_t bsp_codec_dac_init(int sample_rate, int channel_format, int bits_per_chan)
{
    esp_err_t ret_val = ESP_OK;

    // I2S data interface (TX only)
    audio_codec_i2s_cfg_t i2s_cfg = {
        .port = I2S_NUM_0,
        .rx_handle = NULL,
        .tx_handle = tx_handle,
    };
    play_data_if = audio_codec_new_i2s_data(&i2s_cfg);
    if (play_data_if == NULL) {
        ESP_LOGE(TAG, "Failed to create I2S data interface");
        return ESP_FAIL;
    }

    // Dummy codec (no real codec chip, just PA control if wired)
    dummy_codec_cfg_t dummy_cfg = {
        .gpio_if     = NULL,
        .pa_pin      = GPIO_PWR_CTRL,
        .pa_reverted = (GPIO_PWR_ON_LEVEL == 0),
    };
    play_codec_if = dummy_codec_new(&dummy_cfg);
    if (play_codec_if == NULL) {
        ESP_LOGE(TAG, "Failed to create dummy codec");
        return ESP_FAIL;
    }

    // Output device
    esp_codec_dev_cfg_t dev_cfg = {
        .codec_if = play_codec_if,
        .data_if  = play_data_if,
        .dev_type = ESP_CODEC_DEV_TYPE_OUT,
    };
    play_dev = esp_codec_dev_new(&dev_cfg);
    if (play_dev == NULL) {
        ESP_LOGE(TAG, "Failed to create output codec device");
        return ESP_FAIL;
    }

    esp_codec_dev_sample_info_t fs = {
        .bits_per_sample = bits_per_chan,
        .sample_rate     = sample_rate,
        .channel         = channel_format,
    };
    ret_val = esp_codec_dev_open(play_dev, &fs);
    if (ret_val != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open output codec device");
    }
    esp_codec_dev_set_out_vol(play_dev, PLAYER_VOLUME);

    return ret_val;
}

static esp_err_t bsp_codec_dac_deinit(void)
{
    if (play_dev) {
        esp_codec_dev_close(play_dev);
        esp_codec_dev_delete(play_dev);
        play_dev = NULL;
    }
    if (play_codec_if) {
        audio_codec_delete_codec_if(play_codec_if);
        play_codec_if = NULL;
    }
    if (play_data_if) {
        audio_codec_delete_data_if(play_data_if);
        play_data_if = NULL;
    }
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/*  Public API                                                        */
/* ------------------------------------------------------------------ */

esp_err_t bsp_board_init(uint32_t sample_rate, int channel_format, int bits_per_chan)
{
    s_play_sample_rate   = sample_rate;
    s_play_channel_format = channel_format;
    s_bits_per_chan      = bits_per_chan;

    ESP_LOGI(TAG, "Init I2S: rate=%lu ch=%d bits=%d", sample_rate, channel_format, bits_per_chan);

    esp_err_t ret = bsp_i2s_init(I2S_NUM_0, sample_rate, channel_format, bits_per_chan);
    if (ret != ESP_OK) return ret;

    ret = bsp_codec_dac_init(sample_rate, channel_format, bits_per_chan);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "Board init done");
    return ESP_OK;
}

esp_err_t bsp_audio_play(const int16_t *data, int length, TickType_t ticks_to_wait)
{
    if (play_dev == NULL) {
        return ESP_FAIL;
    }
    return esp_codec_dev_write(play_dev, (void *)data, length);
}

esp_err_t bsp_audio_set_play_vol(int volume)
{
    if (play_dev == NULL) return ESP_FAIL;
    return esp_codec_dev_set_out_vol(play_dev, volume);
}

esp_err_t bsp_audio_get_play_vol(int *volume)
{
    if (play_dev == NULL) return ESP_FAIL;
    return esp_codec_dev_get_out_vol(play_dev, volume);
}

esp_err_t bsp_audio_flush(void)
{
    if (play_dev == NULL) return ESP_FAIL;
    // Write ~200ms of silence to flush I2S DMA pipeline
    // 16kHz mono 16-bit: 3200 samples * 2 bytes = 6400 bytes
    int16_t *silence = calloc(3200, sizeof(int16_t));
    if (silence == NULL) return ESP_ERR_NO_MEM;
    esp_err_t ret = esp_codec_dev_write(play_dev, silence, 3200 * sizeof(int16_t));
    free(silence);
    return ret;
}

/* -- Stubs (no microphone / recording on this board) -- */

esp_err_t bsp_get_feed_data(bool is_get_raw_channel, int16_t *buffer, int buffer_len)
{
    (void)is_get_raw_channel;
    (void)buffer;
    (void)buffer_len;
    return ESP_ERR_NOT_SUPPORTED;
}

int bsp_get_feed_channel(void)
{
    return 0;
}

char *bsp_get_input_format(void)
{
    return "NONE";
}

/* -- SD card -- stubbed out when FUNC_SDMMC_EN == 0 -- */

esp_err_t bsp_sdcard_init(char *mount_point, size_t max_files)
{
    ESP_LOGW(TAG, "SD card not configured on this board");
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t bsp_sdcard_deinit(char *mount_point)
{
    return ESP_OK;
}
