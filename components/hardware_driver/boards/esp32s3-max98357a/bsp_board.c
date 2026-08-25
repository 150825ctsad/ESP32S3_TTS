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
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define ADC_I2S_CHANNEL 0

/* ------------------------------------------------------------------ */
/*  音频预处理：200Hz 高通 + 软限幅（不做数字增益 / 3kHz 峰化）        */
/* ------------------------------------------------------------------ */
#define AUDIO_PREPROCESS_EN   1

typedef struct {
    float b0, b1, b2, a1, a2;   /* 归一化系数 (RBJ cookbook) */
    float x1, x2, y1, y2;       /* 延迟单元 */
} biquad_t;

static biquad_t s_hpf = {  /* 高通 200Hz, Q=0.707, fs=16kHz */
    .b0 = 0.94597556f, .b1 = -1.89195112f, .b2 = 0.94597556f,
    .a1 = -1.88903308f, .a2 = 0.89487432f,
};

static inline float biquad_process(biquad_t *s, float x)
{
    float y = s->b0 * x + s->b1 * s->x1 + s->b2 * s->x2
                        - s->a1 * s->y1 - s->a2 * s->y2;
    s->x2 = s->x1; s->x1 = x;
    s->y2 = s->y1; s->y1 = y;
    return y;
}

/* 软拐点限幅：-2dBFS(≈26000) 以下线性，以上 4:1 压缩 */
static inline int16_t soft_clip(int32_t x)
{
    const int32_t th = 26000;
    if (x > th) {
        x = th + (x - th) / 4;
        if (x > 32767) x = 32767;
    } else if (x < -th) {
        x = -th + (x + th) / 4;
        if (x < -32768) x = -32768;
    }
    return (int16_t)x;
}

static void audio_preprocess(int16_t *pcm, int n)
{
    for (int i = 0; i < n; i++) {
        float x = biquad_process(&s_hpf, (float)pcm[i]);
        pcm[i] = soft_clip((int32_t)x);
    }
}

static const char *TAG = "MAX98357A";
static int s_play_sample_rate = 16000;
static int s_play_channel_format = 1;
static int s_bits_per_chan = 16;

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static i2s_chan_handle_t tx_handle = NULL;
static i2s_chan_handle_t rx_handle = NULL;
#endif

static audio_codec_data_if_t *play_data_if = NULL;
static audio_codec_if_t      *play_codec_if = NULL;
static esp_codec_dev_handle_t play_dev = NULL;

static audio_codec_data_if_t *rec_data_if = NULL;
static audio_codec_if_t      *rec_codec_if = NULL;
static esp_codec_dev_handle_t rec_dev = NULL;
static SemaphoreHandle_t s_play_lock = NULL;
static int s_out_vol = PLAYER_VOLUME;

/* ------------------------------------------------------------------ */
/*  I2S init (TX + RX 全双工，共用 BCLK/WS)                            */
/* ------------------------------------------------------------------ */

static esp_err_t bsp_i2s_init(i2s_port_t i2s_num, uint32_t sample_rate,
                              int channel_format, int bits_per_chan)
{
    esp_err_t ret_val = ESP_OK;

    /* TX 和 RX 共用同一个 I2S 控制器，BCLK/WS 自动同步。
     * MSM261 要求 fSCK = 64 × fWS（每帧 64 BCLK），
     * 用 32-bit STEREO slot：32×2 = 64 BCLK/帧，满足 MSM261 要求。
     * MAX98357A 只取高 16 位，32-bit slot 也能正常工作。 */
    i2s_slot_mode_t slot_mode = I2S_SLOT_MODE_STEREO;
    int slot_bits = 32;   /* 统一 32-bit slot，兼容 24-bit 麦克风和 16-bit 功放 */

    /* GDMA：6 描述符 × 240 帧 ≈ 90ms @16kHz；欠载自动清零，避免重复最后一帧 */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(i2s_num, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = 6;
    chan_cfg.dma_frame_num = 240;
    chan_cfg.auto_clear_after_cb = true;
    ret_val |= i2s_new_channel(&chan_cfg, &tx_handle, &rx_handle);

    /* TX: 32-bit slot，数据位宽 16-bit（高 16 位有效） */
    i2s_std_config_t tx_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(slot_bits, slot_mode),
        .gpio_cfg = {
            .mclk = GPIO_I2S_MCLK,
            .bclk = GPIO_I2S_SCLK,
            .ws   = GPIO_I2S_LRCK,
            .dout = GPIO_I2S_DOUT,
            .din  = GPIO_I2S_SDIN,
        },
    };
    /* TX slot 数据位宽设为 16-bit（在 32-bit slot 中） */
    tx_cfg.slot_cfg.data_bit_width = bits_per_chan;
    tx_cfg.slot_cfg.slot_bit_width = slot_bits;
    ret_val |= i2s_channel_init_std_mode(tx_handle, &tx_cfg);

    /* RX: 32-bit slot，数据位宽 24-bit（MSM261 输出 24-bit） */
    i2s_std_config_t rx_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(slot_bits, slot_mode),
        .gpio_cfg = {
            .mclk = GPIO_I2S_MCLK,
            .bclk = GPIO_I2S_SCLK,
            .ws   = GPIO_I2S_LRCK,
            .dout = GPIO_I2S_DOUT,
            .din  = GPIO_I2S_SDIN,
        },
    };
    rx_cfg.slot_cfg.data_bit_width = 24;
    rx_cfg.slot_cfg.slot_bit_width = slot_bits;
    /* MSM261 L/R 接 GND = 左声道，只收左 slot */
    rx_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;
    ret_val |= i2s_channel_init_std_mode(rx_handle, &rx_cfg);

    ret_val |= i2s_channel_enable(tx_handle);
    ret_val |= i2s_channel_enable(rx_handle);

    if (ret_val != ESP_OK) {
        ESP_LOGE(TAG, "I2S init failed");
    } else {
        ESP_LOGI(TAG, "I2S GDMA TX/RX desc=%u frame=%u (~%u ms @%lu Hz)",
                 (unsigned)chan_cfg.dma_desc_num,
                 (unsigned)chan_cfg.dma_frame_num,
                 (unsigned)(chan_cfg.dma_desc_num * chan_cfg.dma_frame_num * 1000 / sample_rate),
                 (unsigned long)sample_rate);
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
    if (rx_handle) {
        ret_val |= i2s_channel_disable(rx_handle);
        ret_val |= i2s_del_channel(rx_handle);
        rx_handle = NULL;
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
/*  ADC init (MSM261S4030H0R I2S MEMS microphone, dummy codec)         */
/* ------------------------------------------------------------------ */

static esp_err_t bsp_codec_adc_init(int sample_rate, int channel_format, int bits_per_chan)
{
    esp_err_t ret_val = ESP_OK;

    /* I2S data interface (RX）— 共用同一个 I2S 控制器 */
    audio_codec_i2s_cfg_t i2s_cfg = {
        .port = I2S_NUM_0,
        .rx_handle = rx_handle,
        .tx_handle = NULL,
    };
    rec_data_if = audio_codec_new_i2s_data(&i2s_cfg);
    if (rec_data_if == NULL) {
        ESP_LOGE(TAG, "Failed to create I2S data interface for ADC");
        return ESP_FAIL;
    }

    /* Dummy codec（MSM261 无 I2C 控制，纯 I2S 直出） */
    dummy_codec_cfg_t dummy_cfg = {
        .gpio_if     = NULL,
        .pa_pin      = -1,
        .pa_reverted = false,
    };
    rec_codec_if = dummy_codec_new(&dummy_cfg);
    if (rec_codec_if == NULL) {
        ESP_LOGE(TAG, "Failed to create dummy codec for ADC");
        return ESP_FAIL;
    }

    /* Input device */
    esp_codec_dev_cfg_t dev_cfg = {
        .codec_if = rec_codec_if,
        .data_if  = rec_data_if,
        .dev_type = ESP_CODEC_DEV_TYPE_IN,
    };
    rec_dev = esp_codec_dev_new(&dev_cfg);
    if (rec_dev == NULL) {
        ESP_LOGE(TAG, "Failed to create input codec device");
        return ESP_FAIL;
    }

    /* MSM261: 24-bit 数据，左声道 */
    esp_codec_dev_sample_info_t fs = {
        .bits_per_sample = 24,
        .sample_rate     = sample_rate,
        .channel         = 1,
    };
    ret_val = esp_codec_dev_open(rec_dev, &fs);
    if (ret_val != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open input codec device");
    }
    esp_codec_dev_set_in_gain(rec_dev, RECORD_VOLUME);

    return ret_val;
}

static esp_err_t bsp_codec_adc_deinit(void)
{
    if (rec_dev) {
        esp_codec_dev_close(rec_dev);
        esp_codec_dev_delete(rec_dev);
        rec_dev = NULL;
    }
    if (rec_codec_if) {
        audio_codec_delete_codec_if(rec_codec_if);
        rec_codec_if = NULL;
    }
    if (rec_data_if) {
        audio_codec_delete_data_if(rec_data_if);
        rec_data_if = NULL;
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

    if (s_play_lock == NULL) {
        s_play_lock = xSemaphoreCreateMutex();
    }

    esp_err_t ret = bsp_i2s_init(I2S_NUM_0, sample_rate, channel_format, bits_per_chan);
    if (ret != ESP_OK) return ret;

    ret = bsp_codec_dac_init(sample_rate, channel_format, bits_per_chan);
    if (ret != ESP_OK) return ret;

    ret = bsp_codec_adc_init(sample_rate, channel_format, bits_per_chan);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "ADC init failed, recording disabled");
        /* 录音失败不影响播放 */
    }

    ESP_LOGI(TAG, "Board init done");
    return ESP_OK;
}

esp_err_t bsp_audio_play(const int16_t *data, int length, TickType_t ticks_to_wait)
{
    if (tx_handle == NULL || data == NULL || length <= 0) {
        return ESP_FAIL;
    }

    TickType_t wait = ticks_to_wait ? ticks_to_wait : portMAX_DELAY;
    if (s_play_lock) xSemaphoreTake(s_play_lock, portMAX_DELAY);

    /* DRAM 对齐块，GDMA 直接搬走；禁止改 flash mmap 的 welcome.wav */
    static int16_t scratch[1024] __attribute__((aligned(8)));
    const uint8_t *src = (const uint8_t *)data;
    int remaining = length;
    esp_err_t ret = ESP_OK;
    while (remaining > 0) {
        int n = remaining > (int)sizeof(scratch) ? (int)sizeof(scratch) : remaining;
        memcpy(scratch, src, (size_t)n);
#if AUDIO_PREPROCESS_EN
        audio_preprocess(scratch, n / (int)sizeof(int16_t));
#endif
        /* 用户音量：MAX98357A 无模拟增益，只能在此缩放；100 为原幅度 */
        int ns = n / (int)sizeof(int16_t);
        int vol = s_out_vol;
        if (vol < 0) vol = 0;
        if (vol > 100) vol = 100;
        if (vol != 100) {
            for (int i = 0; i < ns; i++) {
                scratch[i] = (int16_t)(((int32_t)scratch[i] * vol) / 100);
            }
        }
        size_t written = 0;
        ret = i2s_channel_write(tx_handle, scratch, (size_t)n, &written, wait);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "I2S DMA write fail %s", esp_err_to_name(ret));
            break;
        }
        src += n;
        remaining -= n;
    }
    if (s_play_lock) xSemaphoreGive(s_play_lock);
    return ret;
}

esp_err_t bsp_audio_set_play_vol(int volume)
{
    if (volume < 0) volume = 0;
    if (volume > 100) volume = 100;
    s_out_vol = volume;
    if (play_dev) {
        esp_codec_dev_set_out_vol(play_dev, volume);
    }
    return ESP_OK;
}

esp_err_t bsp_audio_get_play_vol(int *volume)
{
    if (volume == NULL) return ESP_FAIL;
    *volume = s_out_vol;
    return ESP_OK;
}

/* 静音长度须 ≥ DMA 缓冲总时长(240ms)，才能保证残留语音全部流出到喇叭 */
#define FLUSH_SILENCE_SAMPLES  4800   /* 300ms @16kHz */

esp_err_t bsp_audio_flush(void)
{
    if (play_dev == NULL) return ESP_FAIL;
    /* 不能加 const：esp_codec_dev 软件音量会就地改写该缓冲，
     * const 数组位于 flash 映射区，写入会触发 Cache error panic */
    static int16_t silence[FLUSH_SILENCE_SAMPLES];   /* .bss 零初始化 */
    if (s_play_lock) xSemaphoreTake(s_play_lock, portMAX_DELAY);
    size_t written = 0;
    esp_err_t ret = ESP_FAIL;
    if (tx_handle) {
        ret = i2s_channel_write(tx_handle, silence, sizeof(silence), &written, portMAX_DELAY);
    }
    if (s_play_lock) xSemaphoreGive(s_play_lock);
    return ret;
}

/* -- Recording: MSM261S4030H0R I2S MEMS microphone -- */

esp_err_t bsp_get_feed_data(bool is_get_raw_channel, int16_t *buffer, int buffer_len)
{
    (void)is_get_raw_channel;
    if (rec_dev == NULL) return ESP_ERR_NOT_SUPPORTED;
    return esp_codec_dev_read(rec_dev, (uint8_t *)buffer, buffer_len);
}

int bsp_get_feed_channel(void)
{
    return 1;   /* MSM261 单声道 */
}

char *bsp_get_input_format(void)
{
    return "M";   /* Mono microphone */
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
