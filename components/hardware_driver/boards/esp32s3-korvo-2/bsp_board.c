/**
 *
 * @copyright Copyright 2021 Espressif Systems (Shanghai) Co. Ltd.
 *
 *      Licensed under the Apache License, Version 2.0 (the "License");
 *      you may not use this file except in compliance with the License.
 *      You may obtain a copy of the License at
 *
 *               http://www.apache.org/licenses/LICENSE-2.0
 *
 *      Unless required by applicable law or agreed to in writing, software
 *      distributed under the License is distributed on an "AS IS" BASIS,
 *      WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *      See the License for the specific language governing permissions and
 *      limitations under the License.
 */

#include <stdlib.h>
#include <string.h>
#include "bsp_board.h"
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
#include "driver/i2s_std.h"
#include "driver/i2s_tdm.h"
#include "soc/soc_caps.h"
#else
#include "driver/i2s.h"
#endif
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_rom_sys.h"
#include "esp_check.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#if ((SOC_SDMMC_HOST_SUPPORTED) && (FUNC_SDMMC_EN))
#include "driver/sdmmc_host.h"
#endif /* ((SOC_SDMMC_HOST_SUPPORTED) && (FUNC_SDMMC_EN)) */

#define GPIO_MUTE_NUM   GPIO_NUM_1
#define GPIO_MUTE_LEVEL 1
#define ACK_CHECK_EN   0x1     /*!< I2C master will check ack from slave*/
#define ADC_I2S_CHANNEL 1      /* ES8311 下混为单声道后交给上层 */
#define ES8311_I2S_CH   2      /* ES8311 I2S 仍是立体声（左=MIC） */
static sdmmc_card_t *card;
static const char *TAG = "board";
static int s_play_sample_rate = 16000;
static int s_play_channel_format = 1;
static int s_bits_per_chan = 16;

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
static i2s_chan_handle_t                tx_handle = NULL;        // I2S tx channel handler
static i2s_chan_handle_t                rx_handle = NULL;        // I2S rx channel handler
#endif
static i2c_master_bus_handle_t          s_i2c_bus = NULL;
static audio_codec_data_if_t *record_data_if = NULL;
static audio_codec_ctrl_if_t *record_ctrl_if = NULL;
static audio_codec_if_t *record_codec_if = NULL;
static esp_codec_dev_handle_t record_dev = NULL;

static audio_codec_data_if_t *play_data_if = NULL;
static audio_codec_ctrl_if_t *play_ctrl_if = NULL;
static audio_codec_gpio_if_t *play_gpio_if = NULL;
static audio_codec_if_t *play_codec_if = NULL;
static esp_codec_dev_handle_t play_dev = NULL;


esp_err_t bsp_i2c_init(i2c_port_t i2c_num, uint32_t clk_speed)
{
    if (s_i2c_bus) {
        return ESP_OK;
    }
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = i2c_num,
        .sda_io_num = GPIO_I2C_SDA,
        .scl_io_num = GPIO_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    (void)clk_speed;
    return i2c_new_master_bus(&bus_cfg, &s_i2c_bus);
}

esp_err_t bsp_codec_adc_init(int sample_rate)
{
    (void)sample_rate;
    if (play_dev == NULL) {
        ESP_LOGE(TAG, "ES8311 not ready, cannot share for ADC");
        return ESP_FAIL;
    }
    record_dev = play_dev;
    ESP_LOGI(TAG, "ADC uses ES8311");
    return ESP_OK;
}

esp_err_t bsp_codec_dac_init(int sample_rate, int channel_format, int bits_per_chan)
{
    esp_err_t ret_val = ESP_OK;

    // Do initialize of related interface: data_if, ctrl_if and gpio_if
    audio_codec_i2s_cfg_t i2s_cfg = {
        .port = I2S_NUM_1,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        .rx_handle = rx_handle,
        .tx_handle = tx_handle,
#endif
    };
    play_data_if = audio_codec_new_i2s_data(&i2s_cfg);

    audio_codec_i2c_cfg_t i2c_cfg = {
        .port = I2C_NUM,
        .addr = ES8311_CODEC_DEFAULT_ADDR,
        .bus_handle = s_i2c_bus,
        .clock_speed_hz = I2C_CLK,
    };
    play_ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    play_gpio_if = audio_codec_new_gpio();
    es8311_codec_cfg_t es8311_cfg = {
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_BOTH,
        .ctrl_if = play_ctrl_if,
        .gpio_if = play_gpio_if,
        .pa_pin = GPIO_PWR_CTRL,
        .use_mclk = false,
        .no_dac_ref = true,
    };
    play_codec_if = es8311_codec_new(&es8311_cfg);
    esp_codec_dev_cfg_t dev_cfg = {
        .codec_if = play_codec_if,
        .data_if = play_data_if,
        .dev_type = ESP_CODEC_DEV_TYPE_IN_OUT,
    };
    play_dev = esp_codec_dev_new(&dev_cfg);

    esp_codec_dev_sample_info_t fs = {
        .bits_per_sample = bits_per_chan,
        .sample_rate = sample_rate,
        .channel = channel_format,
    };
    esp_codec_dev_set_in_gain(play_dev, RECORD_VOLUME);
    esp_codec_dev_set_out_vol(play_dev, PLAYER_VOLUME);
    ret_val = esp_codec_dev_open(play_dev, &fs);
    if (ret_val != ESP_OK) {
        ESP_LOGE(TAG, "ES8311 open fail: %s", esp_err_to_name(ret_val));
    } else {
        ESP_LOGI(TAG, "ES8311 open ok (DAC+ADC)");
    }

    return ret_val;
}

static esp_err_t bsp_codec_adc_deinit()
{
    /* record_dev 与 play_dev 共用 ES8311，由 dac_deinit 释放 */
    record_dev = NULL;
    return ESP_OK;
}

static esp_err_t bsp_codec_dac_deinit()
{
    esp_err_t ret_val = ESP_OK;

    if (play_dev) {
        esp_codec_dev_close(play_dev);
        esp_codec_dev_delete(play_dev);
        play_dev = NULL;
    }

    // Delete codec interface
    if (play_codec_if) {
        audio_codec_delete_codec_if(play_codec_if);
        play_codec_if = NULL;
    }
    
    // Delete codec control interface
    if (play_ctrl_if) {
        audio_codec_delete_ctrl_if(play_ctrl_if);
        play_ctrl_if = NULL;
    }
    
    if (play_gpio_if) {
        audio_codec_delete_gpio_if(play_gpio_if);
        play_gpio_if = NULL;
    }
    
    // Delete codec data interface
    if (play_data_if) {
        audio_codec_delete_data_if(play_data_if);
        play_data_if = NULL;
    }

    return ret_val;
}

esp_err_t bsp_audio_set_play_vol(int volume)
{
    if (!play_dev) {
        ESP_LOGE(TAG, "DAC codec init fail");
        return ESP_FAIL;
    }
    esp_codec_dev_set_out_vol(play_dev, volume);
    return ESP_OK;
}

esp_err_t bsp_audio_get_play_vol(int *volume)
{
    if (!play_dev) {
        ESP_LOGE(TAG, "DAC codec init fail");
        return ESP_FAIL;
    }
    esp_codec_dev_get_out_vol(play_dev, volume);
    return ESP_OK;
}

// static esp_err_t bsp_i2s_init(i2s_port_t i2s_num, uint32_t sample_rate, i2s_channel_fmt_t channel_format, i2s_bits_per_chan_t bits_per_chan)
static esp_err_t bsp_i2s_init(i2s_port_t i2s_num, uint32_t sample_rate, int channel_format, int bits_per_chan)
{
    esp_err_t ret_val = ESP_OK;

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    i2s_slot_mode_t channel_fmt = I2S_SLOT_MODE_STEREO;
    if (channel_format == 1) {
        channel_fmt = I2S_SLOT_MODE_MONO;
    } else if (channel_format == 2) {
        channel_fmt = I2S_SLOT_MODE_STEREO;
    } else {
        ESP_LOGE(TAG, "Unable to configure channel_format %d", channel_format);
        channel_format = 1;
        channel_fmt = I2S_SLOT_MODE_MONO;
    }

    if (bits_per_chan != 16 && bits_per_chan != 32) {
        ESP_LOGE(TAG, "Unable to configure bits_per_chan %d", bits_per_chan);
        bits_per_chan = 32;
    }

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(i2s_num, I2S_ROLE_MASTER);
    ret_val |= i2s_new_channel(&chan_cfg, &tx_handle, &rx_handle);
    i2s_std_config_t std_cfg = I2S_CONFIG_DEFAULT(sample_rate, channel_fmt, bits_per_chan);
    ret_val |= i2s_channel_init_std_mode(tx_handle, &std_cfg);
    ret_val |= i2s_channel_init_std_mode(rx_handle, &std_cfg);
    ret_val |= i2s_channel_enable(tx_handle);
    ret_val |= i2s_channel_enable(rx_handle);
#else
    i2s_channel_fmt_t channel_fmt = I2S_CHANNEL_FMT_RIGHT_LEFT;
    if (channel_format == 1) {
        channel_fmt = I2S_CHANNEL_FMT_ONLY_LEFT;
    } else if (channel_format == 2) {
        channel_fmt = I2S_CHANNEL_FMT_RIGHT_LEFT;
    } else {
        ESP_LOGE(TAG, "Unable to configure channel_format %d", channel_format);
        channel_format = 1;
        channel_fmt = I2S_CHANNEL_FMT_ONLY_LEFT;
    }

    if (bits_per_chan != 16 && bits_per_chan != 32) {
        ESP_LOGE(TAG, "Unable to configure bits_per_chan %d", bits_per_chan);
        bits_per_chan = 16;
    }

    i2s_config_t i2s_config = I2S_CONFIG_DEFAULT(sample_rate, channel_fmt, bits_per_chan);

    i2s_pin_config_t pin_config = {
        .bck_io_num = GPIO_I2S_SCLK,
        .ws_io_num = GPIO_I2S_LRCK,
        .data_out_num = GPIO_I2S_DOUT,
        .data_in_num = GPIO_I2S_SDIN,
        .mck_io_num = GPIO_I2S_MCLK,
    };

    ret_val |= i2s_driver_install(i2s_num, &i2s_config, 0, NULL);
    ret_val |= i2s_set_pin(i2s_num, &pin_config);
#endif

    return ret_val;
}

static esp_err_t bsp_i2s_deinit(i2s_port_t i2s_num)
{
    esp_err_t ret_val = ESP_OK;

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    if (i2s_num == I2S_NUM_1 && rx_handle) {
        ret_val |= i2s_channel_disable(rx_handle);
        ret_val |= i2s_del_channel(rx_handle);
        rx_handle = NULL;
    } else if (i2s_num == I2S_NUM_0  && tx_handle) {
        ret_val |= i2s_channel_disable(tx_handle);
        ret_val |= i2s_del_channel(tx_handle);
        tx_handle = NULL;
    }
#else
    ret_val |= i2s_stop(i2s_num);
    ret_val |= i2s_driver_uninstall(i2s_num);
#endif

    return ret_val;
}

static esp_err_t bsp_codec_init(int adc_sample_rate, int dac_sample_rate, int dac_channel_format, int dac_bits_per_chan)
{
    esp_err_t ret_val = ESP_OK;
    /* 先开 ES8311 全双工，再让 ADC 复用同一设备（不再访问 ES7210） */
    ret_val |= bsp_codec_dac_init(dac_sample_rate, dac_channel_format, dac_bits_per_chan);
    ret_val |= bsp_codec_adc_init(adc_sample_rate);
    return ret_val;
}

static esp_err_t bsp_codec_deinit()
{
    esp_err_t ret_val = ESP_OK;

    ret_val |= bsp_codec_adc_deinit();
    ret_val |= bsp_codec_dac_deinit();
    return ret_val;
}

esp_err_t bsp_audio_play(const int16_t* data, int length, TickType_t ticks_to_wait)
{
    (void)ticks_to_wait;
    if (!play_dev || data == NULL || length <= 0) {
        return ESP_FAIL;
    }

    /* 应用层是 16-bit 单声道；I2S/ES8311 是 16-bit 立体声，复制到 L/R */
    int n = length / (int)sizeof(int16_t);
    int16_t *st = (int16_t *)malloc((size_t)n * 2 * sizeof(int16_t));
    if (st == NULL) {
        return ESP_ERR_NO_MEM;
    }
    for (int i = 0; i < n; i++) {
        st[2 * i] = data[i];
        st[2 * i + 1] = data[i];
    }
    esp_err_t ret = esp_codec_dev_write(play_dev, st, n * 2 * (int)sizeof(int16_t));
    free(st);
    return ret;
}

esp_err_t bsp_get_feed_data(bool is_get_raw_channel, int16_t *buffer, int buffer_len)
{
    if (!record_dev) {
        return ESP_FAIL;
    }

    int mono_samples = buffer_len / (int)sizeof(int16_t);
    if (mono_samples <= 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (mono_samples > 512) {
        mono_samples = 512;
    }

    static int16_t stereo[512 * ES8311_I2S_CH];
    int stereo_bytes = mono_samples * ES8311_I2S_CH * (int)sizeof(int16_t);
    esp_err_t ret = esp_codec_dev_read(record_dev, stereo, stereo_bytes);
    if (ret != ESP_OK) {
        return ret;
    }

    uint32_t abs_l = 0, abs_r = 0, peak_l = 0, peak_r = 0;
    for (int i = 0; i < mono_samples; i++) {
        int16_t l = stereo[i * ES8311_I2S_CH];
        int16_t r = stereo[i * ES8311_I2S_CH + 1];
        buffer[i] = l;
        int vl = l, vr = r;
        uint32_t al = (uint32_t)(vl < 0 ? -vl : vl);
        uint32_t ar = (uint32_t)(vr < 0 ? -vr : vr);
        abs_l += al;
        abs_r += ar;
        if (al > peak_l) peak_l = al;
        if (ar > peak_r) peak_r = ar;
    }

    static bool dumped;
    if (!dumped) {
        dumped = true;
        ESP_LOGI(TAG, "adc raw L=%d R=%d  L2=%d R2=%d",
                 (int)stereo[0], (int)stereo[1], (int)stereo[2], (int)stereo[3]);
    }

    static uint32_t acc_l, acc_r, pk_l, pk_r, frames;
    static TickType_t t_log;
    acc_l += abs_l / (uint32_t)mono_samples;
    acc_r += abs_r / (uint32_t)mono_samples;
    if (peak_l > pk_l) pk_l = peak_l;
    if (peak_r > pk_r) pk_r = peak_r;
    frames++;
    // TickType_t now = xTaskGetTickCount();
    // if (t_log == 0 || (now - t_log) >= pdMS_TO_TICKS(1000)) {
    //     ESP_LOGI(TAG, "adc L|avg|=%u peak=%u  R|avg|=%u peak=%u  frames=%u",
    //              (unsigned)(frames ? acc_l / frames : 0), (unsigned)pk_l,
    //              (unsigned)(frames ? acc_r / frames : 0), (unsigned)pk_r,
    //              (unsigned)frames);
    //     acc_l = acc_r = pk_l = pk_r = frames = 0;
    //     t_log = now;
    // }

    (void)is_get_raw_channel;
    return ESP_OK;
}

int bsp_get_feed_channel(void)
{
    return ADC_I2S_CHANNEL;
}

char* bsp_get_input_format(void)
{
    return "M";
}

esp_err_t bsp_board_init(uint32_t sample_rate, int channel_format, int bits_per_chan)
{
    /*!< Initialize I2C bus, used for audio codec*/
    bsp_i2c_init(I2C_NUM, I2C_CLK);
    (void)sample_rate;
    (void)channel_format;
    (void)bits_per_chan;
    /* 与小智一致：16 kHz / 16-bit / 立体声，应用层再下混成单声道 */
    s_play_sample_rate = 16000;
    s_play_channel_format = 2;
    s_bits_per_chan = 16;

    bsp_i2s_init(I2S_NUM_1, 16000, 2, 16);
    bsp_codec_init(16000, 16000, 2, 16);
    /* Initialize PA */
    // gpio_config_t  io_conf;
    // memset(&io_conf, 0, sizeof(io_conf));
    // io_conf.intr_type = GPIO_INTR_DISABLE;
    // io_conf.mode = GPIO_MODE_OUTPUT;
    // io_conf.pin_bit_mask = ((1ULL << GPIO_PWR_CTRL));
    // io_conf.pull_down_en = 0;
    // io_conf.pull_up_en = 0;
    // gpio_config(&io_conf);
    // gpio_set_level(GPIO_PWR_CTRL, 1);
    return ESP_OK;
}

esp_err_t bsp_sdcard_init(char *mount_point, size_t max_files)
{
    if (NULL != card) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Check if SD crad is supported */
    if (!FUNC_SDMMC_EN && !FUNC_SDSPI_EN) {
        ESP_LOGE(TAG, "SDMMC and SDSPI not supported on this board!");
        return ESP_ERR_NOT_SUPPORTED;
    }

    esp_err_t ret_val = ESP_OK;

    /**
     * @brief Options for mounting the filesystem.
     *   If format_if_mount_failed is set to true, SD card will be partitioned and
     *   formatted in case when mounting fails.
     *
     */
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = max_files,
        .allocation_unit_size = 16 * 1024
    };

    /**
     * @brief Use settings defined above to initialize SD card and mount FAT filesystem.
     *   Note: esp_vfs_fat_sdmmc/sdspi_mount is all-in-one convenience functions.
     *   Please check its source code and implement error recovery when developing
     *   production applications.
     *
     */
    sdmmc_host_t host =
#if FUNC_SDMMC_EN
        SDMMC_HOST_DEFAULT();
#else
        SDSPI_HOST_DEFAULT();
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = GPIO_SDSPI_MOSI,
        .miso_io_num = GPIO_SDSPI_MISO,
        .sclk_io_num = GPIO_SDSPI_SCLK,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = 4000,
    };
    ret_val = spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret_val != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return ret_val;
    }
#endif

    /**
     * @brief This initializes the slot without card detect (CD) and write protect (WP) signals.
     *   Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
     *
     */
#if FUNC_SDMMC_EN
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
#else
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
#endif

#if FUNC_SDMMC_EN
    /* Config SD data width. 0, 4 or 8. Currently for SD card, 8 bit is not supported. */
    slot_config.width = SDMMC_BUS_WIDTH;

    /**
     * @brief On chips where the GPIOs used for SD card can be configured, set them in
     *   the slot_config structure.
     *
     */
#if SOC_SDMMC_USE_GPIO_MATRIX
    slot_config.clk = GPIO_SDMMC_CLK;
    slot_config.cmd = GPIO_SDMMC_CMD;
    slot_config.d0 = GPIO_SDMMC_D0;
    slot_config.d1 = GPIO_SDMMC_D1;
    slot_config.d2 = GPIO_SDMMC_D2;
    slot_config.d3 = GPIO_SDMMC_D3;
#endif
    slot_config.cd = GPIO_SDMMC_DET;
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
#else
    slot_config.gpio_cs = GPIO_SDSPI_CS;
    slot_config.host_id = host.slot;
#endif
    /**
     * @brief Enable internal pullups on enabled pins. The internal pullups
     *   are insufficient however, please make sure 10k external pullups are
     *   connected on the bus. This is for debug / example purpose only.
     */

    /* get FAT filesystem on SD card registered in VFS. */
    ret_val =
#if FUNC_SDMMC_EN
        esp_vfs_fat_sdmmc_mount(mount_point, &host, &slot_config, &mount_config, &card);
#else
        esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);
#endif

    /* Check for SDMMC mount result. */
    if (ret_val != ESP_OK) {
        if (ret_val == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret_val));
        }
        return ret_val;
    }

    /* Card has been initialized, print its properties. */
    sdmmc_card_print_info(stdout, card);

    return ret_val;
}

esp_err_t bsp_sdcard_deinit(char *mount_point)
{
    if (NULL == mount_point) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Unmount an SD card from the FAT filesystem and release resources acquired */
    esp_err_t ret_val = esp_vfs_fat_sdcard_unmount(mount_point, card);

    /* Make SD/MMC card information structure pointer NULL */
    card = NULL;

    return ret_val;
}
