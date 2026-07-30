/**
 * @brief ESP32-S3 + MAX98357A custom board pin definitions
 *
 * MAX98357A is a pure I2S Class-D amplifier (no I2C codec control).
 * Required connections: BCLK, LRCK(WS), DOUT(DIN on MAX98357A)
 * Optional:       SD_MODE pin (PA enable/disable), GAIN pin
 *
 * Change the GPIO values below to match your actual wiring.
 */
#pragma once

#include "driver/gpio.h"
#include "esp_idf_version.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "esp_codec_dev_os.h"
#include "dummy_codec.h"

/**
 * @brief I2C - not used (MAX98357A has no I2C control interface)
 */
#define FUNC_I2C_EN     (0)
#define I2C_NUM         (I2C_NUM_0)
#define I2C_CLK         (600000)
#define GPIO_I2C_SCL    (GPIO_NUM_NC)
#define GPIO_I2C_SDA    (GPIO_NUM_NC)

/**
 * @brief SD card - disabled by default
 */
#define FUNC_SDMMC_EN   (0)
#define SDMMC_BUS_WIDTH (1)
#define GPIO_SDMMC_CLK  (GPIO_NUM_NC)
#define GPIO_SDMMC_CMD  (GPIO_NUM_NC)
#define GPIO_SDMMC_D0   (GPIO_NUM_NC)
#define GPIO_SDMMC_D1   (GPIO_NUM_NC)
#define GPIO_SDMMC_D2   (GPIO_NUM_NC)
#define GPIO_SDMMC_D3   (GPIO_NUM_NC)
#define GPIO_SDMMC_DET  (GPIO_NUM_NC)

#define FUNC_SDSPI_EN       (0)
#define SDSPI_HOST          (SPI2_HOST)
#define GPIO_SDSPI_CS       (GPIO_NUM_NC)
#define GPIO_SDSPI_SCLK     (GPIO_NUM_NC)
#define GPIO_SDSPI_MISO     (GPIO_NUM_NC)
#define GPIO_SDSPI_MOSI     (GPIO_NUM_NC)

/**
 * @brief I2S pins for MAX98357A (TX only)
 *
 * These are commonly-used ESP32-S3 pins. Adjust to match your wiring.
 */
#define FUNC_I2S_EN         (1)
#define GPIO_I2S_LRCK       (GPIO_NUM_12)   /* WS / word select */
#define GPIO_I2S_MCLK       (GPIO_NUM_NC)   /* MAX98357A has internal PLL, no MCLK needed */
#define GPIO_I2S_SCLK       (GPIO_NUM_11)   /* BCLK / bit clock */
#define GPIO_I2S_SDIN       (GPIO_NUM_NC)   /* no microphone input */
#define GPIO_I2S_DOUT       (GPIO_NUM_10)    /* DIN on MAX98357A */

/**
 * @brief Secondary I2S - not used
 */
#define FUNC_I2S0_EN         (0)
#define GPIO_I2S0_LRCK       (GPIO_NUM_NC)
#define GPIO_I2S0_MCLK       (GPIO_NUM_NC)
#define GPIO_I2S0_SCLK       (GPIO_NUM_NC)
#define GPIO_I2S0_SDIN       (GPIO_NUM_NC)
#define GPIO_I2S0_DOUT       (GPIO_NUM_NC)

/**
 * @brief Recording - not supported on this board
 */
#define RECORD_VOLUME   (0)

/**
 * @brief Default playback volume (0-100)
 */
#define PLAYER_VOLUME   (50)

/**
 * @brief PA power control via MAX98357A SD_MODE pin
 *
 * Set FUNC_PWR_CTRL to 1 and assign GPIO_PWR_CTRL if you have
 * the SD_MODE pin wired to a GPIO. Otherwise the amp runs continuously.
 */
#define FUNC_PWR_CTRL       (0)
#define GPIO_PWR_CTRL       (GPIO_NUM_NC)
#define GPIO_PWR_ON_LEVEL   (1)

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)

#define I2S_CONFIG_DEFAULT(sample_rate, channel_fmt, bits_per_chan) { \
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate), \
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(bits_per_chan, channel_fmt), \
        .gpio_cfg = { \
            .mclk = GPIO_I2S_MCLK, \
            .bclk = GPIO_I2S_SCLK, \
            .ws   = GPIO_I2S_LRCK, \
            .dout = GPIO_I2S_DOUT, \
            .din  = GPIO_I2S_SDIN, \
        }, \
    }

#else

#define I2S_CONFIG_DEFAULT(sample_rate, channel_fmt, bits_per_chan) { \
    .mode                   = I2S_MODE_MASTER | I2S_MODE_TX, \
    .sample_rate            = sample_rate, \
    .bits_per_sample        = bits_per_chan, \
    .channel_format         = channel_fmt, \
    .communication_format   = I2S_COMM_FORMAT_STAND_I2S, \
    .intr_alloc_flags       = ESP_INTR_FLAG_LEVEL1, \
    .dma_buf_count          = 6, \
    .dma_buf_len            = 160, \
    .use_apll               = false, \
    .tx_desc_auto_clear     = true, \
    .fixed_mclk             = 0, \
    .mclk_multiple          = I2S_MCLK_MULTIPLE_DEFAULT, \
    .bits_per_chan          = bits_per_chan, \
}

#endif
