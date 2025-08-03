#include "driver/i2s.h"
#include "soc/io_mux_reg.h"
#include "soc/soc.h"
#include "driver/gpio.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include "esp_log.h"
#include "soc/rtc.h"
#include "esp_private/rtc_clk.h"
#include "esp_private/esp_sleep_internal.h"
#include "soc/rtc_periph.h"
#include "soc/sens_reg.h"
#include "soc/soc_caps.h"
#include "soc/chip_revision.h"
#include "hal/efuse_ll.h"
#include "hal/efuse_hal.h"
#include "soc/gpio_struct.h"
#include "hal/gpio_ll.h"
#include "sdkconfig.h"
#include "esp_rom_sys.h"
#include "esp_rom_gpio.h"
#include "esp32/rom/rtc.h"
#include "hal/clk_tree_ll.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/io_mux_reg.h"
#include "esp_timer.h"
#include "esp32/rom/rtc.h"
#include <math.h>
#include "fm_tx.h"


#include "rickroll.h"

#define FM_CARRIER_HZ   100000000UL     // FM Carrier Frequency
#define MAX_DEV_HZ      62000UL        // ±75 kHz standard broadcast
#define WAV_SR_HZ       8000           // Sample Rate (8 kHz)

#if !CONFIG_IDF_TARGET_ESP32
    #error "Questo progetto richiede un chip con APLL (ESP32 D0WD / WROOM / WROVER). \
            Le serie S2/S3/C3 non la possiedono."
#endif

static fm_apll_cfg_t fm_calc_apll(uint32_t fout_hz, uint32_t dev_hz);

static fm_apll_cfg_t g_apll;

// void fm_apll_on(void)
// {
//     /*  XTAL = 40 MHz
//         Fout = XTAL · (4 + sdm2 + sdm1/256 + sdm0/65536) / [ 2 · (o_div+2) ]
//         Vogliamo Fout = 100 MHz  ⇒  scegliamo:
//         o_div = 0   (denominatore = 4)
//         sdm2 = 6    (4+sdm2 = 10  →  40·10 / 4 = 100 MHz esatti)
//         sdm1 = sdm0 = 0
//     */
//     rtc_clk_apll_enable(true);
//     rtc_clk_apll_coeff_set(0, 0, 0, 6);
// }

static inline uint32_t get_xtal_hz(void)
{
    return rtc_clk_xtal_freq_get() * 1000000UL;
}

void fm_apll_init(void)
{
    g_apll = fm_calc_apll(FM_CARRIER_HZ, MAX_DEV_HZ);

    rtc_clk_apll_enable(true);
    rtc_clk_apll_coeff_set(g_apll.o_div,
                           g_apll.base_sdm0,
                           g_apll.sdm1,
                           g_apll.sdm2);

    /* Audio a 8 bit unsigned → max ampiezza = 127 */
    g_apll.g_mod_shift = 7;                      // 2^7 = 128 ≈ 127
    g_apll.g_mod_mul = g_apll.dev_lsb; //* 3 / 4;         // dev_lsb * 1   (dividerai dopo con >>7)
                        
    ESP_LOGI("FM",
            "o_div=%u  sdm2=%u  sdm1=%u  base=%u  dev=%u",
            g_apll.o_div,
            g_apll.sdm2,
            g_apll.sdm1,
            g_apll.base_sdm0,
            g_apll.dev_lsb);
}

static fm_apll_cfg_t fm_calc_apll(uint32_t fout_hz, uint32_t dev_hz)
{
    uint32_t XTAL = get_xtal_hz();
    fm_apll_cfg_t c = {0};
    /* 1) trova il più piccolo o_div che rispetta il lock 350-500 MHz */
    while (c.o_div < 31) {
        if (fout_hz * 2 * (c.o_div + 2) >= 350000000UL) break;     // 350 MHz min lock
        ++c.o_div;
    }

    /* 2) fattore di moltiplica totale */
    double factor = (double)fout_hz * 2 * (c.o_div + 2) / XTAL;
    c.sdm2       = (uint8_t)factor - 4;
    double frac  = factor - (c.sdm2 + 4);
    c.sdm1       = (uint8_t)(frac * 256);
    c.base_sdm0  = (uint16_t)((frac * 256 - c.sdm1) * 65536);

    /* 3) quanti LSB valgono 1 Hz a questo o_div */
    double lsb_hz = XTAL / (2.0 * (c.o_div + 2) * 65536);
    c.dev_lsb    = (uint16_t)round(dev_hz / lsb_hz);           // → DEV_LSB automatico

    /* 4) se la frazione è troppo piccola, spostati al centro ±dev_lsb */
    if (c.base_sdm0 < c.dev_lsb)              // tipico con portanti 87-108 MHz
        c.base_sdm0 += c.dev_lsb;             //  → sdoppiamento simmetrico

    c.is_rev0   = (efuse_ll_get_chip_ver_rev1() == 0);
    return c;
}

static inline void fm_set_deviation(int16_t delta_lsb)
{
    int32_t sdm0 = (int32_t)g_apll.base_sdm0 + delta_lsb;
    if (sdm0 < 0)       sdm0 = 0;
    if (sdm0 > 65535)   sdm0 = 65535;

    clk_ll_apll_set_config(g_apll.is_rev0, g_apll.o_div, (uint16_t)sdm0, g_apll.sdm1, g_apll.sdm2);
}

void fm_route_to_pin(void)
{
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_GPIO0_U, FUNC_GPIO0_CLK_OUT1);   // abilita il mux IO-MUX
    REG_SET_FIELD(PIN_CTRL, CLK_OUT1, 0);                           // sorgente = I2S0 MCLK
    gpio_set_direction(GPIO_NUM_0, GPIO_MODE_OUTPUT);
}

void fm_i2s_init(void)
{
    const i2s_config_t cfg = {
        .mode                 = I2S_MODE_MASTER | I2S_MODE_TX,
        .sample_rate          = WAV_SR_HZ,
        .bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format       = I2S_CHANNEL_FMT_ONLY_RIGHT,
        .communication_format = I2S_COMM_FORMAT_STAND_PCM_SHORT,   // qualunque, non trasmetti dati (correspond to I2S_COMM_FORMAT_PCM)
        .use_apll             = true,
        .fixed_mclk           = FM_CARRIER_HZ,
        .dma_buf_count        = 4,
        .dma_buf_len          = 64,
        .intr_alloc_flags     = ESP_INTR_FLAG_LEVEL1
    };
    ESP_ERROR_CHECK(i2s_driver_install(I2S_NUM_0, &cfg, 0, NULL));
    ESP_ERROR_CHECK(i2s_start(I2S_NUM_0));
}

static void IRAM_ATTR fm_timer_cb(void *arg)
{
    static size_t pos;
    int16_t delta = (((rickroll[pos++] - 128) * g_apll.dev_lsb) >> 7);
    fm_set_deviation((int16_t)delta);
    if (pos >= rickroll_len) pos = 0;
}


// static void IRAM_ATTR fm_timer_cb(void *arg)
// {
//     static size_t  pos = 0;
//     uint8_t pcm8 = rickroll[pos];
//     if (++pos >= rickroll_len) pos = 0;

//     static int16_t y_prev = 0;
//     int16_t x = (int16_t)pcm8 - 128;
//     int16_t y = x + ((y_prev * 225) >> 8);
//     y_prev = y;

//     int16_t delta = (y * g_apll.dev_lsb) >> 7;
//     fm_set_deviation(delta);
// }

void fm_start_audio(void)
{
    const esp_timer_create_args_t t = {
        .callback = fm_timer_cb,
        .name = "fm_audio"
    };
    esp_timer_handle_t h;
    esp_timer_create(&t, &h);
    esp_timer_start_periodic(h, 1000000ULL / WAV_SR_HZ);
}