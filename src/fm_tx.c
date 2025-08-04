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
#define MAX_DEV_HZ      75000UL        // ±75 kHz standard broadcast
#define WAV_SR_HZ       8000           // Sample Rate (8 kHz)

#if !CONFIG_IDF_TARGET_ESP32
    #error "Questo progetto richiede un chip con APLL (ESP32 D0WD / WROOM / WROVER). \
            Le serie S2/S3/C3 non la possiedono."
#endif

static fm_apll_cfg_t fm_calc_apll(uint32_t fout_hz, uint32_t dev_hz);

static fm_apll_cfg_t g_apll;

static inline uint32_t get_xtal_hz(void)
{
    return rtc_clk_xtal_freq_get() * 1000000UL;
}

void fm_apll_init(void)
{
    g_apll = fm_calc_apll(FM_CARRIER_HZ, MAX_DEV_HZ);

    uint8_t sdm0 = g_apll.base_frac16 & 0xFF;
    uint8_t sdm1 = g_apll.base_frac16 >> 8;
    rtc_clk_apll_enable(true);
    rtc_clk_apll_coeff_set(g_apll.o_div, sdm0, sdm1, g_apll.sdm2);

    ESP_LOGI("FM", "o_div=%u  sdm2=%u  frac=0x%04X  dev=%u LSB",
             g_apll.o_div, g_apll.sdm2, g_apll.base_frac16, g_apll.dev_frac16);
}

static fm_apll_cfg_t fm_calc_apll(uint32_t fout_hz, uint32_t dev_hz)
{
    uint32_t XTAL = get_xtal_hz();
    fm_apll_cfg_t c = {0};
    /* 1) choose o_div so VCO ≥350 MHz */
    while (c.o_div < 31) {
        if (fout_hz * 2 * (c.o_div + 2) >= 350000000UL) break;
        ++c.o_div;
    }
    /* 2) numerator  (4 + sdm2 + frac16/65536) */
    double mul   = (double)fout_hz * 2 * (c.o_div + 2) / XTAL;
    c.sdm2       = (uint8_t)mul - 4;           // integer part
    double frac  = mul - (c.sdm2 + 4);         // 0…<1
    uint32_t f16 = lround(frac * 65536.0);     // 0…65535

    if (f16 == 65536) {        // handle round-up overflow
        f16 = 0;
        ++c.sdm2;
    }
    c.base_frac16 = (uint16_t)f16;

    /* keep at least ±dev_frac16 margin */
    if (c.base_frac16 < c.dev_frac16)
        c.base_frac16 += c.dev_frac16;
    else if (c.base_frac16 > 65535 - c.dev_frac16)
        c.base_frac16 -= c.dev_frac16;

    /* 3) how many fraction-LSB for 1 Hz at this o_div */
    double lsb_hz = XTAL / (2.0 * (c.o_div + 2) * 65536);
    c.dev_frac16  = (uint16_t)lround(dev_hz / lsb_hz);

    c.is_rev0 = (efuse_ll_get_chip_ver_rev1() == 0);
    return c;
}

static inline void fm_set_deviation(int16_t delta_frac16)
{
    int32_t frac32 = (int32_t)g_apll.base_frac16 + delta_frac16;
    int32_t sdm2   = g_apll.sdm2;          // work copy

    /* borrow / carry across the 16-bit fraction */
    if (frac32 < 0) {
        int32_t borrow = (-frac32 + 65535) >> 16;  // how many 65536 steps
        frac32 += borrow * 65536;
        sdm2   -= borrow;
    } else if (frac32 > 65535) {
        int32_t carry = frac32 >> 16;              // how many 65536 steps
        frac32 -= carry * 65536;
        sdm2   += carry;
    }

    /* clamp sdm2 to valid 0…63 just in case */
    if (sdm2 < 0)      { sdm2 = 0;      frac32 = 0;      }
    if (sdm2 > 63)     { sdm2 = 63;     frac32 = 65535;  }

    uint8_t sdm0 = frac32 & 0xFF;
    uint8_t sdm1 = frac32 >> 8;

    clk_ll_apll_set_config(g_apll.is_rev0,
                           g_apll.o_div,
                           sdm0,
                           sdm1,
                           (uint8_t)sdm2);
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

static inline int16_t clip16(int32_t v){
    if (v >  2047) v = 2047 + ((v-2047)>>2);
    if (v < -2047) v = -2047 + ((v+2047)>>2);
    return (int16_t)v;
}

static void IRAM_ATTR fm_timer_cb(void *arg)
{
    static size_t  pos      = 0;
    /* 1. Leggi campione 8-bit e portalo a signed */
    int16_t audio = (int16_t)rickroll[pos++] - 128;   // –128 … +127
    if (pos >= rickroll_len) pos = 0;
    /* 3. Scala in frazione APLL e applica deviazione */
    int16_t delta = (audio * g_apll.dev_frac16) >> 7;      // scale
    fm_set_deviation(delta);
}

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