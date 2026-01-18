#ifndef FM_TX_H
#define FM_TX_H

typedef struct {
    uint8_t  o_div, sdm2;      // integer part
    uint16_t base_frac16;      // 16-bit fractional (sdm1:sdm0)
    uint16_t dev_frac16;       // ±deviation in the same units
    bool     is_rev0;
} fm_apll_cfg_t;

void fm_i2s_init(void);
void fm_apll_init(void);
void fm_route_to_pin(void);
void fm_start_audio(void);
void fm_start_audio_from_file(const char *filename);

#endif // FM_TX_H