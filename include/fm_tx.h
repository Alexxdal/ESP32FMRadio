#ifndef FM_TX_H
#define FM_TX_H

typedef struct {
    uint8_t  o_div, sdm1, sdm2;
    uint16_t base_sdm0;         // “centro” da cui si parte
    uint16_t dev_lsb;           // quanti LSB servono per ±MAX_DEV_HZ
    uint16_t g_mod_mul;         // moltiplicatore per la modulazione
    uint8_t  g_mod_shift;       // = 0 se usi divisione intera
    bool     is_rev0;
} fm_apll_cfg_t;

void fm_i2s_init(void);
void fm_apll_init(void);
void fm_route_to_pin(void);
void fm_start_audio(void);

#endif // FM_TX_H