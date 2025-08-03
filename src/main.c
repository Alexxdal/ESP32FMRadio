#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "fm_tx.h"

void app_main() 
{
    fm_i2s_init();
    fm_route_to_pin();
    fm_apll_init();

    fm_start_audio();

    while (true)
        vTaskDelay(pdMS_TO_TICKS(1000));
}