#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "spi_flash_mmap.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_log.h"

void app_main(void){
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11);

    while (1){
        int val=0;
        val=adc1_get_raw(ADC1_CHANNEL_4);
        printf("ADC1: %d\n",val);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

