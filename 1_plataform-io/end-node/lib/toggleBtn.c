#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "spi_flash_mmap.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define PIN 2
#define BT_PIN 34

void app_main(void){
    //LED INTERNO
    esp_rom_gpio_pad_select_gpio(PIN); 
    gpio_set_direction(PIN, GPIO_MODE_OUTPUT);

    //BUTTON
    esp_rom_gpio_pad_select_gpio(BT_PIN); 
    gpio_set_direction(BT_PIN, GPIO_MODE_INPUT);
    bool status = 0;

    while (1){
        status = gpio_get_level(BT_PIN);
        gpio_set_level(PIN,status);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

