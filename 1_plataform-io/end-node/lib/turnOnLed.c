#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "spi_flash_mmap.h"
#include "driver/gpio.h"

// #include "sdkconfig.h"

#define PIN 2 //led que está nesse pino

void app_main(void){
    esp_rom_gpio_pad_select_gpio(PIN); //função para seleção do pino
    gpio_set_direction(PIN, GPIO_MODE_OUTPUT); //configurar o sentido do pino
    bool status = 0;

    while (1)
    {
        status = !status;
        gpio_set_level(PIN,status);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
