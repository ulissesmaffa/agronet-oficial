#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "spi_flash_mmap.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define PIN 2 //led que está nesse pino

void app_main(void){
    esp_rom_gpio_pad_select_gpio(PIN); //função para seleção do pino
    gpio_set_direction(PIN, GPIO_MODE_OUTPUT); //configurar o sentido do pino
    bool status = 0;

    //uso padrão dos logs
    ESP_LOGE("LOG1","LOG DE ERRO");
    ESP_LOGW("LOG1","LOG DE WARNING/AVISO");
    ESP_LOGI("LOG1","LOG DE INFORMAÇÃO");
    ESP_LOGD("LOG1","LOG DE DEBUG");
    ESP_LOGV("LOG1","LOG DE TEXTO COMUM/VERBOSE");

    //uso de logs com variáveis
    int var=0;
    ESP_LOGE("LOG2","LOG DE ERRO %i",var++);
    ESP_LOGW("LOG2","LOG DE WARNING/AVISO %i",var++);
    ESP_LOGI("LOG2","LOG DE INFORMAÇÃO %i",var++);
    ESP_LOGD("LOG2","LOG DE DEBUG %i",var++);
    ESP_LOGV("LOG2","LOG DE TEXTO COMUM/VERBOSE %i",var++);

    //limitando os logs por tags
    esp_log_level_set("LOG3",ESP_LOG_WARN); //o log2 só vai mostrar até o warning. O nível máximo de log é definido no menuconfig
    ESP_LOGW("LOG3","LOG DE WARNING/AVISO");
    ESP_LOGE("LOG3","LOG DE ERRO");
    ESP_LOGI("LOG3","LOG DE INFORMAÇÃO");
    ESP_LOGD("LOG3","LOG DE DEBUG");
    ESP_LOGV("LOG3","LOG DE TEXTO COMUM/VERBOSE");

    while (1)
    {
        status = !status;
        gpio_set_level(PIN,status);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
