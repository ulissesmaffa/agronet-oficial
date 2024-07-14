#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "esp_timer.h"
#include "esp_system.h" //retorna o tempo de execucação do microprocessador
#include "driver/gpio.h"

#define PIN 2
#define TEMP_PIN 34


void disparo(TimerHandle_t xTimer){
    printf("Disparado em %lld \n", esp_timer_get_time()/1000);
    gpio_set_level(PIN,1);
}

void app_main(void){
    //LED INTERNO
    esp_rom_gpio_pad_select_gpio(PIN); 
    gpio_set_direction(PIN, GPIO_MODE_OUTPUT);

    // int status=0;

    printf("Sistema iniciado em %lld \n",esp_timer_get_time()/1000);
    TimerHandle_t xTimer = xTimerCreate("Timer disparo",pdMS_TO_TICKS(500),true,NULL,disparo);
    xTimerStart(xTimer,0);
}
