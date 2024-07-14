#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define LED_PIN       2 // LED interno da ESP32
#define TIMER_DIVIDER 80 // 80/80 = 1MHz -> incremento a cada 1ms

// Variaveis globais
volatile bool timer_interrupt_flag = false;

void config_led(){
    // Configuração do led interno
    esp_rom_gpio_pad_select_gpio(LED_PIN); 
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
}

void reset_timer(int timer_idx){
    timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, timer_idx);// Limpar a bandeira de interrupção do timer usando a API do driver
    timer_pause(TIMER_GROUP_0, timer_idx); // Parar o timer
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);// Redefinir o valor do contador do timer para zero    
    timer_set_alarm_value(TIMER_GROUP_0, timer_idx, 3000000);// Configurar o valor do alarme (3 segundos neste caso)
    timer_set_alarm(TIMER_GROUP_0, timer_idx, TIMER_ALARM_EN);// Habilitar o alarme do timer
}

void IRAM_ATTR timer_group0_isr(void *para) {
    int timer_idx = (int) para;
    gpio_set_level(LED_PIN, 0);// Desligar o LED
    reset_timer(timer_idx);// Reconfigurar o timer para o próximo uso
}

void config_timer(int timer_idx){
    timer_config_t config = {
        .divider = TIMER_DIVIDER, //define o clk do timer
        .counter_dir = TIMER_COUNT_UP, //conta para cima
        .counter_en = TIMER_PAUSE, //inicia o timer parado
        .alarm_en = TIMER_ALARM_EN, //habilita alarme no timer
        .auto_reload = TIMER_AUTORELOAD_DIS, //atinge o alarme e permanece parado, não recomeça a contagem
    };
    timer_init(TIMER_GROUP_0, timer_idx, &config);//inicializa timer
    
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL); //timer inicia em zero
    timer_set_alarm_value(TIMER_GROUP_0, timer_idx, 3000000);//depende do clk do timer
    timer_enable_intr(TIMER_GROUP_0, timer_idx); //habilita interrupções
    timer_isr_register(TIMER_GROUP_0, timer_idx, timer_group0_isr, (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);
}

void start_timer(int timer_idx){
    ESP_LOGI("MAIN", "Vou iniciar o timer");
    timer_start(TIMER_GROUP_0, timer_idx);
    gpio_set_level(LED_PIN,1);
}

void app_main(void) {
    config_led();
    config_timer(TIMER_0);

    while (1) {
        start_timer(TIMER_0);
        vTaskDelay(10000/portTICK_PERIOD_MS);
    }
}