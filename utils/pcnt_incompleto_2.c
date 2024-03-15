#include <stdio.h>
#include "esp_log.h"
#include "esp_types.h"
#include "esp_intr_alloc.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "soc/timer_group_struct.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"

#define LED_PIN       2 // LED interno da ESP32
#define INPUT_SIGNAL 35
#define BT_PIN 34
#define PCNT_HIGH_LIMIT 100
#define PCNT_LOW_LIMIT  -100

// Variáveis globais
static pcnt_unit_handle_t pcnt_unit = NULL;
volatile int event_flag=false;

/*
Configuração de LED da ESP32
*/
void config_led(){
    esp_rom_gpio_pad_select_gpio(LED_PIN); 
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
}

/*
Configuração de BTN
*/
void config_btn(){
    esp_rom_gpio_pad_select_gpio(BT_PIN); 
    gpio_set_direction(BT_PIN, GPIO_MODE_INPUT);
}


void pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx){
    event_flag=true;
}

/*
Configuração de PCNT
*/
void config_pcnt() {
    ESP_LOGI("CONFIG_PCNT", "install pcnt unit");
    pcnt_unit_config_t unit_config = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit = PCNT_LOW_LIMIT,
    };
    // pcnt_unit_handle_t pcnt_unit = NULL; 
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    ESP_LOGI("CONFIG_PCNT", "set glitch filter");
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    ESP_LOGI("CONFIG_PCNT", "install pcnt channels");
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = INPUT_SIGNAL,
        .level_gpio_num = -1, //funciona como clk, um sinal de controle
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));

    ESP_LOGI("CONFIG_PCNT", "set edge and level actions for pcnt channels");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_LEVEL_ACTION_HOLD));

    ESP_LOGI("CONFIG_PCNT", "add watch points and register callbacks");
    // ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, 2));
    pcnt_event_callbacks_t cbs = {
        .on_reach = pcnt_on_reach,
    };
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, NULL));

    ESP_LOGI("CONFIG_PCNT", "enable pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_LOGI("CONFIG_PCNT", "clear pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_LOGI("CONFIG_PCNT", "start pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

}

void app_main(void) {
    config_led();
    config_btn();
    config_pcnt();

    while(1){
        if(event_flag){
            ESP_LOGI("MAIN", "event flag");
            event_flag=false;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}