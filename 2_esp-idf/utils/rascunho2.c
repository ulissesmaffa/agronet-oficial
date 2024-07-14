#include <stdio.h>
#include "esp_log.h"
#include "esp_types.h"
#include "esp_intr_alloc.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// #include "driver/timer.h"
#include "soc/timer_group_struct.h"
#include "driver/gpio.h"
// #include "driver/pcnt.h"
#include "driver/pulse_cnt.h"

#define LED_PIN       2 // LED interno da ESP32
#define TIMER_DIVIDER 80 // 80/80 = 1MHz -> incremento a cada 1ms
#define TIMER_SCALE (80000000/TIMER_DIVIDER)

#define INPUT_SIGNAL 35
#define PCNT_THRESH 1000     // Limite para contagem de bordas ascendentes
#define BT_PIN 34

#define PCNT_HIGH_LIMIT 1000
#define PCNT_LOW_LIMIT  0

// Variáveis globais
volatile bool edge_detected = false;
volatile bool turn_on_edge = false;
volatile bool intr_1 = false;
volatile int  counter_intr_1 = 0;

volatile bool intr_2 = false;
volatile int  counter_intr_2 = 0;

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

/*
Iniciar contagem de bordas
*/
// void start_pcnt(){
//     //REFAZER
//     pcnt_intr_enable(PCNT_UNIT_0);    // habilita interrupção
//     pcnt_counter_clear(PCNT_UNIT_0);  // Limpa o contador se necessário
//     pcnt_counter_resume(PCNT_UNIT_0); // Retoma a contagem
// }

/*
INTERRUPÇÃO DE BORDA DE SUBIDA 1
*/
void IRAM_ATTR edge_detect_isr(void *arg) {
    if(turn_on_edge){
        // start_pcnt();
        turn_on_edge = false;
        counter_intr_1++;
        intr_1 = true;
        gpio_set_level(LED_PIN, 1); // Indicativo de fim da medição
    }
}

/*
INTERRUPÇÃO DE BORDA DE SUBIDA 2
*/
// void IRAM_ATTR pcnt_intr_handler(void *arg) {
//     // pcnt_unit_stop(PCNT_UNIT_0); 
//     counter_intr_2++;
//     intr_2 = true;
//     gpio_set_level(LED_PIN, 0); // Indicativo de fim da medição
// }


void config_edge_detect() {
    gpio_set_direction(INPUT_SIGNAL, GPIO_MODE_INPUT); //definir pino como entrada
    gpio_set_intr_type(INPUT_SIGNAL, GPIO_INTR_POSEDGE); //detectar bordas de subida
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3); //prioridade alta para interrupção
    gpio_isr_handler_add(INPUT_SIGNAL, edge_detect_isr, NULL); //identifica borda de subida
}

void config_pcnt() {
    ESP_LOGI("CONFIG_PCNT", "install pcnt unit");
    pcnt_unit_config_t unit_config = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit = PCNT_LOW_LIMIT,
    };
    pcnt_unit_handle_t pcnt_unit = NULL; 
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


    // // Configuração inicial do PCNT
    // pcnt_config_t config = {
    //     .pulse_gpio_num = INPUT_SIGNAL, // Pino de entrada para os pulsos
    //     .channel = PCNT_CHANNEL_0,      // Canal utilizado
    //     .unit = PCNT_UNIT_0,            // Unidade do PCNT
    //     .pos_mode = PCNT_COUNT_INC,     // Incrementa na borda de subida
    //     .neg_mode = PCNT_COUNT_DIS,     // Não conta na borda de descida
    //     .counter_h_lim = PCNT_THRESH,   // Define o limite superior para o contador
    // };
    // pcnt_unit_config(&config); // Aplicar a configuração ao PCNT

    // pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_H_LIM); // Habilitar o evento de limite alto e configurar o manipulador de interrupção
    // pcnt_isr_register(pcnt_intr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL); // Registrar a função que será chamada quando uma interrupção do PCNT ocorrer
    // pcnt_intr_enable(PCNT_UNIT_0); // Habilitar interrupções do PCNT
    // pcnt_counter_pause(PCNT_UNIT_0); // Pausa contador
    // pcnt_counter_clear(PCNT_UNIT_0); // Limpa contador
}

void app_main(void) {
    int counter_btn=0;
    bool btn_status=false;
    bool last_btn_status=false;

    config_led();
    config_btn();
    config_edge_detect();
    config_pcnt();

    while(1){
        // Verifica se o botão foi solto (transição de pressionado para não pressionado)
        btn_status = gpio_get_level(BT_PIN); // Lê o estado atual do botão
        if(last_btn_status && !btn_status) {
            counter_btn++;
            ESP_LOGI("MAIN", "Botão pressionado [%i]",counter_btn);
            turn_on_edge=true;
        }
        last_btn_status = btn_status; // Atualiza o último estado do botão para o próximo ciclo

        if(intr_1){
            intr_1 = false;
            ESP_LOGI("MAIN", "Interrupção 1 ocorreu [%i]",counter_intr_1);
        }

        if(intr_2){
            intr_2 = false;
            ESP_LOGI("MAIN", "Interrupção 2 ocorreu [%i]",counter_intr_2);
        }
        
        vTaskDelay(10 / portTICK_PERIOD_MS); // Delay para debounce simples
    }


}