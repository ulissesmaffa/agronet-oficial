#include <stdio.h>
#include "esp_log.h"
#include "esp_types.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/timer.h"
#include "soc/timer_group_struct.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"

#define LED_PIN       2  // LED interno da ESP32
#define TIMER_DIVIDER 80 // 80/80 = 1MHz -> incremento a cada 1ms
#define TIMER_SCALE (80000000/TIMER_DIVIDER)

#define PCNT_INPUT_SIG_IO 35 // GPIO para o sinal de entrada
#define BTN_PIN 34 
#define PCNT_THRESH 20000 // Limite para contagem de bordas ascendentes

// Variáveis globais
volatile int counter_index=0;
typedef struct {
    int i;
    uint64_t timer_counter_value;
    int16_t counter_edges;
} timer_event_t;

QueueHandle_t timer_queue;

/*
Configuração de LED INTERNO
*/
void config_led(){
    esp_rom_gpio_pad_select_gpio(LED_PIN); 
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
}

/*
Configuração de BTN
*/
void config_btn(){
    esp_rom_gpio_pad_select_gpio(BTN_PIN); 
    gpio_set_direction(BTN_PIN, GPIO_MODE_INPUT);
}

/*
Resetar o contador de bordas ascendetes
*/
void reset_pcnt(){
    pcnt_counter_pause(PCNT_UNIT_0); // Pausa contador
    pcnt_counter_clear(PCNT_UNIT_0); // Limpa contador
}

/*
Inicio de timer
*/
void start_timer(int timer_idx){
    ESP_LOGI("MAIN", "Vou iniciar o timer");
    gpio_set_level(LED_PIN,1);
    timer_start(TIMER_GROUP_0, timer_idx);
}

/*
Resetar o timer
*/
void reset_timer(int timer_idx){
    timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, timer_idx);// Limpar a bandeira de interrupção do timer usando a API do driver
    timer_pause(TIMER_GROUP_0, timer_idx); // Parar o timer
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);// Redefinir o valor do contador do timer para zero    
    timer_set_alarm_value(TIMER_GROUP_0, timer_idx, 3000000);// Configurar o valor do alarme (3 segundos neste caso)
    timer_set_alarm(TIMER_GROUP_0, timer_idx, TIMER_ALARM_EN);// Habilitar o alarme do timer
}

/*
INTERRUPÇÃO DE TIMER
*/
void IRAM_ATTR timer_group0_isr(void *para) {
    timer_event_t evt;
    pcnt_get_counter_value(PCNT_UNIT_0, &evt.counter_edges);
    int timer_idx = (int) para;
    evt.i=counter_index++;
    evt.timer_counter_value = timer_group_get_counter_value_in_isr(TIMER_GROUP_0,timer_idx);
    xQueueSendFromISR(timer_queue, &evt, NULL); // Coloca evento na fila
    gpio_set_level(LED_PIN, 0); // Desligar o LED
    reset_pcnt(); // reseta pcnt
    reset_timer(timer_idx); // Reconfigurar o timer para o próximo uso
}

/*
Configuração de timer
*/
void config_timer(int timer_idx){
    timer_config_t config = {
        .divider = TIMER_DIVIDER, //define o clk do timer
        .counter_dir = TIMER_COUNT_UP, //conta para cima
        .counter_en = TIMER_PAUSE, //inicia o timer parado
        .alarm_en = TIMER_ALARM_EN, //habilita alarme no timer
        .auto_reload = TIMER_AUTORELOAD_DIS, //atinge o alarme e recomeça a contagem
    };
    timer_init(TIMER_GROUP_0, timer_idx, &config);//inicializa timer
    
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL); //timer inicia em zero
    timer_set_alarm_value(TIMER_GROUP_0, timer_idx, 3000000);//depende do clk do timer
    timer_enable_intr(TIMER_GROUP_0, timer_idx); //habilita interrupções
    timer_isr_register(TIMER_GROUP_0, timer_idx, timer_group0_isr, (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);
}

/*
Task para verificar fila de interrupção de timer
*/
void timer_evt_task(void *arg){
    timer_event_t evt;
    while(1){
        if(xQueueReceive(timer_queue, &evt, portMAX_DELAY)){
            // Calc frequência
            double freq = ((double)evt.counter_edges/(evt.timer_counter_value/TIMER_SCALE)); // Convertendo para segundos
            printf("Event timer [%i]\n",evt.i);
            printf("Time:  %.5f s\n",(double) evt.timer_counter_value / TIMER_SCALE);
            printf("Edges: %.2f edges\n",(double) evt.counter_edges);
            printf("Freq:  %.2f Hz\n",freq);
        }
    }
}

/*
Configuração de PCNT
*/
void config_pcnt(){
    pcnt_config_t config = {
        .pulse_gpio_num = PCNT_INPUT_SIG_IO,
        // .ctrl_gpio_num = PCNT_INPUT_CTRL_IO, // caso use controle para detecção de borda
        .channel = PCNT_CHANNEL_0,
        .unit = PCNT_UNIT_0,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DIS,
        .counter_h_lim = PCNT_THRESH,
    };
    pcnt_unit_config(&config); // Inicializar o PCNT
}

void app_main(void) {
    //Variáveis usadas pelo BTN
    int counter_btn=0;
    bool btn_status=false;
    bool last_btn_status=false;

    timer_queue = xQueueCreate(10, sizeof(timer_event_t));
    config_led();
    config_btn();
    config_pcnt();
    reset_pcnt();
    config_timer(TIMER_0);
    xTaskCreate(timer_evt_task, "timer_evt_task", 2048, NULL, 5, NULL);

    while (1) {
        btn_status = gpio_get_level(BTN_PIN);
        if(last_btn_status && !btn_status) {
            counter_btn++;
            ESP_LOGI("MAIN", "Botão pressionado [%i]",counter_btn);
            start_timer(TIMER_0);
            pcnt_counter_resume(PCNT_UNIT_0); // Inicia contagem
        }
        last_btn_status = btn_status;
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}