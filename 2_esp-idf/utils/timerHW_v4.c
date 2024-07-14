#include <stdio.h>
#include "esp_log.h"
#include "esp_types.h"
#include "esp_intr_alloc.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/timer.h"
#include "soc/timer_group_struct.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"

#define LED_PIN       2 // LED interno da ESP32
#define TIMER_DIVIDER 80 // 80/80 = 1MHz -> incremento a cada 1ms
#define TIMER_SCALE (80000000/TIMER_DIVIDER)

#define PCNT_INPUT_SIG_IO 4  // GPIO para o sinal de entrada
#define EDGE_DETECT_PIN 4
#define PCNT_THRESH 10     // Limite para contagem de bordas ascendentes

volatile int counter_index=0;
volatile bool timer_started = false; //para detectar a primeira subida de borda
typedef struct {
    int i;
    uint64_t timer_counter_value;
    int16_t counter_edges;
} timer_event_t;

QueueHandle_t timer_queue;

void config_led(){
    // Configuração do led interno
    esp_rom_gpio_pad_select_gpio(LED_PIN); 
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
}

static void reset_pcnt(){
    pcnt_counter_pause(PCNT_UNIT_0); // Pausa contador
    pcnt_counter_clear(PCNT_UNIT_0); // Limpa contador
}

//INTERRUPÇÃO BORDA DE SUBIDA
void IRAM_ATTR edge_detect_isr(void *arg) {
    if (!timer_started) {
        // Iniciar o timer aqui
        timer_start(TIMER_GROUP_0, TIMER_0);
        timer_started = true;
        gpio_set_level(LED_PIN, 1); // Indicativo de início da medição
    }
}

//INTERRUPÇÃO QUANDO CONTA PCNT_THRESH
void IRAM_ATTR pcnt_intr_handler(void *arg) {
    timer_pause(TIMER_GROUP_0, TIMER_0); // Parar o timer após 1000 bordas
    gpio_set_level(LED_PIN, 0); // Indicativo de fim da medição

    timer_event_t evt;
    timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &evt.timer_counter_value);
    pcnt_get_counter_value(PCNT_UNIT_0, &evt.counter_edges);
    evt.i=counter_index++;

    xQueueSendFromISR(timer_queue, &evt, NULL);

    // Reset para próxima contagem
    pcnt_counter_clear(PCNT_UNIT_0);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    timer_started = false; // Pronto para detectar a próxima primeira borda
}

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
}

void timer_evt_task(void *arg){
    timer_event_t evt;
    while(1){
        if(xQueueReceive(timer_queue, &evt, portMAX_DELAY)){
            // Calc frequência
            double freq = ((double)evt.counter_edges/(evt.timer_counter_value/TIMER_SCALE)); // Convertendo para segundos
            printf("[%i] event timer\n",evt.i);
            printf("Time:  %.8f s\n",(double) evt.timer_counter_value / TIMER_SCALE);
            printf("Edges: %.8f edges\n",(double) evt.counter_edges);
            printf("Freq:  %.8f Hz\n",freq);
        }
    }
}

void config_pcnt() {
    // Configuração inicial do PCNT
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = PCNT_INPUT_SIG_IO, // Pino de entrada para os pulsos
        .channel = PCNT_CHANNEL_0,           // Canal utilizado
        .unit = PCNT_UNIT_0,                 // Unidade do PCNT
        .pos_mode = PCNT_COUNT_INC,          // Incrementa na borda de subida
        .neg_mode = PCNT_COUNT_DIS,          // Não conta na borda de descida
        .counter_h_lim = PCNT_THRESH,        // Define o limite superior para o contador
    };
    // Aplicar a configuração ao PCNT
    pcnt_unit_config(&pcnt_config);
    // Habilitar o evento de limite alto e configurar o manipulador de interrupção
    pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_H_LIM);
    // Registrar a função que será chamada quando uma interrupção do PCNT ocorrer
    pcnt_isr_register(pcnt_intr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);
    // Habilitar interrupções do PCNT
    pcnt_intr_enable(PCNT_UNIT_0);
    // Opcional: Considerar a retomada da contagem após inicialização
    pcnt_counter_resume(PCNT_UNIT_0);
}

void config_edge_detect() {
    gpio_set_direction(EDGE_DETECT_PIN, GPIO_MODE_INPUT); //definir pino como entrada
    gpio_set_intr_type(EDGE_DETECT_PIN, GPIO_INTR_POSEDGE); //detectar bordas de subida
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3); //prioridade alta para interrupção
    gpio_isr_handler_add(EDGE_DETECT_PIN, edge_detect_isr, NULL); //identifica borda de subida
}


void app_main(void) {
    timer_queue = xQueueCreate(10, sizeof(timer_event_t));
    config_led();
    config_edge_detect();
    config_pcnt();
    reset_pcnt();
    config_timer(TIMER_0);
    xTaskCreate(timer_evt_task, "timer_evt_task", 2048, NULL, 5, NULL);
    // pcnt_counter_resume(PCNT_UNIT_0); // Inicia contagem


    // while (1) {
    //     pcnt_counter_resume(PCNT_UNIT_0); // Inicia contagem
    //     vTaskDelay(10000/portTICK_PERIOD_MS);
    // }
}