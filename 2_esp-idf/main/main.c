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
#define TIMER_ALARM 3500000

#define PCNT_INPUT_SIG_IO 35 // GPIO para o sinal de entrada
#define BTN_PIN 34 
#define PCNT_THRESH 20000 // Limite para contagem de bordas ascendentes


#define CONTROL_BTN_IN_MOSFET 33 //controle de MOSFET P sinal de entrada
#define CONTROL_BTN_OUT_MOSFET 26 //controle de MOSFET P sinal de saída

// #define CONTROL_BTN_IN_RESET 32 
// #define CONTROL_BTN_OUT_RESET  

typedef struct {
    int i;
    uint64_t timer_counter_value;
    int16_t counter_edges;
} timer_event_t;

// Variáveis globais
volatile int counter_index=0;
timer_event_t evt;
QueueHandle_t timer_queue;

/*
Configuração de LED INTERNO
*/
void config_led(){
    ESP_LOGI("CONFIG_LED", "Configurando led   | LED_PIN = %i",LED_PIN);
    esp_rom_gpio_pad_select_gpio(LED_PIN); 
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
}

/* Configuração de BTN */
void config_btn(){
    ESP_LOGI("CONFIG_BTN", "Configurando botão | BTN_PIN = %i",BTN_PIN);
    esp_rom_gpio_pad_select_gpio(BTN_PIN); 
    gpio_set_direction(BTN_PIN, GPIO_MODE_INPUT);
}

/* Configuração Controle MOSFET (IN/OUT) */
void config_control_mosfet(){
    ESP_LOGI("CONFIG_CONTROL_MOSFET", "Configurando controle mosfet in| CONTROL_BTN_IN_MOSFET = %i",CONTROL_BTN_IN_MOSFET);
    esp_rom_gpio_pad_select_gpio(CONTROL_BTN_IN_MOSFET); 
    gpio_set_direction(CONTROL_BTN_IN_MOSFET, GPIO_MODE_INPUT);

    ESP_LOGI("CONFIG_CONTROL_MOSFET", "Configurando controle mosfet out| CONTROL_BTN_OUT_MOSFET = %i",CONTROL_BTN_OUT_MOSFET);
    esp_rom_gpio_pad_select_gpio(CONTROL_BTN_OUT_MOSFET); 
    gpio_set_direction(CONTROL_BTN_OUT_MOSFET, GPIO_MODE_OUTPUT);
}

/* Resetar o contador de bordas ascendetes */
void reset_pcnt(){
    pcnt_counter_pause(PCNT_UNIT_0); // Pausa contador
    pcnt_counter_clear(PCNT_UNIT_0); // Limpa contador
}

/* Resetar o timer */
void reset_timer(int timer_idx){
    timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, timer_idx);// Limpar a bandeira de interrupção do timer usando a API do driver
    timer_pause(TIMER_GROUP_0, timer_idx); // Parar o timer
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);// Redefinir o valor do contador do timer para zero    
    timer_set_alarm_value(TIMER_GROUP_0, timer_idx, TIMER_ALARM);// Configurar o valor do alarme (3 segundos neste caso)
    timer_set_alarm(TIMER_GROUP_0, timer_idx, TIMER_ALARM_EN);// Habilitar o alarme do timer
}

/* INTERRUPÇÃO DE TIMER */
void IRAM_ATTR timer_group0_isr(void *para) {
    // timer_event_t evt;
    pcnt_get_counter_value(PCNT_UNIT_0, &evt.counter_edges);
    int timer_idx = (int) para;
    evt.i=counter_index++;
    evt.timer_counter_value = timer_group_get_counter_value_in_isr(TIMER_GROUP_0,timer_idx);
    xQueueSendFromISR(timer_queue, &evt, NULL); // Coloca evento na fila
    gpio_set_level(LED_PIN, 0); // Desligar o LED
    reset_pcnt(); // reseta pcnt
    reset_timer(timer_idx); // Reconfigurar o timer para o próximo uso
}

/* Configuração de timer */
void config_timer(int timer_idx){
    ESP_LOGI("CONFIG_TIMER", "Configurando timer para %.2f segundos",(double)(TIMER_ALARM/TIMER_SCALE));
    timer_config_t config = {
        .divider = TIMER_DIVIDER, //define o clk do timer
        .counter_dir = TIMER_COUNT_UP, //conta para cima
        .counter_en = TIMER_PAUSE, //inicia o timer parado
        .alarm_en = TIMER_ALARM_EN, //habilita alarme no timer
        .auto_reload = TIMER_AUTORELOAD_DIS, //atinge o alarme e recomeça a contagem
    };
    timer_init(TIMER_GROUP_0, timer_idx, &config);//inicializa timer
    
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL); //timer inicia em zero
    timer_set_alarm_value(TIMER_GROUP_0, timer_idx, TIMER_ALARM);//depende do clk do timer
    timer_enable_intr(TIMER_GROUP_0, timer_idx); //habilita interrupções
    timer_isr_register(TIMER_GROUP_0, timer_idx, timer_group0_isr, (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);
}

//Essas esquações devem vir de arquivo
double calc_temp(double freq){
    double temp;

    if(freq < 115){//Dados 1 - RMSE=1.15
        temp =  freq*0.30-19.9; 
    }else if(freq >= 115 && freq <= 180){//Dados 2 - RMSE=0.30
        temp = freq*0.10+0.55; 
    }else if(freq >180 && freq <=260){//Dados 3 - RMSE=0.29
        temp = freq*0.09+4.231; 
    }else if(freq >260 && freq <=440){//Dados 4 - RMSE=0.28
        temp = freq*0.07+10.28;  
    }else if(freq >440 && freq <=655){//Dados 5 - RMSE=0.14
        temp = freq*0.04+20.21;  
    }else{//Dados 6 - RMSE=0.34
        temp =  freq*0.03+26.33;
    }

    return temp;
}

/* Task para verificar fila de interrupção de timer */
void timer_evt_task(void *arg){
    timer_event_t evt;
    while(1){
        if(xQueueReceive(timer_queue, &evt, portMAX_DELAY)){
            double freq = ((double)evt.counter_edges * (double)TIMER_SCALE) / (double)evt.timer_counter_value;
            double temp = calc_temp(freq);
            printf("Event timer [%i]\n",evt.i);
            printf("Time:  %.5f s\n",(double) evt.timer_counter_value / TIMER_SCALE);
            printf("Edges: %.2f edges\n",(double) evt.counter_edges);
            printf("Freq:  %.2f Hz\n",freq);
            printf("Temp:  %.2f ºC\n",temp);
        }
    }
}

/* Configuração de PCNT */
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
    /* Configure and enable the input filter */
    pcnt_set_filter_value(PCNT_UNIT_0, 100);
    pcnt_filter_enable(PCNT_UNIT_0);
}

void temp_task(void *arg){
    //Variáveis usadas pelo BTN
    int counter_btn=0;
    bool btn_status=false;
    bool last_btn_status=false;

    bool ctr_mosfet_in_status=false;

    while(1){
        btn_status = gpio_get_level(BTN_PIN);
        if(last_btn_status && !btn_status) {
            counter_btn++;
            ESP_LOGI("MAIN", "[%i] Coletando dados...",counter_btn);
            timer_start(TIMER_GROUP_0, TIMER_0); // Inicia timer
            pcnt_counter_resume(PCNT_UNIT_0);    // Inicia contagem
            gpio_set_level(LED_PIN,1);
        }
        last_btn_status = btn_status;
        vTaskDelay(10/portTICK_PERIOD_MS);

        ctr_mosfet_in_status = gpio_get_level(CONTROL_BTN_IN_MOSFET);
        if (ctr_mosfet_in_status){
            gpio_set_level(LED_PIN,1);
            gpio_set_level(CONTROL_BTN_OUT_MOSFET,1);
        }else{
            gpio_set_level(LED_PIN,0);
            gpio_set_level(CONTROL_BTN_OUT_MOSFET,0);
        }

    }
}

void app_main(void) {
    config_led();
    config_btn();

    config_control_mosfet();

    config_pcnt();
    reset_pcnt();
    config_timer(TIMER_0);

    timer_queue = xQueueCreate(10, sizeof(timer_event_t));
    xTaskCreate(timer_evt_task, "timer_evt_task", 2048, NULL, 5, NULL);
    xTaskCreate(&temp_task, "temp_task", 2048, NULL, 5, NULL);
}

/*
Links úteis:
https://docs.espressif.com/projects/esp-idf/en/v5.0.2/esp32/api-reference/peripherals/pcnt.html#_CPPv434pcnt_unit_register_event_callbacks18pcnt_unit_handle_tPK22pcnt_event_callbacks_tPv
https://docs.espressif.com/projects/esp-idf/en/v5.0.2/esp32/migration-guides/release-5.x/peripherals.html
https://github.com/espressif/esp-idf/blob/v5.0.2/examples/peripherals/pcnt/rotary_encoder/main/rotary_encoder_example_main.c#L60
https://docs.espressif.com/projects/esp-idf/en/v3.3.2/api-reference/peripherals/pcnt.html#_CPPv418pcnt_filter_enable11pcnt_unit_t
https://github.com/espressif/esp-idf/blob/v3.3.2/examples/peripherals/pcnt/main/pcnt_example_main.c
*/