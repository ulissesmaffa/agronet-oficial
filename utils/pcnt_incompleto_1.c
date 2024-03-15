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

#define PCNT_INPUT_SIG_IO 35  // GPIO para o sinal de entrada
#define EDGE_DETECT_PIN 35
#define PCNT_THRESH 1000     // Limite para contagem de bordas ascendentes

#define BT_PIN 34

volatile int counter_index=0;
volatile bool timer_started = false; //para detectar a primeira subida de borda

//Usar para teste
volatile bool timer_interrupt_flag = false;
volatile bool timer_interrupt_flag_1 = false;
volatile bool timer_interrupt_flag_2 = false;
volatile bool turn_on_edge = false;
volatile int control_index=0;
volatile int interrupcao_1=0;
volatile int interrupcao_2=0;


typedef struct {
    int i;
    uint64_t timer_counter_value;
    int16_t counter_edges;
} timer_event_t;

// QueueHandle_t timer_queue;


// void reset_pcnt(){
//     pcnt_counter_pause(PCNT_UNIT_0); // Pausa contador
//     pcnt_counter_clear(PCNT_UNIT_0); // Limpa contador
// }

// void config_timer(int timer_idx){
//     timer_config_t config = {
//         .divider = TIMER_DIVIDER, //define o clk do timer
//         .counter_dir = TIMER_COUNT_UP, //conta para cima
//         .counter_en = TIMER_PAUSE, //inicia o timer parado
//         .alarm_en = TIMER_ALARM_EN, //habilita alarme no timer
//         .auto_reload = TIMER_AUTORELOAD_DIS, //atinge o alarme e recomeça a contagem
//     };
//     timer_init(TIMER_GROUP_0, timer_idx, &config);//inicializa timer
//     timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL); //timer inicia em zero
// }

// void timer_evt_task(void *arg){
//     timer_event_t evt;
//     while(1){
//         if(xQueueReceive(timer_queue, &evt, portMAX_DELAY)){
//             // Calc frequência
//             double freq = ((double)evt.counter_edges/(evt.timer_counter_value/TIMER_SCALE)); // Convertendo para segundos
//             printf("[%i] event timer\n",evt.i);
//             printf("Time:  %.8f s\n",(double) evt.timer_counter_value / TIMER_SCALE);
//             printf("Edges: %.8f edges\n",(double) evt.counter_edges);
//             printf("Freq:  %.8f Hz\n",freq);
//         }
//     }
// }

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
void start_pcnt(){
    pcnt_intr_enable(PCNT_UNIT_0);    // habilita interrupção
    pcnt_counter_clear(PCNT_UNIT_0);  // Limpa o contador se necessário
    pcnt_counter_resume(PCNT_UNIT_0); // Retoma a contagem
}

/*
INTERRUPÇÃO DE BORDA DE SUBIDA 1
Deve identificar a primeira borda de subida, a partir disso:
a) Iniciar o timer para contar o tempo;
b) Iniciar o contador de bordas (limite definido por PCNT_THRESH)
*/
void IRAM_ATTR edge_detect_isr(void *arg) {
    if(turn_on_edge){ //quando turn_on_edge = true, inicia interrupção
        if (!timer_started) {
            // Iniciar o timer aqui
            // timer_start(TIMER_GROUP_0, TIMER_0);
            
            // Inicia contagem de borda aqui
            start_pcnt();

            timer_started = true; //bloqueia esse trecho de código
            turn_on_edge = false; //bloqueira essa interrupção

            control_index++; //incrementa control_index para controle no monitor
            interrupcao_1++; //incrementa interrupcao_1 para controle no monitor
            timer_interrupt_flag_1 = true; //flag que sinaliza interrupção para controle no monitor

            gpio_set_level(LED_PIN, 1); // Indicativo de início da medição
        }
    }
}


/*
INTERRUPÇÃO DE BORDA DE SUBIDA 2
Deve identificar o valor limite da contagem de borda, a partir disso:
a) pausar o timer;
b) enviar para fila valor do timer
c) pausar a contagem de bordas de subida.
*/
void IRAM_ATTR pcnt_intr_handler(void *arg) {
    // Pausa o timer aqui
    // timer_pause(TIMER_GROUP_0, TIMER_0);

    // Envia dados para fila
    // timer_event_t evt; //cria estrutura de dados
    // timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &evt.timer_counter_value); //obtem valor do timer
    // pcnt_get_counter_value(PCNT_UNIT_0, &evt.counter_edges); //não é necessário, mas envia quantide de bordas contadas, é esperado que seja: PCNT_THRESH
    // evt.i=counter_index++; //envia controle de contagem de interrupção
    // xQueueSendFromISR(timer_queue, &evt, NULL);

    // Reset PCNT
    pcnt_counter_clear(PCNT_UNIT_0); // Limpa o contador
    pcnt_intr_disable(PCNT_UNIT_0);  // Desabilita interrupções do PCNT para evitar disparos contínuos
    pcnt_counter_pause(PCNT_UNIT_0); // Pausa o contador
    gpio_intr_disable(EDGE_DETECT_PIN);

    // Reset timer
    // timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    // timer_started = false; // Pronto para detectar a próxima primeira borda

    control_index++; //incrementa control_index para controle no monitor
    interrupcao_2++; //incrementa interrupcao_2 para controle no monitor
    timer_interrupt_flag_2 = true;
    // timer_started = false; 

    gpio_set_level(LED_PIN, 0); // Indicativo de fim da medição
}

void config_edge_detect() {
    gpio_set_direction(EDGE_DETECT_PIN, GPIO_MODE_INPUT); //definir pino como entrada
    gpio_set_intr_type(EDGE_DETECT_PIN, GPIO_INTR_POSEDGE); //detectar bordas de subida
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3); //prioridade alta para interrupção
    gpio_isr_handler_add(EDGE_DETECT_PIN, edge_detect_isr, NULL); //identifica borda de subida
}

void config_pcnt() {
    // Configuração inicial do PCNT
    pcnt_config_t config = {
        .pulse_gpio_num = PCNT_INPUT_SIG_IO, // Pino de entrada para os pulsos
        .channel = PCNT_CHANNEL_0,           // Canal utilizado
        .unit = PCNT_UNIT_0,                 // Unidade do PCNT
        .pos_mode = PCNT_COUNT_INC,          // Incrementa na borda de subida
        .neg_mode = PCNT_COUNT_DIS,          // Não conta na borda de descida
        .counter_h_lim = PCNT_THRESH,        // Define o limite superior para o contador
    };
    pcnt_unit_config(&config); // Aplicar a configuração ao PCNT
    pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_H_LIM); // Habilitar o evento de limite alto e configurar o manipulador de interrupção
    pcnt_isr_register(pcnt_intr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL); // Registrar a função que será chamada quando uma interrupção do PCNT ocorrer
    pcnt_intr_enable(PCNT_UNIT_0); // Habilitar interrupções do PCNT
    pcnt_counter_pause(PCNT_UNIT_0); // Pausa contador
    pcnt_counter_clear(PCNT_UNIT_0); // Limpa contador
}

void app_main(void) {
    // timer_queue = xQueueCreate(10, sizeof(timer_event_t));
    config_led();
    config_btn();
    config_edge_detect();
    config_pcnt();
    // reset_pcnt();
    // config_timer(TIMER_0);
    // xTaskCreate(timer_evt_task, "timer_evt_task", 2048, NULL, 5, NULL);
    
    bool btn_status = false;
    bool last_btn_status = false; // Armazena o último estado do botão
    int counter_btn = 0;

    while(1){
        // Verifica se o botão foi solto (transição de pressionado para não pressionado)
        btn_status = gpio_get_level(BT_PIN); // Lê o estado atual do botão
        if(last_btn_status && !btn_status) {
            ESP_LOGI("MAIN", "Botão pressionado [%i]",counter_btn);
            counter_btn++;
            turn_on_edge = true;
            gpio_intr_enable(EDGE_DETECT_PIN);
        }
        last_btn_status = btn_status; // Atualiza o último estado do botão para o próximo ciclo
        
        // // Informa interrupção
        // if (timer_interrupt_flag) {
        //     timer_interrupt_flag = false;
        //     ESP_LOGI("MAIN", "A interrupção do PCNT ocorreu [%i] 1[%i] 2[%i]", control_index, interrupcao_1, interrupcao_2);
        // }

        //Interrupção detecta primeira borda:
        if(timer_interrupt_flag_1){
            timer_interrupt_flag_1 = false;
            ESP_LOGI("MAIN", "A interrupção_1 do PCNT ocorreu [%i] 1[%i] 2[%i]", control_index, interrupcao_1, interrupcao_2);
        }
        //Interrupção de contagem de 1000
        if(timer_interrupt_flag_2){
            timer_interrupt_flag_2 = false;
            ESP_LOGI("MAIN", "A interrupção_2 do PCNT ocorreu [%i] 1[%i] 2[%i]", control_index, interrupcao_1, interrupcao_2);
            timer_started = false; 
        }


        vTaskDelay(10 / portTICK_PERIOD_MS); // Delay para debounce simples
    }


}