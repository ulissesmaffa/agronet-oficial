#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/queue.h"
#include "driver/pcnt.h"
#include "esp_attr.h"

#define PCNT_INPUT_SIG_IO 4  // GPIO para o sinal de entrada
// #define PCNT_INPUT_CTRL_IO 5  // GPIO para o sinal de controle (se necessário)
#define PCNT_THRESH 20000     // Limite para contagem de bordas ascendentes
#define LED_PIN 2

// Variáveis globais para o timer
static esp_timer_handle_t timer0;
static int64_t start_time = 0;

// Handler para o timer
void IRAM_ATTR timer_callback(void* arg) {
    int64_t time_elapsed = esp_timer_get_time() - start_time;
    printf("Tempo decorrido: %lld us\n", time_elapsed);

    int16_t count_alta = 0;
    pcnt_get_counter_value(PCNT_UNIT_0, &count_alta);
    printf("Bordas Altas: %d\n", count_alta);

    // Calculando a frequência
    if (time_elapsed > 0) {
        double freq = ((double)count_alta / time_elapsed) * 1000000; // Convertendo para segundos
        printf("Frequência: %f Hz\n", freq);
    }

    // Reiniciar a contagem e o timer
    pcnt_counter_clear(PCNT_UNIT_0);
    start_time = esp_timer_get_time();
}

void app_main(void) {
    // Configuração do led interno
    esp_rom_gpio_pad_select_gpio(LED_PIN); 
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    // Configuração PCNT para contagem de bordas altas
    pcnt_config_t pcnt_config_alta = {
        .pulse_gpio_num = PCNT_INPUT_SIG_IO,
        // .ctrl_gpio_num = PCNT_INPUT_CTRL_IO,
        .channel = PCNT_CHANNEL_0,
        .unit = PCNT_UNIT_0,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DIS,
        .counter_h_lim = PCNT_THRESH,
    };

    // Inicializar o PCNT
    pcnt_unit_config(&pcnt_config_alta);

    // Configurar e iniciar o timer
    const esp_timer_create_args_t timer0_config = {
        .callback = &timer_callback,
        .name = "timer0"
    };

    esp_timer_create(&timer0_config, &timer0); // Apenas cria o timer0
    pcnt_counter_pause(PCNT_UNIT_0); // Pausa contador
    pcnt_counter_clear(PCNT_UNIT_0); // Limpa contador

    //Inicio timer0 e contagem pela primeira vez. Depois segue pela função de callback 
    start_time = esp_timer_get_time(); // Pega ponto de referência inicial
    esp_timer_start_periodic(timer0, 3000000); // Timer de 3 segundos
    pcnt_counter_resume(PCNT_UNIT_0); // Inicia contagem

}
