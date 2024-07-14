#include <stdio.h>
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"

#include "driver/gpio.h"
#include "esp_log.h"

#define LED_PIN 2 // LED interno da ESP32
#define TIMER_DIVIDER (16) // Hardware timer clock divider
#define TIMER_INTERVAL0_SEC (3.0)
#define TEST_WITH_RELOAD 1  

/*
 * A sample structure to pass events
 * from the timer interrupt handler to the main program.
 */
typedef struct {
    int type;  // the type of timer's event
    int timer_group;
    int timer_idx;
    uint64_t timer_counter_value;
} timer_event_t;

// xQueueHandle timer_queue;

// Variaveis globais
volatile bool timer_interrupt_flag = false;

void IRAM_ATTR timer_group0_isr(void *para) {
    int timer_idx = (int) para; //identifica qual timer gerou interrupção
    // Obter o valor do contador do timer
    uint64_t timer_counter_value;
    timer_get_counter_value(TIMER_GROUP_0, timer_idx, &timer_counter_value);
      
    // Preencher a estrutura do evento do timer
    timer_event_t evt;
    evt.timer_group = TIMER_GROUP_0;
    evt.timer_idx = timer_idx;
    evt.timer_counter_value = timer_counter_value;
 
    // Pausar o timer
    timer_pause(TIMER_GROUP_0, timer_idx);
    // Desligar o LED
    gpio_set_level(LED_PIN, 0);
    // Setar a flag para indicar que a interrupção ocorreu
    timer_interrupt_flag = true;
}

static void tg0_timer_init(int timer_idx, bool auto_reload, double timer_interval_sec){
    // Configuração do timer
    timer_config_t config = {
        .divider = TIMER_DIVIDER, //divisor de frequência para o clk do timer
        .counter_dir = TIMER_COUNT_UP, //conta para cima
        .counter_en = TIMER_PAUSE, //inicia o timer parado
        .alarm_en = TIMER_ALARM_EN, //habilita alarme no timer
        .auto_reload = auto_reload, //quando chega no limite, volta para estado original (zero)
    };
    timer_init(TIMER_GROUP_0, timer_idx, &config);//inicializa timer

    /* Timer's counter will initially start from value below.
    Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL); //timer inicia em zero

    /* Configure the alarm value and the interrupt on alarm. */
    // timer_set_alarm_value(TIMER_GROUP_0, timer_idx, timer_interval_sec * TIMER_SCALE); //tempo em us para acionamento do alarme
    timer_set_alarm_value(TIMER_GROUP_0, timer_idx, 30000000);
    timer_enable_intr(TIMER_GROUP_0, timer_idx); //habilita interrupções
    timer_isr_register(TIMER_GROUP_0, timer_idx, timer_group0_isr, (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);

}


void app_main(void) {
    // Configuração do led interno
    esp_rom_gpio_pad_select_gpio(LED_PIN); 
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    tg0_timer_init(TIMER_0, TEST_WITH_RELOAD, TIMER_INTERVAL0_SEC);

    ESP_LOGI("MAIN", "Vou iniciar o timer");
    timer_start(TIMER_GROUP_0, TIMER_0);
    gpio_set_level(LED_PIN,1);
    
    
    while (1) {
        if (timer_interrupt_flag) {
            ESP_LOGI("MAIN", "A interrupção do TIMER ocorreu");
            timer_interrupt_flag = false;
        }
        vTaskDelay(100/ portTICK_PERIOD_MS);
    }

}
