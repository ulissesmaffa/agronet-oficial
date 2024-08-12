#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_types.h"
#include "esp_sleep.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "esp_spiffs.h"
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/timer.h"
#include "soc/timer_group_struct.h"
#include "driver/gpio.h"
#include "driver/pcnt.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_mac.h"

#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"


#define LED_PIN       2  // LED interno da ESP32
#define TIMER_DIVIDER 80 // 80/80 = 1MHz -> incremento a cada 1ms
#define TIMER_SCALE (80000000/TIMER_DIVIDER)
#define TIMER_ALARM 3500000

#define PCNT_INPUT_SIG_IO 35 // GPIO para o sinal de entrada
#define PCNT_THRESH 20000 // Limite para contagem de bordas ascendentes
#define CONTROL_BTN_OUT_MOSFET 26 //controle de MOSFET P sinal de saída

/* WIFI */
#define BTN_OPERATION_MODE 33 //controle via btn do modo de operação 0 para coleta de dados e 1 para transmissão
#define WIFI_SSID      "AGRONET_01"
#define WIFI_PASS      "12345678"
#define WIFI_CHANNEL   1
#define MAX_STA_CONN       4
static const char *TAG = "wifi softAP";

/* SD */
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

/* Deletar arquivo */
#define PIN_DELETE_ARQ 34

/* Estruturas de dados e variáveis globais */
typedef struct {
    int i;
    uint64_t timer_counter_value;
    int16_t counter_edges;
} timer_event_t;

/* Estrutura de dados simples para controle de tempo */
typedef struct {
    int hour;
    int minute;
    int second;
} SimpleTime;

RTC_DATA_ATTR SimpleTime timeinfo;

RTC_DATA_ATTR volatile int counter_index=0;
RTC_DATA_ATTR volatile bool operationMode=1; //0 para coleta de dados e 1 para transmissão de dados
RTC_DATA_ATTR volatile bool deleteArq=0; //0 não faz nada e 1 deleta arquivo e aguarda

volatile int isDataCollectionEnabled=0;
volatile int isCollectingData=0;
timer_event_t evt;
QueueHandle_t timer_queue;

/* Funções do sistema*/
void temp_task(void *arg);
void timer_evt_task(void *arg);
void IRAM_ATTR timer_group0_isr(void *para);

void config_led();
void config_control_mosfet();
void config_pcnt();
void config_timer(int timer_idx);
void config_operation_mode();

void reset_pcnt();
void reset_timer(int timer_idx);
double calc_temp(double freq);

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
void wifi_init_softap(void);
void wifi_deinit_softap(void);
httpd_handle_t start_webserver(void);
void stop_webserver(httpd_handle_t server);
esp_err_t root_get_handler(httpd_req_t *req);

esp_err_t init_spiffs();
void get_file_info(const char *filename);
void write_to_file(const char *filename, const char *data);
void delete_file(const char *filename);

void mount_sdcard();
void copy_spiffs_to_sdcard(const char *spiffs_file, const char *sdcard_file);

void config_delete_arq();

void zerar_hora(SimpleTime *time);
void add_minutes(SimpleTime *time, int minutes);
void print_current_time(SimpleTime *time);

/* MAIN*/
void app_main(void) 
{
    config_led();
    config_control_mosfet();
    config_operation_mode();
    config_pcnt();
    reset_pcnt();
    
    config_delete_arq();

    config_timer(TIMER_0);
    
    timer_queue = xQueueCreate(10, sizeof(timer_event_t));
    // xTaskCreate(timer_evt_task, "timer_evt_task", 2048, NULL, 5, NULL);
    xTaskCreate(timer_evt_task, "timer_evt_task", 4096, NULL, 5, NULL);
    xTaskCreate(&temp_task, "temp_task", 2048, NULL, 5, NULL);

    // esp_sleep_enable_timer_wakeup(10 * 3000000); //30s
    // esp_sleep_enable_timer_wakeup(3600000000ULL); //1h
    esp_sleep_enable_timer_wakeup(900000000ULL); // 15 minutos
    esp_sleep_enable_ext0_wakeup(BTN_OPERATION_MODE, 1); // Wakeup ao pressionar o botão
    esp_sleep_enable_ext0_wakeup(PIN_DELETE_ARQ, 1); // Wakeup ao pressionar o botão
    
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Inicializa o sistema de arquivos
    if (init_spiffs() != ESP_OK){
        ESP_LOGE("MAIN","Erro ao inicializar o sistema de arquivos SPIFFS");
        return; // Não continue se falhar
    }

    // Verifica arquivo, caso não tenha, crie
    get_file_info("/spiffs/temp.txt");
    // delete_file("/spiffs/temp.txt");

    // Inicializa o cartão SD
    mount_sdcard();

    // Se for o primeiro evento, inicialize ou zere a hora
    if (counter_index == 0) {
        zerar_hora(&timeinfo);
        print_current_time(&timeinfo);
    } else {
        print_current_time(&timeinfo);
    }

    while(1){

        operationMode=gpio_get_level(BTN_OPERATION_MODE);
        deleteArq=gpio_get_level(PIN_DELETE_ARQ);

        if(!deleteArq){
            // Coleta de dados => operationMode=0
            if(!operationMode){
                if(!isCollectingData){
                    ESP_LOGI("MAIN", "MODO DE OPERAÇÃO = %i - COLETA DE DADOS",operationMode);
                    isCollectingData=1; //controla se está coletando dados
                    isDataCollectionEnabled=1; //libera para função de coletar dados
                }
            }
            // Transmissão de dados => operationMode=1
            else if(!isCollectingData){
                // Modo de transmissão de dados
                ESP_LOGI("MAIN", "MODO DE OPERAÇÃO = %i - TRANSMISSÃO DE DADOS",operationMode);
                wifi_init_softap();
                httpd_handle_t server = start_webserver();  // Inicializa o servidor web
                while (operationMode){
                    vTaskDelay(100 / portTICK_PERIOD_MS); //delay do while
                    operationMode=gpio_get_level(BTN_OPERATION_MODE);
                }
                ESP_LOGI("MAIN", "MODO DE OPERAÇÃO = %i - Vou desligar o WIFI agora (isCollectingData=%i)",operationMode,isCollectingData);
                // Desliga o Wi-Fi após a transmissão
                stop_webserver(server); 
                wifi_deinit_softap();
                zerar_hora(&timeinfo);
            }
        }else{
            ESP_LOGI("MAIN", "DELETE_ARQ = %i - DELETAR ARQUIVO",deleteArq);
            delete_file("/spiffs/temp.txt");
            while(deleteArq){
                deleteArq=gpio_get_level(PIN_DELETE_ARQ);
                zerar_hora(&timeinfo);
                vTaskDelay(100 / portTICK_PERIOD_MS); //delay do while
            }
        }

        vTaskDelay(100 / portTICK_PERIOD_MS); // Atraso para evitar um loop muito rápido
    }
}

/* Task principal do sensor */
void temp_task(void *arg)
{
    while(1){
        if(isDataCollectionEnabled){
            isDataCollectionEnabled=0; //controla a coleta de dados
            ESP_LOGI("TEMP_TASK", "Coletando dados...");
            gpio_set_level(CONTROL_BTN_OUT_MOSFET, 1); // Energiza sensor
            vTaskDelay(100 / portTICK_PERIOD_MS); // Aguarda pequeno tempo para iniciar contagem após energizar sensor
            timer_start(TIMER_GROUP_0, TIMER_0); // Inicia timer
            pcnt_counter_resume(PCNT_UNIT_0);    // Inicia contagem
            gpio_set_level(LED_PIN, 1);
        }
        vTaskDelay(100/portTICK_PERIOD_MS);
    }

 
}

/* Task para verificar fila de interrupção de timer */
void timer_evt_task(void *arg)
{
    timer_event_t evt;
    char log_buffer[256]; //gravar em arquivo
    while(1){
        if(xQueueReceive(timer_queue, &evt, portMAX_DELAY)){
            double freq = ((double)evt.counter_edges * (double)TIMER_SCALE) / (double)evt.timer_counter_value;
            double temp = calc_temp(freq);
            printf("Event timer [%i]\n",evt.i);
            // printf("Time:  %.5f s\n",(double) evt.timer_counter_value / TIMER_SCALE);
            // printf("Edges: %.2f edges\n",(double) evt.counter_edges);
            printf("hr: %02i:%02i:%02i\n",timeinfo.hour,timeinfo.minute,timeinfo.second);
            printf("Freq:  %.2f Hz\n",freq);
            printf("Temp:  %.2f ºC\n",temp);

            //Grava informação em arquivo
            sprintf(log_buffer, "[%i];Frequencia(Hz):%.2f; Temperatura(C):%.2f; Hora:%02i:%02i:%02i;",evt.i,freq,temp,timeinfo.hour,timeinfo.minute,timeinfo.second);
            write_to_file("/spiffs/temp.txt", log_buffer); //Escreve resultado em arquivo
            copy_spiffs_to_sdcard("/spiffs/temp.txt", "/sdcard/temp.txt"); //Salva do SD
            
            //Incremento de hora
            add_minutes(&timeinfo, 15);

            vTaskDelay(10/portTICK_PERIOD_MS);
            ESP_LOGI("timer_evt_task", "Vou colocar a ESP em deep-sleep");
            esp_deep_sleep_start();
        }
    }
}

/* INTERRUPÇÃO DE TIMER */
void IRAM_ATTR timer_group0_isr(void *para) 
{
    // timer_event_t evt;
    pcnt_get_counter_value(PCNT_UNIT_0, &evt.counter_edges);
    int timer_idx = (int) para;
    evt.i=counter_index++;
    evt.timer_counter_value = timer_group_get_counter_value_in_isr(TIMER_GROUP_0,timer_idx);
    xQueueSendFromISR(timer_queue, &evt, NULL); // Coloca evento na fila
    gpio_set_level(LED_PIN, 0); // Desligar o LED
    reset_pcnt(); // Reseta pcnt
    reset_timer(timer_idx); // Reconfigurar o timer para o próximo uso

    gpio_set_level(CONTROL_BTN_OUT_MOSFET,0); //desliga a energia do sensor
}

/* Configuração de LED INTERNO */
void config_led()
{
    ESP_LOGI("CONFIG_LED", "Configurando led   | LED_PIN = %i",LED_PIN);
    esp_rom_gpio_pad_select_gpio(LED_PIN); 
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
}

/* Configuração Controle MOSFET (IN/OUT) */
void config_control_mosfet()
{
    // ESP_LOGI("CONFIG_CONTROL_MOSFET", "Configurando controle mosfet in| CONTROL_BTN_IN_MOSFET = %i",CONTROL_BTN_IN_MOSFET);
    // esp_rom_gpio_pad_select_gpio(CONTROL_BTN_IN_MOSFET); 
    // gpio_set_direction(CONTROL_BTN_IN_MOSFET, GPIO_MODE_INPUT);

    ESP_LOGI("CONFIG_CONTROL_MOSFET", "Configurando controle mosfet out| CONTROL_BTN_OUT_MOSFET = %i",CONTROL_BTN_OUT_MOSFET);
    esp_rom_gpio_pad_select_gpio(CONTROL_BTN_OUT_MOSFET); 
    gpio_set_direction(CONTROL_BTN_OUT_MOSFET, GPIO_MODE_OUTPUT);
}

/* Configuração de PCNT */
void config_pcnt()
{
    gpio_set_direction(PCNT_INPUT_SIG_IO, GPIO_MODE_INPUT);
    //gpio_set_pull_mode(PCNT_INPUT_SIG_IO, GPIO_FLOATING);

    pcnt_config_t config = {
        .pulse_gpio_num = PCNT_INPUT_SIG_IO,
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

/* Configuração de timer */
void config_timer(int timer_idx)
{
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

/* Configuração do botão externo de modo de operação */
void config_operation_mode()
{
    ESP_LOGI("CONFIG_OPERATION_MODE", "Configurando controle por botão do modo de operação| BTN_OPERATION_MODE = %i",BTN_OPERATION_MODE);
    esp_rom_gpio_pad_select_gpio(BTN_OPERATION_MODE); 
    gpio_set_direction(BTN_OPERATION_MODE, GPIO_MODE_INPUT);
}

/* Resetar o contador de bordas ascendetes */
void reset_pcnt()
{
    pcnt_counter_pause(PCNT_UNIT_0); // Pausa contador
    pcnt_counter_clear(PCNT_UNIT_0); // Limpa contador
}

/* Resetar o timer */
void reset_timer(int timer_idx)
{
    timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, timer_idx);// Limpar a bandeira de interrupção do timer usando a API do driver
    timer_pause(TIMER_GROUP_0, timer_idx); // Parar o timer
    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);// Redefinir o valor do contador do timer para zero    
    timer_set_alarm_value(TIMER_GROUP_0, timer_idx, TIMER_ALARM);// Configurar o valor do alarme (3 segundos neste caso)
    timer_set_alarm(TIMER_GROUP_0, timer_idx, TIMER_ALARM_EN);// Habilitar o alarme do timer
}

//Essas esquações devem vir de arquivo
double calc_temp(double freq)
{
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

/* Inicializa sistema de arquivos */
esp_err_t init_spiffs()
{
    ESP_LOGI("SPIFFS","Inicializando o sistema de arquivos SPIFFS");

    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true};

    // Registra o sistema de arquivos no VFS
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL){
            ESP_LOGE("SPIFFS","Falha ao montar ou formatar o sistema de arquivos");
        }
        else if (ret == ESP_ERR_NOT_FOUND){
            ESP_LOGE("SPIFFS","Partição SPIFFS não encontrada");
        }
        else{
            ESP_LOGE("SPIFFS","Erro desconhecido ao inicializar SPIFFS: %s",esp_err_to_name(ret));
        }
    }
    else
    {
        size_t total = 0, used = 0;
        if (esp_spiffs_info(NULL, &total, &used) == ESP_OK){
            ESP_LOGI("SPIFFS","Sistema de arquivos montado, total: %d, usado: %d",total, used);
        }
    }
    return ret;
}

/* Cria arquivo ou pega arquivo que existe */
void get_file_info(const char *filename)
{
    FILE *f = fopen(filename, "r");
    if (f == NULL)
    {
        // O arquivo não existe, então cria um novo arquivo
        f = fopen(filename, "w");
        if (f == NULL)
        {
            ESP_LOGE("get_file_info","Não foi possível criar o arquivo %s", filename);
            return;
        }
        ESP_LOGI("get_file_info","Arquivo %s criado", filename);
    }
    else
    {
        // O arquivo existe, lê as informações do arquivo
        int line_count = 0;
        char buffer[128]; // Buffer para ler cada linha

        while (fgets(buffer, sizeof(buffer), f) != NULL){
            line_count++;
        }
        ESP_LOGI("get_file_info","Arquivo %s contém %d linhas", filename, line_count);
    }
    fclose(f);
}

/* Deleta arquivo */
void delete_file(const char *filename)
{
    if (remove(filename) == 0){
        ESP_LOGI("delete_file","Arquivo '%s' excluído com sucesso.", filename);
    }
    else{
        ESP_LOGE("delete_file","Falha ao excluir o arquivo '%s'.", filename);
    }
}

/* Escreve no arquivo */
void write_to_file(const char *filename, const char *data)
{
    FILE *f = fopen(filename, "a");
    if (f == NULL)
    {
        ESP_LOGE("write_to_file","Falha ao abrir o arquivo %s para escrita", filename);
        return;
    }

    fprintf(f, "%s\n", data); // Escreve a linha no arquivo e adiciona uma quebra de linha
    fclose(f);
    // ESP_LOGI("write_to_file","Dados gravados no arquivo: %s", data);
}

/*======================================= WIFI ============================================*/

/* Escuta do wifi e detecção de eventos */
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d",
                 MAC2STR(event->mac), event->aid);
    }
}

/* Inicia wifi como access point */
void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .channel = WIFI_CHANNEL,
            .password = WIFI_PASS,
            .max_connection = MAX_STA_CONN,
#ifdef CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT
            .authmode = WIFI_AUTH_WPA3_PSK,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
#else /* CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT */
            .authmode = WIFI_AUTH_WPA2_PSK,
#endif
            .pmf_cfg = {
                    .required = true,
            },
        },
    };
    if (strlen(WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             WIFI_SSID, WIFI_PASS, WIFI_CHANNEL);
}

/* Desliga wifi */
void wifi_deinit_softap(void)
{
    ESP_ERROR_CHECK(esp_wifi_stop());
    ESP_ERROR_CHECK(esp_wifi_deinit());
    ESP_LOGI(TAG, "WiFi desativado após transmissão.");
}

/* Inicialização do servidor web */
httpd_handle_t start_webserver(void) 
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t root = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = root_get_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &root);
    }
    return server;
}

/* Parar webserver */
void stop_webserver(httpd_handle_t server)
{
    if (server != NULL) {
        httpd_stop(server);
        ESP_LOGI("WEB_SERVER", "Servidor web parado.");
    }
}

/* Handler para o URI root */
esp_err_t root_get_handler(httpd_req_t *req) 
{
    const char* filename = "/spiffs/temp.txt"; // Altere para o nome do seu arquivo
    FILE* f = fopen(filename, "r");
    if (f == NULL) {
        ESP_LOGE("root_get_handler", "Falha ao abrir o arquivo %s para leitura", filename);
        const char* error_resp = "<html><body><h1>Erro</h1><p>Falha ao ler o arquivo.</p></body></html>";
        httpd_resp_send(req, error_resp, strlen(error_resp));
        return ESP_FAIL;
    }

    // Formata a resposta HTML
    const char* resp_template_start = "<html><body><h1>Dados coletados</h1><pre>";
    const char* resp_template_end = "</pre></body></html>";

    httpd_resp_send_chunk(req, resp_template_start, strlen(resp_template_start));

    char line[128]; // Buffer para leitura de linhas do arquivo
    while (fgets(line, sizeof(line), f) != NULL) {
        httpd_resp_send_chunk(req, line, strlen(line));
    }

    fclose(f);

    httpd_resp_send_chunk(req, resp_template_end, strlen(resp_template_end));
    httpd_resp_send_chunk(req, NULL, 0); // Envia o último pedaço da resposta (NULL para sinalizar o fim)

    return ESP_OK;
}

/*======================================= SD ============================================*/

void mount_sdcard() {
    esp_err_t ret;

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = VSPI_HOST;

    // Configuração do barramento SPI
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE("SDCARD", "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_card_t* card;
    ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE("SDCARD", "Failed to mount filesystem. "
                                "If you want the card to be formatted, set format_if_mount_failed = true.");
        } else {
            ESP_LOGE("SDCARD", "Failed to initialize the card (%s). "
                                "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return;
    }

    ESP_LOGI("SDCARD", "SD card mounted successfully");
}
  
void copy_spiffs_to_sdcard(const char *spiffs_file, const char *sdcard_file) {
    FILE *spiffs_f = fopen(spiffs_file, "r");
    if (spiffs_f == NULL) {
        ESP_LOGE("COPY", "Failed to open SPIFFS file for reading");
        return;
    }

    FILE *sdcard_f = fopen(sdcard_file, "w");
    if (sdcard_f == NULL) {
        ESP_LOGE("COPY", "Failed to open SD card file for writing");
        fclose(spiffs_f);
        return;
    }

    char buffer[128];
    while (fgets(buffer, sizeof(buffer), spiffs_f) != NULL) {
        fputs(buffer, sdcard_f);
    }

    fclose(spiffs_f);
    fclose(sdcard_f);
    ESP_LOGI("COPY", "File copied from SPIFFS to SD card successfully");
}

//usuário apagar arquivo para recomeçar
void config_delete_arq(){
    ESP_LOGI("CONFIG_DELETE_ARQ", "Configurando controle por botão para deletar arquivo | PIN = %i",PIN_DELETE_ARQ);
    esp_rom_gpio_pad_select_gpio(PIN_DELETE_ARQ); 
    gpio_set_direction(PIN_DELETE_ARQ, GPIO_MODE_INPUT);
}

//data e hora inicio
void zerar_hora(SimpleTime *time) {
    time->hour = 0;
    time->minute = 0;
    time->second = 0;
}

void add_minutes(SimpleTime *time, int minutes) {
    // Incrementa os minutos
    time->minute += minutes;

    // Verifica se os minutos excedem 59
    if (time->minute >= 60) {
        // Incrementa as horas correspondentes
        time->hour += time->minute / 60;
        // Ajusta os minutos para ficar no intervalo de 0 a 59
        time->minute %= 60;
    }
}

void print_current_time(SimpleTime *time) {
    printf("Hora atual: %02d:%02d:%02d\n", time->hour, time->minute, time->second);
}