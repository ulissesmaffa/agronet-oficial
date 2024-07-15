/* includes LoRa*/
#include "AgroNet.h"
#include "LoraModule.h"
#include "esp32-hal-timer.h"

/* Includes sensor*/
#include "driver/pcnt.h"
#include "esp_timer.h"
#include "esp_attr.h"

/* definições de constantes LoRa*/
#define AIR_DATA_RATE AIR_DATA_RATE_000_03
#define SENSOR_PERIOD 300        /* valor em minutos */
#define SENDER_PERIOD_DEFAULT 50 /* valor em minutos */
#define SENDER_PERIOD_WAIT 100   /* valor em minutos */
#define main_route_max_tries 10

/* Definições de constantes sensor*/
#define PCNT_INPUT_SIG_IO 35 // GPIO para o sinal de entrada
#define PCNT_THRESH 20000 // Limite para contagem de bordas ascendentes
#define LED_PIN 2  // LED interno da ESP32
#define TIMER_DIVIDER 80 // 80/80 = 1MHz -> incremento a cada 1ms
#define TIMER_SCALE (80000000/TIMER_DIVIDER)

#define BTN_PIN 34 //controle da chamada do sensor
#define CONTROL_BTN_IN_MOSFET 33 //controle de MOSFET P sinal de entrada
#define CONTROL_BTN_OUT_MOSFET 26 //controle de MOSFET P sinal de saída

/* definições para a comunicação serial LoRa*/
HardwareSerial mySerial(1);                                                                             // RX, TX
LoRa_E32 e32Serial(TX_PIN, RX_PIN, &mySerial, AUX_PIN, M0_PIN, M1_PIN, UART_BPS_RATE_9600, SERIAL_8N1); // Criação do objeto SoftwareSerial

/* definição das variaveis compartilhadas LoRa*/
SemaphoreHandle_t semFlags;
SemaphoreHandle_t semReadingsQueue;
TaskHandle_t thReceive, thReadSensor, thSendData;
std::queue<sensorReadings> vsrReadingsQueue;
std::list<loraEntity *>::iterator it;
bool bHasData;
int iSenderPeriod = SENDER_PERIOD_DEFAULT;
int iSenderPeriodWait = SENDER_PERIOD_WAIT;
int main_route_tries = 0;
int forward_routes_number = 0;
timeval tv;

/* Variáveis e estruturas de dados Sensor*/
typedef struct {
    int i;
    uint64_t timer_counter_value;
    int16_t counter_edges;
} timer_event_t;

volatile int counter_index=0;
timer_event_t evt;
QueueHandle_t timer_queue;

// Variáveis para o timer high resolution
static esp_timer_handle_t timer0;
static int64_t start_time = 0;

/* funções comunicação LoRa e gerenciamento geral do sistema*/
void vGetConfigurations();
void IRAM_ATTR vOnReceive();
void vReceiveLoop(void *pvParameters);
void vSendDataLoop(void *pvParameters);
void vHandleACK(defaultMsg dmMsg);
void vHandleRet(defaultMsg dmMsg);

/* Funções Sensor*/
void config_pcnt();
void reset_pcnt();
void config_led();
void config_timer();
void IRAM_ATTR timer_callback(void* arg);
void timer_evt_task(void *arg);
double calc_temp(double freq);
void vReadTmpSnsrLoop(void *pvParameters);
void timer_evt_task(void *arg);
void coleta_dados();

/* implementação das funções LoRa e gerenciamento geral do sistema*/
void setup()
{
  semFlags = xSemaphoreCreateMutex();
  semReadingsQueue = xSemaphoreCreateMutex();

  // Inicializar a porta serial
  Serial.begin(115200);
  Serial.println("Inicializando");
  while (!Serial)
  {
    ; // wait for serial port to connect. Needed for native USB
  }
  delay(1);

  loadRoutes(&routes, &self, &main_route);

  printf("rotas carregadas\n");
  for (it = routes.begin(); it != routes.end(); ++it)
  {
    printf("id: %x\n", (*it)->personalId);
    printf("entityType: %d\n", (*it)->entityType);
    printf("Direction: %d\n", (*it)->direction);
    if ((*it)->direction == FORWARD)
    {
      forward_routes_number = forward_routes_number + 1;
    }
  }

  tv.tv_sec = 0;
  tv.tv_usec = 0;
  settimeofday(&tv, NULL);

  /* Iniciar modulo lora */
  e32Serial.begin();

  ResponseStructContainer c;
  c = e32Serial.getConfiguration();
  Configuration configuration = *(Configuration *)c.data;
  printf("endereco: %x - %x - %x\n", self.ADDH, self.ADDL, self.CHAN);
  /* seta as configuraçoes */
  configuration.SPED.airDataRate = AIR_DATA_RATE;
  configuration.SPED.uartBaudRate = UART_BPS_9600;
  configuration.ADDH = self.ADDH;
  configuration.ADDL = self.ADDL;
  configuration.CHAN = self.CHAN;
  configuration.OPTION.fec = FEC_1_ON;
  configuration.OPTION.ioDriveMode = IO_D_MODE_PUSH_PULLS_PULL_UPS;
  configuration.OPTION.transmissionPower = POWER_20;
  configuration.OPTION.fixedTransmission = FT_FIXED_TRANSMISSION;
  configuration.OPTION.wirelessWakeupTime = WAKE_UP_250;
  configuration.SPED.uartParity = MODE_00_8N1;

  ResponseStatus rs = e32Serial.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
  vPrintParameters(configuration);

  c.close();

  /* configuração da interrupção de recebimento de dados */
  pinMode(AUX_PIN, INPUT_PULLUP);
  attachInterrupt(AUX_PIN, vOnReceive, RISING);

  /* Configuração sensor */
  config_led();
  config_pcnt();
  reset_pcnt();
  config_timer();
  timer_queue = xQueueCreate(10, sizeof(timer_event_t));

  xTaskCreate(vReceiveLoop, "receiveLoop", 2048, NULL, 1, &thReceive);
  xTaskCreate(vSendDataLoop, "SendDataLoop", 2048, NULL, 1, &thSendData);

  /* Tarefas do sensor  */
   xTaskCreate(vReadTmpSnsrLoop, "ReadTmpSnsrLoop", 2048, NULL, 1, &thReadSensor);
   xTaskCreate(timer_evt_task, "timer_evt_task", 2048, NULL, 5, NULL);
}

/* loop desativado pois o freertos executa por threads */
void loop()
{
  vTaskDelete(NULL);
}

void IRAM_ATTR vOnReceive()
{
  xTaskResumeFromISR(thReceive);
}

void vReceiveLoop(void *pvParameters)
{
  ResponseStructContainer rs;
  int valor;
  int diferenca = 0;
  String message;

  while (1)
  {
    vTaskSuspend(NULL);
    defaultMsg dmMsg;
    if (e32Serial.available() > 1)
    {
      rs = e32Serial.receiveMessage(sizeof(defaultMsg));
      dmMsg = *(defaultMsg *)rs.data;
      printf("recebido de %x\n", dmMsg.idIniSender);
      if (dmMsg.type == ACK)
      {
        printf("ACK\n");
        vHandleACK(dmMsg);
      }
      else if (dmMsg.type == RET)
      {
        printf("RET\n");
        vHandleRet(dmMsg);
      }
    } // end if
  }   // end while
}

void vHandleRet(defaultMsg dmMsg)
{
  sensorReadings srReading, srAux;
  loraEntity *sender;
  defaultMsg dmACK;
  bool sendACK = false;

  memcpy(&srReading, dmMsg.message, dmMsg.sizeOfMessage);
  if (checkMsgParity(dmMsg))
  {
    printf("valor: %u - sender: %x\n", srReading.value, dmMsg.idIniSender);
    if (xSemaphoreTake(semReadingsQueue, (TickType_t)10) == pdTRUE)
    {
      vsrReadingsQueue.push(srReading);
      sendACK = true;
      xSemaphoreGive(semReadingsQueue);
    }

    if (sendACK)
    {
      printf("montando ack\n");
      sender = getRoute(BACKWARD, dmMsg.idIniSender);
      if (sender != NULL)
      {
        dmACK.dtSender = time(NULL);
        dmACK.idIniSender = self.personalId;
        dmACK.type = ACK;
        memcpy(&dmACK.message, dmMsg.message, dmMsg.sizeOfMessage);
        dmACK.sizeOfMessage = dmMsg.sizeOfMessage;
        memcpy(&dmACK.parity, dmMsg.parity, sizeof(dmMsg.parity));
        ResponseStatus rsACK;
        printf("enviando ack\n");
        rsACK = e32Serial.sendFixedMessage(sender->ADDH,
                                           sender->ADDL,
                                           sender->CHAN,
                                           &dmACK,
                                           sizeof(dmACK));
        Serial.println(rsACK.getResponseDescription());
      }
    }
  }
}

void vHandleACK(defaultMsg dmMsg)
{
  sensorReadings srReading, srAux;
  if (checkMsgParity(dmMsg))
  {
    printf("ack recebido\n");
    memcpy(&srReading, dmMsg.message, dmMsg.sizeOfMessage);
    if (xSemaphoreTake(semReadingsQueue, (TickType_t)10) == pdTRUE)
    {
      srAux = vsrReadingsQueue.front();
      if (srReading.dtRead == srAux.dtRead && srReading.value == srAux.value)
      {
        printf("pop\n");
        vsrReadingsQueue.pop();
      }
      xSemaphoreGive(semReadingsQueue);
      if (xSemaphoreTake(semFlags, (TickType_t)10) == pdTRUE)
      {
        main_route_tries = 0;
        xSemaphoreGive(semFlags);
      }
    }
    if (dmMsg.dtSender > time(NULL))
    {
      Serial.println("atualizando tempo");
      tv.tv_sec = dmMsg.dtSender;
      tv.tv_usec = 0;
      settimeofday(&tv, NULL);
    }
  }
}

void vSendDataLoop(void *pvParameters)
{
  sensorReadings srReading;
  defaultMsg dmMsg;
  bool bGotData;
  while (1)
  {
    printf("iniciando envio\n");
    bGotData = false;
    if (xSemaphoreTake(semReadingsQueue, (TickType_t)10) == pdTRUE)
    {
      if (vsrReadingsQueue.size() > 0)
      {
        srReading = vsrReadingsQueue.front();
        bGotData = true;
      }
      xSemaphoreGive(semReadingsQueue);
      if (bGotData)
      {
        // printf("sensor: %s\n valor: %u\n", srReading.sensor, srReading.value);
        dmMsg.idIniSender = self.personalId;
        dmMsg.dtSender = time(NULL);
        dmMsg.sizeOfMessage = sizeof(srReading);
        if (main_route->entityType == GATEWAY)
        {
          dmMsg.type = DATA;
        }
        else
        {
          dmMsg.type = RET;
        }

        memcpy(dmMsg.message, &srReading, dmMsg.sizeOfMessage);
        calcMsgParity(&dmMsg);

        printf("enviando %u para: H%x-L%x-C%x\n",
               srReading.value,
               main_route->ADDH,
               main_route->ADDL,
               main_route->CHAN);
        ResponseStatus rs = e32Serial.sendFixedMessage(main_route->ADDH,
                                                       main_route->ADDL,
                                                       main_route->CHAN,
                                                       &dmMsg,
                                                       sizeof(dmMsg));

        printf("%s\n", rs.getResponseDescription());
        if (rs.code == E32_SUCCESS)
        {
          if (xSemaphoreTake(semFlags, (TickType_t)10) == pdTRUE)
          {
            main_route_tries = main_route_tries + 1;
            xSemaphoreGive(semFlags);
          }
        }
        printf("tentativa %d\n", main_route_tries);
        if (main_route_tries > main_route_max_tries && forward_routes_number > 1)
        {
          printf("trocando rotas\n");
          it = routes.begin();
          routes.splice(routes.end(), routes, it);
          for (it = routes.begin(); it != routes.end(); ++it)
          {
            if ((*it)->direction == FORWARD &&
                (*it)->personalId != main_route->personalId)
            {
              main_route = (*it);
              routes.splice(routes.begin(), routes, it);
              main_route_tries = 0;
              break;
            }
          }
          for (it = routes.begin(); it != routes.end(); ++it)
          {
            printf("id: %x\n", (*it)->personalId);
            printf("entityType: %d\n", (*it)->entityType);
            printf("Direction: %d\n", (*it)->direction);
          }
        }
      }
    }
    if (bGotData = false)
    {
      printf("sem dados, wait");
      vTaskDelay((iSenderPeriodWait * 1000) / portTICK_PERIOD_MS);
    }
    else
    {
      vTaskDelay(((iSenderPeriod + getRandInt(5, 1)) * 1000) / portTICK_PERIOD_MS);
    }
  }
}

//===============================================================================================

/* Configuração de PCNT */
void config_pcnt()
{
    pcnt_config_t config = {
        .pulse_gpio_num = PCNT_INPUT_SIG_IO,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DIS,
        .counter_h_lim = PCNT_THRESH,
        .unit = PCNT_UNIT_0,
        .channel = PCNT_CHANNEL_0,
    };
    pcnt_unit_config(&config); // Inicializar o PCNT
    /* Configure and enable the input filter */
    pcnt_set_filter_value(PCNT_UNIT_0, 100);
    pcnt_filter_enable(PCNT_UNIT_0);
}

/* Resetar o contador de bordas ascendetes */
void reset_pcnt()
{
    pcnt_counter_pause(PCNT_UNIT_0); // Pausa contador
    pcnt_counter_clear(PCNT_UNIT_0); // Limpa contador
}

/* Configuração de LED INTERNO */
void config_led()
{
  pinMode(LED_PIN, OUTPUT);
}

/* Configuração do timer high resolution */
void config_timer()
{
   printf("Configurando timer0 para sensor\n");
   const esp_timer_create_args_t timer0_config = {
        .callback = &timer_callback,
        .name = "timer0"
    };
    esp_timer_create(&timer0_config, &timer0); // Apenas cria o timer0
}

/* Interrupção do TIMER*/
void IRAM_ATTR timer_callback(void* arg) {    
    pcnt_get_counter_value(PCNT_UNIT_0, &evt.counter_edges);
    evt.i=counter_index++;
    evt.timer_counter_value = esp_timer_get_time() - start_time;
    xQueueSendFromISR(timer_queue, &evt, NULL); // Coloca evento na fila
    reset_pcnt(); // reseta pcnt
    esp_timer_stop(timer0); // parar funcionamento do timer
    digitalWrite(LED_PIN, LOW);
}


/* Cáculo de temperatura a partir da frequencia coletada
AJUSTE: ESSAS EQUAÇÕES DEVEM VIR DE UM ARQUIVO */
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

/* Tarefa principal do sensor */
void vReadTmpSnsrLoop(void *pvParameters)
{
  sensorReadings srReading;

  //Essas variáveis, estão sendo controladas fisicamente, devem ser alteradas, para serem controladas por software.
  bool btn_status=false;
  bool last_btn_status=false;
  bool ctr_mosfet_in_status=false;

  while(1){
    // vTaskDelay(10000 / portTICK_PERIOD_MS); // Esse Delay não é necessário, está aqui apenas para teste
    // coleta_dados();

    /* Controle feito fisicamente, deve ser alterado para controle via software. Para teste, comentar essa parte e descomentar parte de cima */
    //btn_status = gpio_get_level(BTN_PIN); //modo feito em IDF
    btn_status = digitalRead(BTN_PIN);
    if(last_btn_status && !btn_status) {
      coleta_dados();
    }
    last_btn_status = btn_status;
    vTaskDelay(10/portTICK_PERIOD_MS);

    // ctr_mosfet_in_status = gpio_get_level(CONTROL_BTN_IN_MOSFET); //modo feito em IDF
    ctr_mosfet_in_status= digitalRead(CONTROL_BTN_IN_MOSFET);
    if (ctr_mosfet_in_status){
      digitalWrite(LED_PIN, HIGH);
      digitalWrite(CONTROL_BTN_OUT_MOSFET, HIGH);
    }else{
      digitalWrite(LED_PIN, LOW);
      digitalWrite(CONTROL_BTN_OUT_MOSFET, LOW);
    }
  }
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

/* Configuração de BTN */
void config_btn(){
  pinMode(BTN_PIN, INPUT);
  /* IDF VERSION */
  // esp_rom_gpio_pad_select_gpio(BTN_PIN); 
  // gpio_set_direction(BTN_PIN, GPIO_MODE_INPUT);
}

/* Configuração Controle MOSFET (IN/OUT) */
void config_control_mosfet(){
  pinMode(CONTROL_BTN_IN_MOSFET, INPUT);
  pinMode(CONTROL_BTN_OUT_MOSFET, OUTPUT);

  /* IDF VERSION */
  // esp_rom_gpio_pad_select_gpio(CONTROL_BTN_IN_MOSFET); 
  // gpio_set_direction(CONTROL_BTN_IN_MOSFET, GPIO_MODE_INPUT);

  // esp_rom_gpio_pad_select_gpio(CONTROL_BTN_OUT_MOSFET); 
  // gpio_set_direction(CONTROL_BTN_OUT_MOSFET, GPIO_MODE_OUTPUT);
}

/* Ações necessárias para coleta de dados */
void coleta_dados()
{
  printf("Coletando dados...\n");
  digitalWrite(LED_PIN, HIGH);
  start_time = esp_timer_get_time(); // Pega ponto de referência inicial
  esp_timer_start_periodic(timer0, 3000000); // Timer de 3 segundos
  pcnt_counter_resume(PCNT_UNIT_0); // Inicia contagem
}