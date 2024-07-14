/* includes */
#include "AgroNet.h"
#include "LoraModule.h"

#include "esp32-hal-timer.h"
//======================================
// Para trabalhar com sistema de arquivos
#include "esp_spiffs.h"
#include "esp_err.h"
#include <string.h>
//======================================

/* definições de constantes */
#define AIR_DATA_RATE AIR_DATA_RATE_000_03
#define SENSOR_PERIOD 300        /* valor em minutos */
#define SENDER_PERIOD_DEFAULT 50 /* valor em minutos */
#define SENDER_PERIOD_WAIT 100   /* valor em minutos */
#define main_route_max_tries 10

#define LED_PIN 2
// #define BOTAO_PIN 0

#define TEMP_PIN_1 34
#define TEMP_PIN_2 35
#define TEMP_PIN_3 32

#define BUFFER_SIZE 200
#define BUFFER_SIZE2 500
#define AMOSTRAGEM_US 50
#define AMOSTRAS 5000

esp_timer_handle_t timer1Handle;
QueueHandle_t signalQueue;   // amostragem parte 1: fila para contagem de 1 e 0
QueueHandle_t sequenceQueue; // amostragem parte 2: fila para agrupamento de 1 e 0

// Variáveis
volatile int sampleIndex = 0;
int c0 = 0, c1 = 0, sig, tempPin = TEMP_PIN_1, ctrTempPin = 0;
float dc = 0.0;
volatile bool start = true;

typedef struct
{
  int value; // 0 or 1
  int count; // Count of 0s or 1s
} SignalData_t;

/* definições para a comunicação serial */
HardwareSerial mySerial(1);                                                                             // RX, TX
LoRa_E32 e32Serial(TX_PIN, RX_PIN, &mySerial, AUX_PIN, M0_PIN, M1_PIN, UART_BPS_RATE_9600, SERIAL_8N1); // Criação do objeto SoftwareSerial

/* definição das variaveis compartilhadas */
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

void vGetConfigurations();
void IRAM_ATTR vOnReceive();
void vReceiveLoop(void *pvParameters);
void vReadTmpSnsrLoop(void *pvParameters);
void vSendDataLoop(void *pvParameters);
void vHandleACK(defaultMsg dmMsg);
void vHandleRet(defaultMsg dmMsg);
float calcFrequency();
int getPinTemp();
void clearQueue(QueueHandle_t queue);
void printQueueContents();
void signalProcessingTask(void *pvParameters);
void timerTrigger1(void *arg);

/* implementação das funções */

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
  // e32Serial.setMode(MODE_0_NORMAL);

  /* configuração da interrupção de recebimento de dados */
  pinMode(AUX_PIN, INPUT_PULLUP);
  attachInterrupt(AUX_PIN, vOnReceive, RISING);

  // Configuração do led interno
  pinMode(LED_PIN, OUTPUT);
  // Configuração do pino que recebe o sinal do sensor
  pinMode(TEMP_PIN_1, INPUT);
  pinMode(TEMP_PIN_2, INPUT);
  pinMode(TEMP_PIN_3, INPUT);
  // Configuração do timer
  const esp_timer_create_args_t timer1Conf = {
      .callback = timerTrigger1,
      .name = "Timer1",
  };
  esp_timer_create(&timer1Conf, &timer1Handle);
  esp_timer_start_periodic(timer1Handle, AMOSTRAGEM_US);
  // Cria a fila amostragem 1
  signalQueue = xQueueCreate(BUFFER_SIZE, sizeof(int));
  if (signalQueue == NULL)
  {
    printf("\nFalha ao criar fila.");
    return;
  }

  xTaskCreate(vReceiveLoop, "receiveLoop", 2048, NULL, 1, &thReceive);
  xTaskCreate(vReadTmpSnsrLoop, "ReadTmpSnsrLoop", 2048, NULL, 1, &thReadSensor);
  xTaskCreate(vSendDataLoop, "SendDataLoop", 2048, NULL, 1, &thSendData);
  // Cria a tarefa de processamento de sinal
  xTaskCreate(signalProcessingTask, "signalProcessingTask", 2048, NULL, 5, NULL);

  // Cria a fila amostragem 2
  sequenceQueue = xQueueCreate(BUFFER_SIZE2, sizeof(SignalData_t));
  if (sequenceQueue == NULL)
  {
    printf("\nFalha ao criar fila.");
    return;
  }
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

void vReadTmpSnsrLoop(void *pvParameters)
{
  sensorReadings srReading;
  float aux;
  // int count = 500;
  while (1)
  {
    vTaskDelay(10000 / portTICK_PERIOD_MS); // Espera por 10
    printf("Realizando leitura do sensor\n");
    int loop = 0;
    while (loop <= 3)
    { // Coleta a temperatura dos 3 sensores 3 vezes, por isso o loop roda 9 vezes.
      if (start)
      { // Iniciar o timer somente se start for true
        srReading.dtRead = time(NULL);
        strcpy(srReading.sensor, "TEMP");
        aux = calcFrequency();
        Serial.println(aux);
        if(aux == NULL){
          printf("leitura nula\n");
          break;
        }
        srReading.value = aux;
        srReading.idSensor = (ctrTempPin + 1);
        clearQueue(sequenceQueue);
        start = false;
        // tempPin = getPinTemp(); // Modificação para coletar informação de 3 sensores
        tempPin = TEMP_PIN_1; //ajuste Ulisses e Matheus 20/01/24
        digitalWrite(LED_PIN, HIGH);
        // esp_timer_start_periodic(timer1Handle, AMOSTRAGEM_US);
        srReading.bSent = false;
        srReading.bAck = false;
        if (xSemaphoreTake(semReadingsQueue, (TickType_t)10) == pdTRUE)
        {
          vsrReadingsQueue.push(srReading);
          bHasData = true;
          xSemaphoreGive(semReadingsQueue);
        }
      }
      // vTaskDelay(10000 / portTICK_PERIOD_MS); // Espera por 10 segundos
      vTaskDelay(10000 / portTICK_PERIOD_MS); // Espera por 10 segundos
      loop++;
    }

    vTaskDelay(SENSOR_PERIOD / portTICK_PERIOD_MS); // Espera por 1h
  }
  // while (1)
  // {
  //   for (int i = 0; i < 3; i++)
  //   {
  //     printf("Realizando leitura do sensor\n");
  //     srReading.dtRead = time(NULL);
  //     strcpy(srReading.sensor, "TEMP");
  //     srReading.value = (unsigned char)(count + i * 5);
  //     srReading.idSensor = (i + 1);
  //     srReading.bSent = false;
  //     srReading.bAck = false;
  //     Serial.print("time:");
  //     Serial.println(srReading.dtRead);
  //     if (xSemaphoreTake(semReadingsQueue, (TickType_t)10) == pdTRUE)
  //     {
  //       vsrReadingsQueue.push(srReading);
  //       bHasData = true;
  //       xSemaphoreGive(semReadingsQueue);
  //     }
  //   }
  //   count = count + 10;
  //   if (count > 700)
  //   {
  //     count = 500;
  //   };
  //   vTaskDelay((SENSOR_PERIOD * 1000) / portTICK_PERIOD_MS);
  // }
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

void timerTrigger1(void *arg)
{
  if (sampleIndex < AMOSTRAS)
  {
    sig = digitalRead(tempPin);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xQueueSendFromISR(signalQueue, (void *)&sig, &xHigherPriorityTaskWoken) != pdPASS)
    {
      printf("\nFalha ao adicionar à fila! Overflow?");
    }
    sampleIndex++;
  }
  else
  {
    sampleIndex = 0;
    dc = ((float)c1 / (c1 + c0)) * 100;
    c0 = 0;
    c1 = 0;
    start = true;
    digitalWrite(LED_PIN, LOW);
    esp_timer_stop(timer1Handle);
  }
}

// Processamento do sinal
void signalProcessingTask(void *pvParameters)
{
  int receivedSignal;
  int currentSignal = -1; // Inicia com valor inválido
  int currentCount = 0;

  while (1)
  {
    if (xQueueReceive(signalQueue, &receivedSignal, portMAX_DELAY))
    {
      if (currentSignal == -1)
      { // Primeira interação
        currentSignal = receivedSignal;
      }
      // Verifica se o sinal mudou
      if (currentSignal != receivedSignal)
      {
        SignalData_t data = {currentSignal, currentCount};
        xQueueSend(sequenceQueue, &data, portMAX_DELAY); // Envia estrutura de dados para a segunda fila
        // Reset
        currentSignal = receivedSignal;
        currentCount = 0;
      }
      // Incrementa contadores
      currentCount++;
      if (receivedSignal == 0)
      {
        c0++;
      }
      else
      {
        c1++;
      }
    }
  }
}

void printQueueContents()
{
  SignalData_t data;
  printf("\nInício da fila:");
  while (xQueueReceive(sequenceQueue, &data, pdMS_TO_TICKS(20)))
  { // 0 timeout, para não bloquear
    // ESP_LOGI("Main", "[Value: %d, Count: %d]", data.value, data.count);
  }
  printf("\nFim da fila:");
}

void clearQueue(QueueHandle_t queue)
{
  void *tempData;
  while (xQueueReceive(queue, &tempData, 0) == pdTRUE)
  {
    // Faz nada, apenas retira o item da fila
  }
}

float calcFrequency()
{
  SignalData_t currentData, nextData;
  uint64_t totalPeriodUs = 0; // total de microssegundos
  int numPeriods = 0;         // número de períodos completos
  float temp = 0.0;           // Temperatura

  printf("calcFrequency\n");
  // Se não houver dados na fila, retorne
  if (uxQueueMessagesWaiting(sequenceQueue) < 3)
  { // Precisa de pelo menos 3 itens para ter "itens do meio"
    // ESP_LOGE("calcFrequency", "Fila com menos de 3 itens.");
    printf("[%lld] Fila com menos de 3 itens.", esp_timer_get_time());
    return NULL;
  }

  // Ignore o primeiro elemento
  xQueueReceive(sequenceQueue, &currentData, portMAX_DELAY);

  while (uxQueueMessagesWaiting(sequenceQueue) > 1)
  {
    xQueueReceive(sequenceQueue, &nextData, portMAX_DELAY);
    // Calcular período usando a contagem de 'currentData' e 'nextData'
    // Isso assume que sua sequência é alternada entre 0s e 1s.
    uint64_t periodUs = (currentData.count + nextData.count) * AMOSTRAGEM_US;
    totalPeriodUs += periodUs;
    numPeriods++;
    // Prossiga para o próximo par
    currentData = nextData;
  }
  // Ignore o último item (já que ele foi parcialmente usado no último período calculado)
  xQueueReceive(sequenceQueue, &nextData, portMAX_DELAY);
  if (numPeriods > 0)
  {
    uint64_t avgPeriodUs = totalPeriodUs / numPeriods;
    float frequencyHz = 1000000.0 / avgPeriodUs; // convertendo período em frequência
    int dados = 0;
    if (frequencyHz < 115)
    { // Dados 1 - RMSE=1.15
      temp = frequencyHz * 0.30 - 19.9;
      dados = 1;
    }
    else if (frequencyHz >= 115 && frequencyHz <= 180)
    { // Dados 2 - RMSE=0.30
      temp = frequencyHz * 0.10 + 0.55;
      dados = 2;
    }
    else if (frequencyHz > 180 && frequencyHz <= 260)
    { // Dados 3 - RMSE=0.29
      temp = frequencyHz * 0.09 + 4.231;
      dados = 3;
    }
    else if (frequencyHz > 260 && frequencyHz <= 440)
    { // Dados 4 - RMSE=0.28
      temp = frequencyHz * 0.07 + 10.28;
      dados = 4;
    }
    else if (frequencyHz > 440 && frequencyHz <= 655)
    { // Dados 5 - RMSE=0.14
      temp = frequencyHz * 0.04 + 20.21;
      dados = 5;
    }
    else
    { // Dados 6 - RMSE=0.34
      temp = frequencyHz * 0.03 + 26.33;
      dados = 6;
    }
    printf("[%lld] Frequência calculada[%i]: %.2f Hz - DC: %.2fp.p - Dados: %i - Temp: %.2f", esp_timer_get_time(), tempPin, frequencyHz, dc, dados, temp);
    return temp;
  }
  else
  {
    printf("[%lld] Não foi possível calcular a frequência[%i].", esp_timer_get_time(), tempPin);
  }
  printf("nada por aqui\n");
  return NULL;
}

int getPinTemp()
{
  if (ctrTempPin == 0)
  {
    ctrTempPin++;
    return TEMP_PIN_1;
  }
  else if (ctrTempPin == 1)
  {
    ctrTempPin++;
    return TEMP_PIN_2;
  }
  else
  {
    ctrTempPin = 0;
    return TEMP_PIN_3;
  }
}
