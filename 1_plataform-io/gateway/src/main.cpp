/* includes */
#include "AgroNet.h"
#include "LoraModule.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>

const char *ssid = "Roxeng";
const char *password = "rox@2023";

// Your Domain name with URL path or IP address with path
// String serverName = "http://piru-piru.uergs.edu.br:8443";
String serverName = "http://192.168.10.77:8443";

/* definições de constantes */
#define AIR_DATA_RATE AIR_DATA_RATE_000_03
#define SENDER_PERIOD_DEFAULT 10 /* valor em minutos */
#define SENDER_PERIOD_WAIT 50    /* valor em minutos */
#define UPDATE_TIME_PERIOD 720

/* definições para a comunicação serial */
HardwareSerial mySerial(1);                                                                             // RX, TX
LoRa_E32 e32Serial(TX_PIN, RX_PIN, &mySerial, AUX_PIN, M0_PIN, M1_PIN, UART_BPS_RATE_9600, SERIAL_8N1); // Criação do objeto SoftwareSerial

/* definição das variaveis compartilhadas */
SemaphoreHandle_t semLoraModule;
SemaphoreHandle_t semReadingsQueue;
TaskHandle_t thReceive, thSend, thUpTime;
std::queue<sensorReadings> vsrReadingsQueue;
std::list<loraEntity *>::iterator it;
timeval tv;  

bool bHasData;
int iSenderPeriod = SENDER_PERIOD_DEFAULT;
int iSenderPeriodWait = SENDER_PERIOD_WAIT;

void vGetConfigurations();
void IRAM_ATTR vOnReceive();
void vReceiveLoop(void *pvParameters);
void vSendToServerLoop(void *pvParameters);
void vUpdateTime(void *pvParameters);
void vHandleACK(defaultMsg dmMsg);
void vHandleDATA(defaultMsg dmMsg);
/* implementação das funções */

void setup()
{
  semLoraModule = xSemaphoreCreateMutex();
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
  }
      
  tv.tv_sec = 0; 
  tv.tv_usec = 0;
  settimeofday(&tv, NULL); 


  // iniciar wifi
  WiFi.begin(ssid, password);
  Serial.println("Connecting");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to WiFi network with IP Address: ");
  Serial.println(WiFi.localIP());

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

  xTaskCreate(vReceiveLoop, "receiveLoop", 2048, NULL, 1, &thReceive);
  xTaskCreate(vSendToServerLoop, "sendLoop", 6144, NULL, 1, &thSend);
  xTaskCreate(vUpdateTime, "updateTimeLoop", 4096, NULL, 1, &thUpTime);
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

void vSendToServerLoop(void *pvParameters)
{
  bool bGotData;
  time_t aux_time;
  sensorReadings srReading;
  HTTPClient http;
  String serverPath = serverName + "/sensorReading";
  String requestBody;
  while (1)
  {

    bGotData = false;

    printf("iniciando server \n");
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
        printf("path: %s\n", serverPath.c_str());
        printf("pegou dado");
        if (WiFi.status() == WL_CONNECTED)
        {
          aux_time = time(NULL);
          if(aux_time - srReading.dtRead < 2592016){
            aux_time = srReading.dtRead;
          }

          // Your Domain name with URL path or IP address with path
          http.begin(serverPath.c_str());

          // Send HTTP POST request
          Serial.print("lida_em:");
          Serial.println(srReading.dtRead);
          http.addHeader("Content-Type", "application/json");
          int httpResponseCode = http.POST("{\"sensor_id\":" + String((int)srReading.idSensor) + "," +
                                           "\"value\":" + String((int)srReading.value) + "," +
                                           "\"lida_em\":" + String(aux_time) +
                                           "}");

          if (httpResponseCode > 0)
          {
            Serial.print("HTTP Response code: ");
            Serial.println(httpResponseCode);
            String payload = http.getString();
            Serial.println(payload);
            if (payload.indexOf("\"status\":\"ok\"") > 0)
            {
              if (xSemaphoreTake(semReadingsQueue, (TickType_t)10) == pdTRUE)
              {
                if (vsrReadingsQueue.size() > 0)
                {
                  vsrReadingsQueue.pop();
                }
                xSemaphoreGive(semReadingsQueue);
              }
            }
          }
          else
          {
            Serial.printf("Error code: %\n", httpResponseCode);
            Serial.printf("Error occurred while sending HTTP REQ: %s\n", http.errorToString(httpResponseCode).c_str());
          }
          // Free resources
          http.end();
        }
        else
        {
          Serial.println("WiFi Disconnected");
        }
      }
    }
    vTaskDelay((iSenderPeriod * 1000) / portTICK_PERIOD_MS);
  }
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
      else if (dmMsg.type == DATA)
      {
        printf("DATA\n");
        vHandleDATA(dmMsg);
      }
    } // end if
  }   // end while
}

void vHandleDATA(defaultMsg dmMsg)
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
        printf("enviando ack para %x %x %x \n",sender->ADDH,
                                               sender->ADDL,
                                               sender->CHAN);
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
        vsrReadingsQueue.pop();
      }
      xSemaphoreGive(semReadingsQueue);
    }
  }
}

void vUpdateTime(void *pvParameters){
  HTTPClient http;
  String serverPath = serverName + "/time";
  String requestBody;
  while(1){
    if (WiFi.status() == WL_CONNECTED)
    {
      http.begin(serverPath.c_str());
      Serial.println(serverPath);
      // Send HTTP POST request
      http.addHeader("Content-Type", "application/json");
      int httpResponseCode = http.GET();
      Serial.println(httpResponseCode);
      if (httpResponseCode == 200)
      {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
        String payload = http.getString();
        Serial.println(payload);
        payload = payload.substring(payload.indexOf(":") + 1, payload.length() - 1);
        Serial.print("tempo server: ");
        Serial.println(payload);
        tv.tv_sec = payload.toInt();
        Serial.print("tempo convertido: ");
        Serial.println(tv.tv_sec);
        tv.tv_usec = 0;
        settimeofday(&tv, NULL); 
      }
    }
    vTaskDelay((UPDATE_TIME_PERIOD * 1000) / portTICK_PERIOD_MS);
  }
}