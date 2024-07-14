#define FREQUENCY_915
#define E32_TTL_100
#include <Arduino.h>
#include <time.h>
#include <sys/time.h>
#include "HardwareSerial.h"
#include "LoRa_E32.h"
#include <queue>
#include <list>
#include <string.h>

enum msgType
{ 
    DATA, //coleta de dados 
    ACK, //ack
    RET  //retransmissao
};

enum loraEntityType{
  GATEWAY,
  NODE
};

enum directionType{
  FORWARD,
  BACKWARD
};

/* estruturas */
struct defaultMsg {
    byte idIniSender;
    byte type;
    unsigned char message[32];
    unsigned char parity[4]; //32 bits de paridade
    int sizeOfMessage;
    time_t dtSender; /* data e hora do envio original */
};

struct sensorReadings
{
  unsigned char value;
  char sensor[5];
  time_t dtRead;
  time_t dtSend;
  byte idSensor;
  bool bSent;
  bool bAck;
};

struct loraEntity
{
  byte personalId;
  loraEntityType entityType;
  byte ADDH;
  byte ADDL;
  byte CHAN;
  directionType direction;
};

std::list<loraEntity*> routes;
loraEntity self;
loraEntity* main_route;

unsigned int getParity(unsigned char text);
int getRandInt(int max_number, int minimum_number);
void calcMsgParity(defaultMsg *msg);
bool checkMsgParity(defaultMsg msg);
loraEntity* getRoute(directionType direction, byte dest); 
void loadRoutes(std::list<loraEntity*> *routes, loraEntity *self,loraEntity* *main_route);

void loadRoutes(std::list<loraEntity*> *routes, loraEntity *self,loraEntity* *main_route){
  loraEntity *aux;
  
  aux = new loraEntity;
  aux->personalId = 0x02;
  aux->ADDL = 0x02;
  aux->ADDH = 0x00;
  aux->CHAN = 0x06;
  aux->entityType = NODE;
  aux->direction = BACKWARD;
  (*routes).push_back(aux);

  aux = new loraEntity;
  aux->personalId = 0x03;
  aux->ADDL = 0x03;
  aux->ADDH = 0x00;
  aux->CHAN = 0x04;
  aux->entityType = NODE;
  aux->direction = BACKWARD;
  (*routes).push_back(aux);

  (*self).personalId = 0xFF;
  (*self).ADDL = 0xFF;
  (*self).ADDH = 0x00;
  (*self).CHAN = 0x08;
  (*self).entityType = GATEWAY;
  (*self).direction = FORWARD;

}
loraEntity* getRoute(directionType direction, byte dest){
  for (std::list<loraEntity*>::iterator it = routes.begin(); it != routes.end(); ++it){
    if((*it)->personalId == dest && (*it)->direction == direction){
      return *it;
    }
  }
  return NULL; 
}


int getRandInt(int max_number, int minimum_number){
    srand(time(0));
    return rand() % (max_number + 1 - minimum_number) + minimum_number;
}

void calcMsgParity(defaultMsg *msg){
  int index_parity;
  //pode ir de 0 a 32
  for (int i = 0; i < (msg->sizeOfMessage); i++)
  {
    index_parity = i/8;
    if(getParity(msg->message[i]) == 1){
      msg->parity[index_parity] |= 1UL << (i-index_parity*8);
    }
  }
  
}

bool checkMsgParity(defaultMsg msg){
  defaultMsg aux;
  aux = msg; 
  calcMsgParity(&aux);
  if(aux.parity[0] == msg.parity[0] &&
     aux.parity[1] == msg.parity[1] &&
     aux.parity[2] == msg.parity[2])
  {
    return true;
  }else{
    return false;
  }
}


// retorna 1 se for impar e zero se for par
unsigned int getParity(unsigned char text){

  text ^= text >> 4;
  text &= 0xf;
  return ((0x6996 >> text) & 1);  

}