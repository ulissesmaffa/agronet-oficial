// Definir o pinout do módulo LoRa
byte  RX_PIN  = 17;   // Pino RX do módulo E32-900T20D conectado ao pino TX da ESP32
byte  TX_PIN  = 16;   // Pino TX do módulo E32-900T20D conectado ao pino RX da ESP32
byte  M0_PIN  = 5;    // Pino M0 do módulo E32-900T20D conectado a um pino digital da ESP32
byte  M1_PIN  = 4;    // Pino M1 do módulo E32-900T20D conectado a um pino digital da ESP32
byte  AUX_PIN = 15;   // Pino AUX do módulo E32-900T20D conectado a um pino digital da ESP32


/* definição das funções */
void vPrintParameters(struct Configuration configuration);
void vPrintModuleInformation(struct ModuleInformation moduleInformation);


void vPrintParameters(struct Configuration configuration)
{
    Serial.println("----------------------------------------");

    Serial.print(F("HEAD BIN: "));
    Serial.print(configuration.HEAD, BIN);
    Serial.print(" ");
    Serial.print(configuration.HEAD, DEC);
    Serial.print(" ");
    Serial.println(configuration.HEAD, HEX);
    Serial.println(F(" "));
    Serial.print(F("AddH BIN: "));
    Serial.println(configuration.ADDH, HEX);
    Serial.print(F("AddL BIN: "));
    Serial.println(configuration.ADDL, HEX);
    Serial.print(F("Chan BIN: "));
    Serial.print(configuration.CHAN, DEC);
    Serial.print(" -> ");
    Serial.println(configuration.getChannelDescription());
    Serial.println(F(" "));
    Serial.print(F("SpeedParityBit BIN    : "));
    Serial.print(configuration.SPED.uartParity, BIN);
    Serial.print(" -> ");
    Serial.println(configuration.SPED.getUARTParityDescription());
    Serial.print(F("SpeedUARTDataRate BIN : "));
    Serial.print(configuration.SPED.uartBaudRate, BIN);
    Serial.print(" -> ");
    Serial.println(configuration.SPED.getUARTBaudRate());
    Serial.print(F("SpeedAirDataRate BIN  : "));
    Serial.print(configuration.SPED.airDataRate, BIN);
    Serial.print(" -> ");
    Serial.println(configuration.SPED.getAirDataRate());

    Serial.print(F("OptionTrans BIN       : "));
    Serial.print(configuration.OPTION.fixedTransmission, BIN);
    Serial.print(" -> ");
    Serial.println(configuration.OPTION.getFixedTransmissionDescription());
    Serial.print(F("OptionPullup BIN      : "));
    Serial.print(configuration.OPTION.ioDriveMode, BIN);
    Serial.print(" -> ");
    Serial.println(configuration.OPTION.getIODroveModeDescription());
    Serial.print(F("OptionWakeup BIN      : "));
    Serial.print(configuration.OPTION.wirelessWakeupTime, BIN);
    Serial.print(" -> ");
    Serial.println(configuration.OPTION.getWirelessWakeUPTimeDescription());
    Serial.print(F("OptionFEC BIN         : "));
    Serial.print(configuration.OPTION.fec, BIN);
    Serial.print(" -> ");
    Serial.println(configuration.OPTION.getFECDescription());
    Serial.print(F("OptionPower BIN       : "));
    Serial.print(configuration.OPTION.transmissionPower, BIN);
    Serial.print(" -> ");
    Serial.println(configuration.OPTION.getTransmissionPowerDescription());

    Serial.println("----------------------------------------");
}
void vPrintModuleInformation(struct ModuleInformation moduleInformation)
{
    Serial.println("----------------------------------------");
    Serial.print(F("HEAD BIN: "));
    Serial.print(moduleInformation.HEAD, BIN);
    Serial.print(" ");
    Serial.print(moduleInformation.HEAD, DEC);
    Serial.print(" ");
    Serial.println(moduleInformation.HEAD, HEX);

    Serial.print(F("Freq.: "));
    Serial.println(moduleInformation.frequency, HEX);
    Serial.print(F("Version  : "));
    Serial.println(moduleInformation.version, HEX);
    Serial.print(F("Features : "));
    Serial.println(moduleInformation.features, HEX);
    Serial.println("----------------------------------------");
}