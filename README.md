# agronet-oficial

Este projeto é uma implementação de um sistema embarcado utilizando o microcontrolador ESP32, operado através do FreeRTOS e configurado com o ESP-IDF. O objetivo principal deste sistema é realizar a leitura de frequências em um ou mais pinos, especificamente de ondas quadradas, variando de 10Hz até 10kHz.

## Caractetísticas

- **Microcontrolador**: ESP32-WROOM (DevKit).
- **Framework de Desenvolvimento**: FreeRTOS e ESP-IDF.
- **Faixa de Frequência**: 10Hz a 10kHz.
- **Tipo de Onda**: Quadrada.
- **Entradas**: Leitura em múltiplos pinos.

## Supported versions of frameworks and devices

| Chip     | Framework        | Versions                                                                                                                                         |
|----------|------------------|--------------------------------------------------------------------------------------------------------------------------------------------------|
| ESP32-xx | ESP-IDF          | All officially supported versions (see [Support Period Policy](https://github.com/espressif/esp-idf/blob/master/SUPPORT_POLICY.md)) and `master` |

## Como usar

## Documentação

- [Documentation]
- [Frequently asked questions](FAQ.md)

## Componentes

### Sensores de temperatura

| Componente                | Descrição                                                                     | Licença | Chip       |
|--------------------------|----------------------------------------------------------------------------------|---------|--------------------|
| **aht**                  | Driver for AHT10/AHT15/AHT20 temperature and humidity sensor                     | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 |
| **am2320**               | Driver for AM2320 temperature and humidity sensor (I2C)                          | BSD-3-Clause | esp32, esp8266, esp32s2, esp32c3 |