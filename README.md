# Agronet

## Descrição do Projeto
Agronet é uma estação meteorológica baseada em um sistema embarcado, utilizando o microcontrolador ESP32. Este sistema opera através do FreeRTOS e é configurado usando o ESP-IDF. Destacando-se por sua modularidade e flexibilidade, Agronet permite a fácil integração e configuração de diferentes tipos de sensores para monitoramento ambiental. A plataforma suporta a leitura de frequências em um ou mais pinos, abrangendo uma variedade de sinais, de ondas quadradas com frequências variando de 10Hz até 10kHz.

## Configuração Modular de Sensores
Agronet introduz um sistema de configuração altamente modular que não apenas facilita a conversão de unidades de medida, mas também permite a definição paramétrica dos tipos de sensores e seus pinos correspondentes através de arquivos externos. Essa abordagem torna o sistema altamente adaptável e personalizável para diferentes necessidades de monitoramento.

### Implementação de Conversão de Unidades
- **Arquivos de Configuração**: As equações de conversão e os parâmetros dos sensores, incluindo os tipos de sensores e os pinos em que estão conectados, são definidos em arquivos de configuração externos. Isso permite uma personalização fácil e rápida do sistema para diferentes aplicações.
- **Flexibilidade**: Os usuários podem adicionar, remover ou modificar sensores simplesmente alterando os arquivos de configuração, sem necessidade de modificar o código fonte do software.
- **Exemplo Prático**: Um termistor NTC de 20kΩ pode ser configurado para converter leituras de frequência em temperatura, com todos os detalhes necessários, incluindo o pino de conexão, especificados no arquivo de configuração.

## Instalação e Uso


## Licença


## Componentes

### Sensores de temperatura

| Componente | Descrição                                         | Licença      | Chips Suportados                    |
|------------|---------------------------------------------------|--------------|-------------------------------------|
| **ntc_20k**| Driver para sensor de temperatura NTC de 20kΩ, que associa uma resistência de 20kΩ a uma temperatura ambiente de 25°C. | (Sua Licença) | esp32, esp8266, esp32s2, esp32c3 |

## Documentação

- [Documentation]()
- [Frequently asked questions](FAQ.md)

## Mantenedores de biblioteca

- [Ulisses Maffazioli](https://github.com/ulissesmaffa)

