/* Módulo 2 - Memória parte 2
    Implementação do código para verificar
    a disponibilidade de Memória
    
    Bobsien - Maurício C. de Camargo
*/
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

void MemoryView();

void app_main(void)
{
    MemoryView();

    int var; //será alocada no .bss
    int var1=123 //será alocada no .data

    while(1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void MemoryView()
{
    printf("Verificacao de memoria\n");

    int Heap = (xPortGetFreeHeapSize())/1024;

    int DRam = (heap_caps_get_free_size(MALLOC_CAP_8BIT))/1024;
    int IRam = (heap_caps_get_free_size(MALLOC_CAP_32BIT) - heap_caps_get_free_size(MALLOC_CAP_8BIT))/1024;

    int freeBlock = (heap_caps_get_largest_free_block(MALLOC_CAP_8BIT))/1024;

    printf("Heap %d kB\n ", Heap);
    printf("DRam %d kB\n ", DRam);
    printf("IRam %d kB\n ", IRam);
    printf("freeBlock %d kB\n ", freeBlock);

} //end MemoryView