#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

void app_main(void)
{
    printf("BESS Firmware Starting...\n");
    
    // TODO: Initialize system components
    
    while(1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
