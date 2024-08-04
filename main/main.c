#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"


void app_main(void)
{
    printf("ESP32 Hello World!\n");

    for (int i = 5; i >= 0; i--) {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    fflush(stdout);
    esp_restart();
}
