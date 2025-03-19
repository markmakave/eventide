
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

extern "C" void app_main()
{
    while (true)
    {
        ESP_LOGI("app", "hello, world");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
