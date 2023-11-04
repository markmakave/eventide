
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_pm.h"
#include "esp_timer.h"

#include "mpu.hpp"

#include <cmath>
#include <cassert>

extern "C" {

void app_main() {

    esp_pm_config_esp32_t pm_config;
    assert(esp_pm_get_configuration(&pm_config) == ESP_OK);

    ESP_LOGI("system", "frequency [%d:%d]", pm_config.max_freq_mhz, pm_config.min_freq_mhz);

    eventide::mpu mpu(33, 32, 400000);

    while (true) {
        auto angle = mpu.angle();

        printf(">roll:%f\n>pitch:%f\n", angle.first, angle.second);

        vTaskDelay(0);
    }
}

}
