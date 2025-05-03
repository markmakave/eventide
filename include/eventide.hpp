#pragma once

#include <nvs_flash.h>

#include "wlan.hpp"
#include "task.hpp"
#include "imu.hpp"
#include "vesc.hpp"

namespace eventide
{

class system
{
public:

    struct config
    {
        struct wlan {
            const char* ssid;
            const char* password;
        } wlan;

        struct ble {

        } ble;
    };

public:

    system(config config)
    {
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
        {
            ESP_ERROR_CHECK(nvs_flash_erase());
            ESP_ERROR_CHECK(nvs_flash_init());
        }

        _wlan.init(config.wlan.ssid, config.wlan.password);

        ESP_LOGI("eventide", "system initialized");
    }

    void start()
    {
        _wlan.start();

        _imu_task.init("imu", _imu_control, _imu);
        vTaskDelay(250 / portTICK_PERIOD_MS);
        _esc_task.init("esc", _esc_control, _esc);

        while (true)
            vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

protected:

    static void _imu_control(imu& imu)
    {
        while (true)
        {
            ESP_LOGI("imu", "heartbeat");
            imu.read(nullptr, 0);
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
    }

    static void _esc_control(vesc& esc)
    {
        while (true)
        {
            ESP_LOGI("esc", "heartbeat");
            esc.write(nullptr, 0);
            vTaskDelay(500 / portTICK_PERIOD_MS);
        }
    }

protected:

    wlan _wlan;

    imu _imu;
    task _imu_task;

    vesc _esc;
    task _esc_task;
};
    
} // namespace eventide
