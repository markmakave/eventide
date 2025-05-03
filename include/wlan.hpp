#pragma once

#include <string_view>
#include <cstring>

#include <esp_log.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_mac.h>

namespace eventide {

class wlan
{
public:

    wlan()
    {}

    wlan(std::string_view ssid, std::string_view password)
    {
        init(ssid, password);
    }

    ~wlan()
    {}

    void init(std::string_view ssid, std::string_view password)
    {
        [[maybe_unused]] static bool init = [](){
            ESP_ERROR_CHECK(esp_netif_init());
            ESP_ERROR_CHECK(esp_event_loop_create_default());
            return true;
        }();

        esp_netif_create_default_wifi_ap();

        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));

        ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &_event_handler, NULL, NULL));

        wifi_config_t config = {
            .ap = {
                .ssid = {},
                .password = {},
                .ssid_len = static_cast<uint8_t>(ssid.size()),
                .channel = 1,
                .authmode = WIFI_AUTH_WPA3_PSK,
                .max_connection = 1,
                .pmf_cfg = {
                    .required = true,
                },
                .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
            }
        };

        std::memcpy(reinterpret_cast<char*>(config.ap.ssid), ssid.data(), ssid.size());
        std::memcpy(reinterpret_cast<char*>(config.ap.password), password.data(), password.size());
    
        if (password.size() == 0)
            config.ap.authmode = WIFI_AUTH_OPEN;
    
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &config));
    }

    void start()
    {
        ESP_ERROR_CHECK(esp_wifi_start());
    }

    void stop()
    {
        ESP_ERROR_CHECK(esp_wifi_stop());
    }

protected:

    static void _event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
    {
        if (event_id == WIFI_EVENT_AP_STACONNECTED) {
            wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
            ESP_LOGI("app", "station " MACSTR " join, AID=%d", MAC2STR(event->mac), event->aid);
        } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
            wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
            ESP_LOGI("app", "station " MACSTR " leave, AID=%d, reason=%d", MAC2STR(event->mac), event->aid, event->reason);
        }
    }

};

}