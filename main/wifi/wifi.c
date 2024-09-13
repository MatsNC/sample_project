#include "wifi.h"


void send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
#ifdef ESP_NOW_LOG
    if (status == ESP_NOW_SEND_SUCCESS)
    {
        ESP_LOGI(TAG_WIFI, "ESP_NOW_SEND_SUCCESS");
    }
    else
    {
        ESP_LOGW(TAG_WIFI, "ESP_NOW_SEND_FAIL");
    }
#endif
}

// void recv_cb(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len)
// {
//     uint8_t out_type;
//     out_type = atoi((char *)data);
//     ESP_LOGI(TAG, "Data received: " MACSTR " %s", MAC2STR(esp_now_info->src_addr), data);
//     printf("%d\n", out_type);
//     switch (out_type)
//     {
//     case 0:
//         B_Caliente_Up = 1;
//         B_Fria_Down = 0;
//         B_Fria_Up = 0;
//         B_Caliente_Down = 0;
//         printf("Agua Natural\n");
//         break;
//     case 1:
//         B_Fria_Up = 1;
//         B_Fria_Down = 0;
//         B_Caliente_Down = 0;
//         B_Caliente_Up = 0;
//         printf("Agua Fria\n");
//         break;
//     case 2:
//         B_Caliente_Down = 1;
//         B_Fria_Down = 0;
//         B_Fria_Up = 0;
//         B_Caliente_Up = 0;
//         printf("Agua Gasificada\n");
//         break;
//     }
// }

esp_err_t init_esp_now()
{
    esp_now_init();
    // esp_now_register_recv_cb(recv_cb);
    esp_now_register_send_cb(send_cb);
    esp_now_off = 0;
    // Añade al peer
    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, peer_mac, 6);
    peer_info.channel = 0;
    peer_info.ifidx = ESP_IF_WIFI_STA;
    peer_info.encrypt = false;
    ESP_ERROR_CHECK(esp_now_add_peer(&peer_info));
    ESP_LOGI(TAG_WIFI, "esp now init completed");
    return ESP_OK;
}

/**
 * @brief Funcion que inicializa WIFI
 * @param [in] void
 * @return esp_err_t Estado de la operacion (ESP_OK si todo fue bien, ESP_FAIL si hubo algun problema)
 */

esp_err_t init_wifi()
{
    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    esp_netif_init();
    esp_event_loop_create_default();
    esp_wifi_init(&wifi_init_config);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_storage(WIFI_STORAGE_FLASH);
    esp_wifi_start();
    wifi_off = 0;
    ESP_LOGI(TAG_WIFI, "wifi init completed");
    return ESP_OK;
}

/**
 * @brief Funcion que apaga WIFI
 * @param [in] void
 * @return void
 */

void deinit_wifi()
{
    wifi_off = 1;
    esp_now_off = 1;
    esp_wifi_stop();
    ESP_LOGI(TAG_WIFI, "wifi deinit completed");
}


/**
 * @brief Funcion que apaga ESP_NOW
 * @param [in] void
 * @return void
 */

void deinit_esp_now()
{
    esp_now_off = 1;
    esp_now_deinit();
    ESP_LOGI(TAG_WIFI, "esp now deinit completed");
}
