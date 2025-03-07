#include "wifi.h"

void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        // Configura el canal de Wi-Fi ahora que Wi-Fi ha comenzado
        ESP_ERROR_CHECK(esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE));
        esp_wifi_connect();
        ESP_LOGI(TAG_WIFI, "Wi-Fi STA started, attempting to connect...");
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        //ESP_LOGI(TAG_WIFI, "Disconnected from Wi-Fi, retrying...");
        esp_wifi_connect();
    }
    // else if (event_id == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED)
    // {
    //     // Configura el canal de Wi-Fi ahora que Wi-Fi ha comenzado
    //     ESP_ERROR_CHECK(esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE));
    //     ESP_LOGI(TAG_WIFI, "Wi-Fi initialized in STA mode, set to channel %d", CHANNEL);
    // }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG_WIFI, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG_WIFI, "Netmask: " IPSTR, IP2STR(&event->ip_info.netmask));
        ESP_LOGI(TAG_WIFI, "Gateway: " IPSTR, IP2STR(&event->ip_info.gw));
    }
}

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
    ESP_ERROR_CHECK(esp_wifi_set_channel(CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_LOGI(TAG_WIFI, "Wi-Fi initialized in STA mode, set to channel %d", CHANNEL);
    wifi_off = 0;
    ESP_LOGI(TAG_WIFI, "wifi init completed");
    return ESP_OK;
}

esp_err_t init_wifi_webserver()
{
    ESP_ERROR_CHECK(nvs_flash_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip);

    // STA mode initialization
    ESP_ERROR_CHECK(esp_netif_init());

    // Wi-Fi Interface creation
    esp_netif_t *netif = esp_netif_create_default_wifi_sta();
    assert(netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG_WIFI, "Wi-Fi initialized in STA mode, waiting for connection...");
    wifi_off = 0;
    return ESP_OK;
}

/**
 * @brief HTTP GET handler for serving the touch data.
 *
 * This function sends an HTML page displaying the latest touch values.
 * It includes a JavaScript snippet to refresh the page every 5 seconds.
 *
 * @param req Pointer to the HTTP request structure.
 * @return esp_err_t Error status of the response.
 */

esp_err_t get_touch_data_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");

    int ret = snprintf(touch_val, sizeof(touch_val),
                       "Received touch values:<br>Umbral fria +: %lu<br>Umbral Caliente +: %lu<br>"
                       "Touch Caliente -: %lu<br>Touch Caliente +: %lu<br>"
                       "Touch Fria -: %lu<br>Touch Fria +: %lu<br><span style='color:green;'>%s</span><br><br>"
                       "<br>Umbral Caliente -: %lu<br>Umbral Fria -: %lu"
                       "<br>Touch_avg caliente -: %lu",
                       touch_data.received_data1, touch_data.received_data2,
                       touch_data.received_data3, touch_data.received_data4,
                       touch_data.received_data5, touch_data.received_data6, touch_status, touch_data.received_data7, touch_data.received_data8, touch_data.received_data9);

    if (ret < 0 || ret >= sizeof(touch_val))
    {
        ESP_LOGE("SNPRINTF", "Buffer size insufficient or snprintf error. Needed size: %d", ret);
    }

    char response[800];
    snprintf(response, sizeof(response),
             "<html>"
             "<head><title>Touch Debug</title></head>"
             "<body>"
             "<h1>Touch Data</h1>"
             "<p>%s</p>"
             "<script>"
             "setInterval(() => {location.reload(); }, 300);"
             "</script>"
             "</body>"
             "</html>",
             touch_val);
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

/**
 * @brief Configures and starts the HTTP web server.
 *
 * This function sets up the server with a handler to serve the touch data.
 */

void start_webserver()
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    ESP_ERROR_CHECK(httpd_start(&server, &config));

    httpd_uri_t touch_data_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = get_touch_data_handler,
        .user_ctx = NULL};
    httpd_register_uri_handler(server, &touch_data_uri);
    ESP_LOGI(TAG_WIFI, "Webserver started \n");
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
