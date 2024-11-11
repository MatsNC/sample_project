#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include <string.h>

//#define ESP_NOW_LOG
#define ESP_NOW

#define WIFI_SSID "FV-IOT"
#define WIFI_PASS "FVVR#iot1980"

//#define CHANNEL 11    //mi celu   
#define CHANNEL 1       //FV-IOT

typedef struct
{
    uint32_t received_data1;
    uint32_t received_data2;
    uint32_t received_data3;
    uint32_t received_data4;
    uint32_t received_data5;
    uint32_t received_data6;
} touch_data_t;

static bool wifi_off = 1;
static bool esp_now_off = 1;
static const char *TAG_WIFI = "ESP_NOW";
static uint8_t peer_mac[ESP_NOW_ETH_ALEN] = {0x7c, 0xdf, 0xa1, 0x61, 0xb8, 0xf8};

extern touch_data_t touch_data;

extern char touch_val[200]; /**< Buffer to store the received touch value as a string. */

esp_err_t init_esp_now(void);
esp_err_t init_wifi(void);
void deinit_wifi(void);
void deinit_esp_now(void);
esp_err_t init_wifi_webserver(void);
void wifi_event_handler(void *, esp_event_base_t, int32_t, void *);
void start_webserver(void);