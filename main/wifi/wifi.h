#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "esp_event.h"
#include <string.h>

//#define ESP_NOW_LOG
#define ESP_NOW

static bool wifi_off = 1;
static bool esp_now_off = 1;
static const char *TAG_WIFI = "ESP_NOW";
static uint8_t peer_mac[ESP_NOW_ETH_ALEN] = {0x7c, 0xdf, 0xa1, 0x61, 0xb8, 0xf8};

esp_err_t init_esp_now(void);
esp_err_t init_wifi(void);
void deinit_wifi(void);
void deinit_esp_now(void);