#include "nvs_flash.h"
#include "esp_log.h"

static const char *TAG_NVS = "NVS_ENCAPSULADO";

esp_err_t init_nvs(void);
esp_err_t save_value_to_nvs(const char*, const char*, uint32_t);
esp_err_t get_value_from_nvs(const char*, const char*, uint32_t*);
esp_err_t save_value_if_not_exists(const char*, const char*, uint32_t);