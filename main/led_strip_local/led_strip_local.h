#include "../managed_components/espressif__led_strip/include/led_strip.h"
#include "esp_err.h"

#define LED_STRIP_MAX_LEDS 12

#define LED_STRIP 38

extern led_strip_handle_t led_strip; 

esp_err_t init_led_strip(void);