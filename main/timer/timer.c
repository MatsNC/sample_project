#include "timer.h"

static const char *TAG_TIMER = "TIMER_INFO";

void timer_callback(void *arg)
{
    ESP_LOGI(TAG_TIMER, "Timer callback triggered");
    // if (!gpio_get_level(PIN_SWITCH))
    // {
    //     switch (touch_config_state)
    //     {
    //     case TOUCH_PAD_ATTEN_VOLTAGE1:
    //         touch_pad_set_voltage(TOUCH_HVOLT_2V4, TOUCH_LVOLT_0V8, TOUCH_HVOLT_ATTEN_1V5);      //el cambio de sensibilidad tiene que disparar una recalibracion, caso contrario no funciona bien
    //         ESP_LOGI(TAG_TIMER, "Cambio de configuracion touch TOUCH_HVOLT_ATTEN_1V5");
    //         touch_config_state = TOUCH_PAD_ATTEN_VOLTAGE2;
    //         break;
    //     case TOUCH_PAD_ATTEN_VOLTAGE2:
    //         touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_0V5);
    //         ESP_LOGI(TAG_TIMER, "Cambio de configuracion touch TOUCH_HVOLT_ATTEN_0V5");
    //         touch_config_state = TOUCH_PAD_ATTEN_VOLTAGE1;
    //         break;
    //     }
    // }
}

/**
 * @brief Set one timer and start once setting periodic value as the period 
 * 
 * @param periodic_value time value for the timer
 */

void set_timer(uint64_t periodic_value)
{
    const esp_timer_create_args_t timer_args = {
        .callback = &timer_callback,
        .name = "main_timer"};

    esp_timer_handle_t timer_handle;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &timer_handle));

    // Start the timer to trigger every periodic_value (in microseconds)
    ESP_ERROR_CHECK(esp_timer_start_once(timer_handle, periodic_value));

    ESP_LOGI(TAG_TIMER, "Timer started, will trigger every periodic_value");
}