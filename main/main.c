#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/touch_pad.h"
#include "driver/adc.h"
#include "driver/uart.h"
#include "esp_adc_cal.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "freertos/timers.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "led_strip.h"
#include "driver/touch_sensor.h"

#define ESP_CHANNEL 1
#define LED_STRIP_MAX_LEDS 12

#define CALIB_STAGE_1 1
#define CALIB_STAGE_2 2
#define CALIB_STAGE_3 3
// #define VALV_MODUL 1

#ifdef VALV_MODUL
#define LED_STRIP 18
#else
#define LED_STRIP 38
#endif

#define Touch_Nivel TOUCH_PAD_NUM4
#define Touch_Fuga TOUCH_PAD_NUM5
#define Touch_Caudal_Sube TOUCH_PAD_NUM1
#define Touch_Caudal_Baja TOUCH_PAD_NUM2

#define adc_caliente ADC1_CHANNEL_7
#define adc_fria ADC1_CHANNEL_5
#define adc_natural ADC1_CHANNEL_6
#define adc_pres_ent ADC1_CHANNEL_3
#define adc_pres_sal ADC1_CHANNEL_4

#define Salida_Bomba 10
#define Salida_Compresor 16
#define Salida_Cooler 9
#define Salida_Resistencia 15

#define Salida_EV1 37
#define Salida_EV2 36
#define Salida_EV3 35
#define Salida_EV4 45
#define Salida_EV5 48
#define Salida_EV6 47

#define Salida_Fria_Up 11
#define Salida_Fria_Down 12
#define Salida_Caliente_Up 13
#define Salida_Caliente_Down 14

#define Salida_Electrovalvula_CO2 21

#define Samples 64

#define UART_NUM UART_NUM_1
#define BUF_SIZE 1024
#define TASK_MEMORY 1024 * 4

bool B_Fria_Up = 0;
bool B_Fria_Down = 1;
bool B_Caliente_Up = 0;
bool B_Caliente_Down = 0;
bool B_Bomba = 0;
bool B_Resistencia = 0;
bool B_Compresor = 0;
bool B_Cooler = 0;
bool B_Electrovalvula_CO2 = 0;
bool wifi_off = 1;
bool esp_now_off = 1;
bool first_on = 1;
bool init_calib_stage = false;

#ifndef VALV_MODUL

bool EV1 = 0;
bool EV2 = 0;
bool EV3 = 0;
bool EV4 = 0;
bool EV5 = 0;
bool EV6 = 0;

#endif

float b_perc = 0.5;
float r_perc = 0.5;

float r = 255 * 0.5;
float b = 255 * 0.5;

uint32_t r_int;
uint32_t b_int;

led_strip_handle_t led_strip;

static const char *TAG = "esp_now_resp";
static const char *tag = "Main";
static const char *tag2 = "UART";
TimerHandle_t xTimers;

uint32_t filtered_Nivel;
uint32_t filtered_Fuga;
uint32_t filtered_Caudal_Up;
uint32_t filtered_Caudal_Down, filtered_Caudal_Down_Touch;
uint32_t filtered_Caudal_Down_Base;
uint32_t filtered_Caudal_Up_Base, filtered_Caudal_Up_Touch;
uint32_t filtered_Nivel_Base;
uint32_t filtered_Fuga_Base;
uint8_t calib = 100;

float caudal = 0.6;

int interval = 100;
int timerId = 1;
int count_touch_read = 10;
int calib_stage = 1;
uint32_t count_mot_off = 100;

TimeOut_t xTimeOut;
TickType_t xTicksToWait;
TickType_t xInitialTimeout = pdMS_TO_TICKS(3000);

TimeOut_t xTimeOut2;
TickType_t xTicksToWait2;
TickType_t xInitialTimeout2 = pdMS_TO_TICKS(1000);

int contador = 0;
int demora = 0;

int prom_caliente = 0;
// int smooth_val1 = 0;
int prom_fria = 0;
int prom_natural = 0;
int prom_pres_ent = 0;
int prom_pres_sal = 0;

int pag = 0;
int count = 0;

float Caliente;
// float Condensador;
float Fria;
float Natural;
float Pres_Ent;
float Pres_Sal;

static esp_adc_cal_characteristics_t adc1_chars;

void inicio_hw(void);
static void init_uart(void);
void touch_read(void);
void adc_read(void);
void set_timer(void);
void set_pwm_duty(void);
static void UART_task(void *pvParameters);

void out_relay(void);

void vTimerCallback(TimerHandle_t pxTimer)
{
}

// Funciones de ESP_NOW y LED strip

void deinit_esp_now(void)
{
    esp_now_off = 1;
    esp_now_deinit();
    ESP_LOGI(TAG, "esp now deinit completed");
}

void deinit_wifi(void)
{
    wifi_off = 1;
    esp_now_off = 1;
    esp_wifi_stop();
    ESP_LOGI(TAG, "wifi deinit completed");
}

static esp_err_t init_wifi(void)
{
    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    esp_netif_init();
    esp_event_loop_create_default();
    nvs_flash_init();
    esp_wifi_init(&wifi_init_config);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_storage(WIFI_STORAGE_FLASH);
    esp_wifi_start();
    wifi_off = 0;
    ESP_LOGI(TAG, "wifi init completed");
    return ESP_OK;
}

static esp_err_t init_led_strip(void)
{
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_STRIP,              // The GPIO that connected to the LED strip's data line
        .max_leds = LED_STRIP_MAX_LEDS,           // The number of LEDs in the strip,
        .led_pixel_format = LED_PIXEL_FORMAT_GRB, // Pixel format of your LED strip
        .led_model = LED_MODEL_WS2812,            // LED strip model
        .flags.invert_out = false,                // whether to invert the output signal (useful when your hardware has a level inverter)
    };

    led_strip_rmt_config_t rmt_config = {
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
        .rmt_channel = 0,
#else
        .clk_src = RMT_CLK_SRC_DEFAULT,    // different clock source can lead to different power consumption
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,           // whether to enable the DMA feature
#endif
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));

    return ESP_OK;
}

void send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (status == ESP_NOW_SEND_SUCCESS)
    {
        ESP_LOGI(TAG, "ESP_NOW_SEND_SUCCESS");
    }
    else
    {
        ESP_LOGW(TAG, "ESP_NOW_SEND_FAIL");
    }
}

void recv_cb(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len)
{
    uint8_t out_type;
    out_type = atoi((char *)data);
    ESP_LOGI(TAG, "Data received: " MACSTR " %s", MAC2STR(esp_now_info->src_addr), data);
    printf("%d\n", out_type);
    switch (out_type)
    {
    case 0:
        B_Caliente_Up = 1;
        B_Fria_Down = 0;
        B_Fria_Up = 0;
        B_Caliente_Down = 0;
        printf("Agua Natural\n");
        break;
    case 1:
        B_Fria_Up = 1;
        B_Fria_Down = 0;
        B_Caliente_Down = 0;
        B_Caliente_Up = 0;
        printf("Agua Fria\n");
        break;
    case 2:
        B_Caliente_Down = 1;
        B_Fria_Down = 0;
        B_Fria_Up = 0;
        B_Caliente_Up = 0;
        printf("Agua Gasificada\n");
        break;
    }
}

static esp_err_t init_esp_now(void)
{
    esp_now_init();
    esp_now_register_recv_cb(recv_cb);
    esp_now_register_send_cb(send_cb);
    esp_now_off = 0;
    ESP_LOGI(TAG, "esp now init completed");
    return ESP_OK;
}

void app_main()
{
    inicio_hw();
    init_uart();
    set_timer();

    ESP_ERROR_CHECK(init_wifi());
    ESP_ERROR_CHECK(init_esp_now());
    ESP_ERROR_CHECK(init_led_strip());

    while (1)
    {
        touch_read();
        // if (first_on)
        // {
        //     r_int = 0;
        //     b_int = 0;
        // }
        // else
        // {
        //     r_int = (uint32_t)r;
        //     b_int = (uint32_t)b;
        // }
        if (calib_stage == CALIB_STAGE_3)
        {
            for (int i = 0; i < LED_STRIP_MAX_LEDS; i++)
            {
                ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, r_int, 0, b_int));
            }
            led_strip_refresh(led_strip);
        }
        adc_read();
        out_relay();
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void inicio_hw(void)
{

    gpio_config_t io_config;

    io_config.mode = GPIO_MODE_OUTPUT;
    // io_config.pin_bit_mask = ((1 << Led_R) | (1 << Led_G) | (1 << Led_B) | (1 << Salida_Bomba) | (1 << Salida_Compresor) | (1 << Salida_Cooler) | (1 << Salida_Resistencia) | (1 <<Salida_Fria_Up) | (1 << Salida_Fria_Down) | (1 << Salida_Caliente_Up) | (1 << Salida_Caliente_Down));
    io_config.pin_bit_mask = ((1 << Salida_Bomba) | (1 << Salida_Compresor) | (1 << Salida_Cooler) | (1 << Salida_Resistencia) | (1 << Salida_Fria_Up) | (1 << Salida_Fria_Down) | (1 << Salida_Caliente_Up) | (1 << Salida_Caliente_Down) | (1 << Salida_Electrovalvula_CO2));
    io_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_config.pull_up_en = GPIO_PULLDOWN_DISABLE;
    io_config.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_config);

    gpio_set_direction(Salida_EV4, GPIO_MODE_DEF_OUTPUT);
    gpio_set_direction(Salida_EV5, GPIO_MODE_DEF_OUTPUT);
    gpio_set_direction(Salida_EV6, GPIO_MODE_DEF_OUTPUT);

    touch_pad_init();
    // touch_pad_set_voltage(TOUCH_HVOLT_2V4, TOUCH_LVOLT_0V8, TOUCH_HVOLT_ATTEN_1V5);
    touch_pad_set_voltage(TOUCH_HVOLT_2V4, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V); // Chat GPT
    touch_pad_config(Touch_Nivel);
    touch_pad_config(Touch_Fuga);
    touch_pad_config(Touch_Caudal_Sube);
    touch_pad_config(Touch_Caudal_Baja);
    // touch_pad_config(Touch_Fria, -1);
    // touch_pad_config(Touch_Caliente, -1);
    // touch_pad_config(Touch_Gasificada, -1);
    // touch_pad_config(Touch_Natural, -1);
    // touch_pad_filter_start(10);
    /* Denoise setting at TouchSensor 0. */
    touch_pad_denoise_t denoise = {
        /* The bits to be cancelled are determined according to the noise level. */
        .grade = TOUCH_PAD_DENOISE_BIT4,
        .cap_level = TOUCH_PAD_DENOISE_CAP_L4,
    };
    // se agrega ahora
    touch_filter_config_t filter_info = {
        .mode = TOUCH_PAD_FILTER_IIR_16,
        .debounce_cnt = 1,
        .noise_thr = 0,
        .jitter_step = 4,
        .smh_lvl = TOUCH_PAD_SMOOTH_IIR_2};

    touch_pad_denoise_set_config(&denoise);
    touch_pad_denoise_enable();

    // ESP_LOGI(TAG, "Denoise function init");

    /* Enable touch sensor clock. Work mode is "timer trigger". */
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
    touch_pad_fsm_start();

    // se agrega ahora
    touch_pad_filter_set_config(&filter_info);
    touch_pad_filter_enable();

    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_DEFAULT, 0, &adc1_chars);

    adc1_config_width(ADC_WIDTH_BIT_DEFAULT);
    adc1_config_channel_atten(adc_caliente, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(adc_fria, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(adc_natural, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(adc_pres_ent, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(adc_pres_sal, ADC_ATTEN_DB_11);

    // printf("inicio_hw\n");
}

static void init_uart(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_param_config(UART_NUM, &uart_config);

    uart_set_pin(UART_NUM, 17, 18, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uart_driver_install(UART_NUM, BUF_SIZE, BUF_SIZE, 0, NULL, 0);
    xTaskCreate(UART_task, "uart_task", TASK_MEMORY, NULL, 5, NULL);

    ESP_LOGI(tag2, "init uart completed!");
}

void touch_read(void)
{
    // touch_pad_read_filtered(Touch_Nivel, &filtered_Nivel);
    // printf("------- Nivel ---------\n");
    touch_pad_read_raw_data(Touch_Nivel, &filtered_Nivel);
    // printf("touch 2 = %ld\n ", filtered_Nivel);
    touch_pad_read_raw_data(Touch_Fuga, &filtered_Fuga);
    // printf("touch 4 = %ld\n ", filtered_Fuga);
    touch_pad_read_raw_data(Touch_Caudal_Baja, &filtered_Caudal_Down);
    // printf("touch 1 = %ld\n ", filtered_Caudal_Down);
    touch_pad_read_raw_data(Touch_Caudal_Sube, &filtered_Caudal_Up);
    // printf("touch 3 = %ld\n ", filtered_Caudal_Up);
    //  touch_pad_read_filtered(Touch_Gasificada, &filtered_Gasificada);
    //  touch_pad_read_filtered(Touch_Natural, &filtered_Natural);
    //   printf("Nivel = %d Fria = %d Caliente = %d Gasificada = %d Natural = %d\n", filtered_Nivel, filtered_Fria, filtered_Caliente, filtered_Gasificada, filtered_Natural);

    if (calib_stage == CALIB_STAGE_3)
    {
        if (count_touch_read > 0)
        {
            count_touch_read--;
        }
        else
        {
            count_touch_read = 10;
            printf("------Nivel-------\n");
            printf("touch 1 = %ld\n ", filtered_Caudal_Down);
            printf("touch 1 base = %ld\n ", filtered_Caudal_Down_Touch);
            printf("touch 2 = %ld\n ", filtered_Nivel);
            printf("touch 3 = %ld\n ", filtered_Caudal_Up);
            printf("touch 3 base = %ld\n ", filtered_Caudal_Up_Touch);
            printf("touch 4 = %ld\n ", filtered_Fuga);
        }
    }

    // aca hacer una maquina de estados para la calibracion con toque
    switch (calib_stage)
    {
    case CALIB_STAGE_1:
        if (!init_calib_stage)
        {
            xTicksToWait = xInitialTimeout;
            vTaskSetTimeOutState(&xTimeOut);
            init_calib_stage = true;
        }
        for (int i = 0; i < LED_STRIP_MAX_LEDS; i++)
        {
            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 0, 0, 0));
        }
        led_strip_refresh(led_strip);

        filtered_Caudal_Down_Base = filtered_Caudal_Down;
        printf("touch 1 base = %ld\n ", filtered_Caudal_Down_Base);
        filtered_Caudal_Up_Base = filtered_Caudal_Up;
        printf("touch 3 base = %ld\n ", filtered_Caudal_Up_Base);
        filtered_Nivel_Base = filtered_Nivel;
        filtered_Fuga_Base = filtered_Fuga;

        if (xTaskCheckForTimeOut(&xTimeOut, &xTicksToWait) != pdFALSE)
        {
            xTicksToWait = xInitialTimeout;
            vTaskSetTimeOutState(&xTimeOut);
            calib_stage = CALIB_STAGE_2;
            printf("Calib stage 1 a 2\n");
        }
        break;

    case CALIB_STAGE_2:
        for (int i = 0; i < LED_STRIP_MAX_LEDS; i++)
        {
            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 255, 255, 255));
        }
        led_strip_refresh(led_strip);

        if (filtered_Caudal_Down > 1.5 * filtered_Caudal_Down_Base)
        {
            filtered_Caudal_Down_Touch = filtered_Caudal_Down;
            printf("touch 1 base = %ld\n ", filtered_Caudal_Down_Touch);
        }

        if (filtered_Caudal_Up > 1.5 * filtered_Caudal_Up_Base)
        {
            filtered_Caudal_Up_Touch = filtered_Caudal_Up;
            printf("touch 3 base = %ld\n ", filtered_Caudal_Up_Touch);
        }

        // vTaskDelay(1000 / portTICK_PERIOD_MS);
        if (xTaskCheckForTimeOut(&xTimeOut, &xTicksToWait) != pdFALSE)
        {
            printf("Calib stage 2 a 3\n");
            calib_stage = CALIB_STAGE_3;
            for (int i = 0; i < LED_STRIP_MAX_LEDS; i++)
            {
                ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 255, 255, 255));
            }
            led_strip_refresh(led_strip);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            for (int i = 0; i < LED_STRIP_MAX_LEDS; i++)
            {
                ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 0, 0, 0));
            }
            led_strip_refresh(led_strip);
        }
        break;
    case CALIB_STAGE_3:
        break;
    }

    if ((filtered_Nivel > (filtered_Nivel_Base * 1.5 /*1.03*/)) && (filtered_Fuga < (filtered_Fuga_Base * 1.5 /*1.03*/)))
    {
        if (b_perc < 1 && r_perc > 0)
        {
            b_perc += 0.1;
            r_perc -= 0.1;
            r = r_perc * 255;
            b = b_perc * 255;
            r_int = (uint32_t)(r);
            b_int = (uint32_t)(b);
        }

#ifdef VALV_MODUL
        B_Caliente_Down = 0;
        B_Caliente_Up = 1;
#endif
    }

    if ((filtered_Fuga > (filtered_Fuga_Base * 1.5 /*1.03*/)) && (filtered_Nivel < (filtered_Nivel_Base * 1.5 /*1.03*/)))
    {
        if (r_perc < 1 && b_perc > 0)
        {
            r_perc += 0.1;
            b_perc -= 0.1;
            r = r_perc * 255;
            b = b_perc * 255;
            r_int = (uint32_t)(r);
            b_int = (uint32_t)(b);
        }

#ifdef VALV_MODUL
        B_Caliente_Down = 1;
        B_Caliente_Up = 0;
#endif
    }

    if ((filtered_Fuga < (filtered_Fuga_Base * 1.5 /*1.03*/)) && (filtered_Nivel < (filtered_Nivel_Base * 1.5 /*1.03*/)))
    {
        r_int = (uint32_t)r;
        b_int = (uint32_t)b;
        B_Caliente_Up = 1;
        B_Caliente_Down = 1;
    }
    if ((filtered_Caudal_Up > (filtered_Caudal_Up_Touch * 0.8 /*1.03*/)) && (filtered_Caudal_Up > filtered_Caudal_Down))
    {
        
        // printf("touch 1 = %ld\n ", filtered_Caudal_Up);
#ifdef VALV_MODUL
        if (B_Caliente_Down && B_Caliente_Up)
        {
            B_Fria_Up = 0;
        }
#endif
        // if (caudal < 1)
        // {
        //     caudal += 0.1;
        // }

        if (b_perc < 1 && r_perc > 0)
        {
            b_perc += 0.1;
            r_perc -= 0.1;
            r = r_perc * 255;
            b = b_perc * 255;
            r_int = (uint32_t)(r);
            b_int = (uint32_t)(b);
        }
    }
    else
    {
        B_Fria_Up = 1;
    }
    if ((filtered_Caudal_Down > filtered_Caudal_Down_Touch * 0.8) /*1.03*/ && (filtered_Caudal_Down > filtered_Caudal_Up))
    {
        // printf("touch 2 = %ld\n ", filtered_Caudal_Down);
#ifdef VALV_MODUL
        if (B_Caliente_Down && B_Caliente_Up)
        {
            B_Fria_Down = 0;
        }
#endif
        // if (caudal > 0.3)
        // {
        //     caudal -= 0.1;
        // }

        if (r_perc < 1 && b_perc > 0)
        {
            r_perc += 0.1;
            b_perc -= 0.1;
            r = r_perc * 255;
            b = b_perc * 255;
            r_int = (uint32_t)(r);
            b_int = (uint32_t)(b);
        }
    }
    else
    {
        B_Fria_Down = 1;
    }

    // r_int = (uint32_t)(r * caudal);
    // b_int = (uint32_t)(b * caudal);

#ifndef VALV_MODUL
    if (b_perc >= 0 && b_perc < 0.1)
    {
        EV4 = 1;
        EV5 = 0;
        EV6 = 0;
        // gpio_set_level(Salida_EV4, 1);
        // printf("Output_45=%d\n",gpio_get_level(Salida_EV4));
    }
    if (b_perc >= 0.1 && b_perc < 0.2)
    {
        EV4 = 0;
        EV5 = 1;
        EV6 = 0;
    }
    if (b_perc >= 0.2 && b_perc < 0.3)
    {
        EV4 = 1;
        EV5 = 1;
        EV6 = 0;
    }
    if (b_perc >= 0.3 && b_perc < 0.4)
    {
        EV4 = 0;
        EV5 = 0;
        EV6 = 1;
    }
    if (b_perc >= 0.4 && b_perc < 0.5)
    {
        EV4 = 1;
        EV5 = 0;
        EV6 = 1;
    }
    if (b_perc >= 0.5 && b_perc < 0.6)
    {
        EV4 = 0;
        EV5 = 1;
        EV6 = 1;
    }
    if (b_perc >= 0.6 && b_perc < 0.7)
    {
        EV4 = 1;
        EV5 = 1;
        EV6 = 1;
    }

#endif
}

void adc_read(void)
{
    for (int i = 0; i < Samples; i++)
    {
        prom_caliente += adc1_get_raw(adc_caliente);
        prom_fria += adc1_get_raw(adc_fria);
        prom_natural += adc1_get_raw(adc_natural);
        prom_pres_ent += adc1_get_raw(adc_pres_ent);
        prom_pres_sal += adc1_get_raw(adc_pres_sal);
    }

    prom_caliente /= Samples;
    prom_fria /= Samples;
    prom_natural /= Samples;
    prom_pres_ent /= Samples;
    prom_pres_sal /= Samples;

    if (prom_caliente > 4095)
    {
        prom_caliente = 4095;
    }
    if (prom_fria > 4095)
    {
        prom_fria = 4095;
    }
    if (prom_natural > 4095)
    {
        prom_natural = 4095;
    }
    if (prom_pres_ent > 4095)
    {
        prom_pres_ent = 4095;
    }
    if (prom_pres_sal > 4095)
    {
        prom_pres_sal = 4095;
    }

    Caliente = 110 + prom_caliente * 0.795;
    Fria = 110 + prom_fria * 0.795;
    Natural = 110 + prom_natural * 0.795;
    Pres_Ent = 110 + prom_pres_ent * 0.795;
    Pres_Sal = 110 + prom_pres_sal * 0.795;
}

void set_timer(void)
{
    ESP_LOGI(tag, "Timer init configuration");
    xTimers = xTimerCreate("Timer",                   // Just a text name, not used by the kernel.
                           (pdMS_TO_TICKS(interval)), // The timer period in ticks.
                           pdTRUE,                    // The timers will auto-reload themselves when they expire.
                           (void *)timerId,           // Assign each timer a unique id equal to its array index.
                           vTimerCallback             // Each timer calls the same callback when it expires.
    );

    if (xTimers == NULL)
    {
        // The timer was not created.
        ESP_LOGE(tag, "The timer was not created.");
    }
    else
    {
        if (xTimerStart(xTimers, 0) != pdPASS)
        {
            // The timer could not be set into the Active state.
            ESP_LOGE(tag, "The timer could not be set into the Active state.");
        }
    }
}

void set_pwm_duty(void)
{
}

static void UART_task(void *pvParameters)
{
    uint8_t *data = (uint8_t *)malloc(BUF_SIZE);
    uint8_t *data2 = (uint8_t *)malloc(BUF_SIZE);
    char tx_data[] = {"\x5a\xa5\x04\x83\x30\x04\x01"};      // leer 3004
    char tx2_data[] = {"\x5a\xa5\x05\x82\x30\x00\x00\x00"}; // escribir 00 en 3000 (3000 en 1 activa protector de pantalla)
    int tx_len = 7;
    int tx2_len = 8;
    int cont_caliente = 0;
    int valvula_caliente = 0;
    int pagina = 0;

    while (1)
    {
        uart_write_bytes(UART_NUM, tx_data, tx_len);
        // uart_write_bytes(UART_NUM, tx2_data, tx2_len);

        bzero(data, BUF_SIZE);

        int len = uart_read_bytes(UART_NUM, data, BUF_SIZE, pdMS_TO_TICKS(100));
        if (len == 0)
        {
            continue;
        }

        if (data[8] == 4) // si se activo protector de pantalla (pagina = 4)
        {
            tx2_data[7] = 0;
        }

        if (data[8] != 4)
        {
            count++;
        }

        if (pag != data[8])
        {
            pag = data[8];
            count = 0;
        }
        if (count >= 50) // Tiempo de espera 5 = 1 segundo
        {
            tx2_data[7] = 1; // protector de pantalla
            count = 0;
            // Apagar wifi y ESP_NOW
            // deinit_esp_now();
            deinit_wifi();
        }

        // tx2_data[7] = data[8];
        uart_write_bytes(UART_NUM, tx2_data, tx2_len);
        uart_read_bytes(UART_NUM, data2, BUF_SIZE, pdMS_TO_TICKS(100)); // lee respuesta del envio anterior, no se utiliza data2.

        switch (data[8])
        {

        case 0:

            if (valvula_caliente == 0)
            {
                if ((cont_caliente >= 2) && (cont_caliente <= 5))
                {
                    valvula_caliente = 1;
                }
                cont_caliente = 0;
                B_Caliente_Up = 0;
            }

            if (valvula_caliente == 1)
            {
                if (cont_caliente <= 5)
                {
                    cont_caliente++;
                }

                else
                {
                    valvula_caliente = 0;
                    cont_caliente = 0;
                }
                B_Caliente_Up = 0;
            }

            if (valvula_caliente == 2)
            {
                valvula_caliente = 3;
                cont_caliente = 0;
                B_Caliente_Up = 1;
            }

            if (valvula_caliente == 3)
            {
                if (cont_caliente <= 1)
                {
                    cont_caliente++;
                    B_Caliente_Up = 1;
                }
                else
                {
                    valvula_caliente = 0;
                    cont_caliente = 0;
                    B_Caliente_Up = 0;
                }
            }

            pagina = 0;
            B_Fria_Down = 0;
            B_Fria_Up = 0;
            B_Caliente_Down = 0;
            break;

        case 1:
            pagina = 1;
            B_Fria_Down = 0;
            B_Fria_Up = 0;
            B_Caliente_Down = 0;
            B_Caliente_Up = 0;
            break;

        case 2:
            pagina = 2;
            B_Fria_Down = 0;
            B_Fria_Up = 0;
            B_Caliente_Down = 0;
            B_Caliente_Up = 0;
            break;

        case 3:

            pagina = 3;
            B_Fria_Down = 0;
            B_Fria_Up = 0;
            B_Caliente_Down = 0;
            B_Caliente_Up = 0;
            break;

        case 4:
            pagina = 6;
            B_Fria_Down = 0;
            B_Fria_Up = 0;
            B_Caliente_Down = 0;
            B_Caliente_Up = 0;
            break;

        case 10:
            if (pagina == 0)
            {
                if (valvula_caliente == 0)
                {
                    if (cont_caliente <= 5)
                    {
                        cont_caliente++;
                    }
                }

                if (valvula_caliente == 1)
                {
                    if ((cont_caliente >= 2) && (cont_caliente <= 5))
                    {
                        valvula_caliente = 2;
                    }
                    else
                    {
                        valvula_caliente = 0;
                    }
                    cont_caliente = 0;
                }

                if (valvula_caliente == 2)
                {
                    printf("Agua Caliente\n");
                    B_Fria_Down = 1;
                    B_Fria_Up = 0;
                    B_Caliente_Down = 0;
                    B_Caliente_Up = 0;
                    cont_caliente = 0;
                }
            }
            else
            {
                pagina = 6;
                B_Fria_Down = 0;
                B_Fria_Up = 0;
                B_Caliente_Down = 0;
                B_Caliente_Up = 0;
            }
            break;

        case 11:
            if (pagina == 1)
            {

                printf("Agua Gasificada\n");
                B_Fria_Down = 0;
                B_Fria_Up = 0;
                B_Caliente_Down = 1;
                B_Caliente_Up = 0;
            }
            else
            {
                pagina = 6;
                B_Fria_Down = 0;
                B_Fria_Up = 0;
                B_Caliente_Down = 0;
                B_Caliente_Up = 0;
            }
            break;

        case 12:
            if (pagina == 2)
            {

                printf("Agua Natural\n");
                B_Fria_Down = 0;
                B_Fria_Up = 0;
                B_Caliente_Down = 0;
                B_Caliente_Up = 1;
            }
            else
            {
                pagina = 6;
                B_Fria_Down = 0;
                B_Fria_Up = 0;
                B_Caliente_Down = 0;
                B_Caliente_Up = 0;
            }
            break;

        case 13:
            if (pagina == 3)
            {

                printf("Agua Fria\n");
                B_Fria_Down = 0;
                B_Fria_Up = 1;
                B_Caliente_Down = 0;
                B_Caliente_Up = 0;
            }
            else
            {
                pagina = 6;
                B_Fria_Down = 0;
                B_Fria_Up = 0;
                B_Caliente_Down = 0;
                B_Caliente_Up = 0;
            }
            break;

        default:
            valvula_caliente = 0;
            cont_caliente = 0;
            pagina = 6;
            break;
        }
        //}
    }
}

void out_relay(void)
{
    gpio_set_level(Salida_Fria_Up, !B_Fria_Up);
    gpio_set_level(Salida_Fria_Down, !B_Fria_Down);
    gpio_set_level(Salida_Caliente_Down, !B_Caliente_Down);
    gpio_set_level(Salida_Caliente_Up, !B_Caliente_Up);
// gpio_set_level(Salida_Bomba, !B_Bomba);
#ifdef VALV_MODUL
    gpio_set_level(Salida_Compresor, !B_Compresor);
    gpio_set_level(Salida_Cooler, !B_Cooler);
#else
    // gpio_set_level(Salida_EV1, EV1);
    // gpio_set_level(Salida_EV2, EV2);
    // gpio_set_level(Salida_EV3, EV3);
    gpio_set_level(Salida_EV4, EV4);
    gpio_set_level(Salida_EV5, EV5);
    gpio_set_level(Salida_EV6, EV6);
#endif
    // gpio_set_level(Salida_Resistencia, !B_Resistencia);
    // if (first_on && count_mot_off > 0)
    // {
    //     gpio_set_level(Salida_Fria_Down, 1);
    //     gpio_set_level(Salida_Caliente_Down, 1);
    //     count_mot_off--;
    // }
    // else
    // {
    //     first_on = 0;
    //     gpio_set_level(Salida_Fria_Down, !B_Fria_Down);
    //     gpio_set_level(Salida_Caliente_Down, !B_Caliente_Down);
    // }
}