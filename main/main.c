/**
 * @brief branch v1.1
   -Se agrega:
   -Apagado/Encendido del equipo con entrada capacitiva (distinta a los volantes). OK.
   -Detección de toque por pulso (si está entre 50 y 1000mS). Si detecta toque setea el umbral. Caso contrario no (bloqueo por agua).

   branch barrido_entradas_pulso
   -Barrido de entradas usando while, arrays y switch case
   -Reset de los capacitivos cuando se quedan bloqueados (Se prueba reset de esp32 primero). EN PROCESO
   -Se agrega guardado en memoria de calibracion.
   -Se agrega cambio con pulsador BOOT
   -Se agregan modulos funcionales
 *
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/touch_pad.h"
#include "driver/adc.h"
#include "driver/uart.h"
#include "esp_adc_cal.h"
#include "driver/ledc.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "led_strip.h"
#include "driver/touch_sensor.h"
#include "esp_timer.h"
#include "nvs/nvs.h"
#include "timer/timer.h"
#include "wifi/wifi.h"
#include "i2c/i2c.h"

#define ESP_CHANNEL 1
#define LED_STRIP_MAX_LEDS 12

#define ANSI_COLOR_RED "\x1b[31m"
#define ANSI_COLOR_GREEN "\x1b[32m"
#define ANSI_COLOR_YELLOW "\x1b[33m"
#define ANSI_COLOR_BLUE "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_RESET "\x1b[0m"

// #define MARMOL_TEST

#define CALIB_STAGE_1 1
#define CALIB_STAGE_2 2
#define CALIB_STAGE_3 3
// #define VALV_MODUL 1

#ifdef VALV_MODUL
#define LED_STRIP 18
#else
#define LED_STRIP 38
#endif

#define Touch_Nivel TOUCH_PAD_NUM5
#define Touch_Fuga TOUCH_PAD_NUM4
#define Touch_Caudal_Sube TOUCH_PAD_NUM2
#define Touch_Caudal_Baja TOUCH_PAD_NUM1
#define Touch_ON TOUCH_PAD_NUM7
// #define adc_caliente ADC1_CHANNEL_7
#define adc_fria ADC1_CHANNEL_5
#define adc_natural ADC1_CHANNEL_6
#define adc_pres_ent ADC1_CHANNEL_3
#define adc_pres_sal ADC1_CHANNEL_4

#define Salida_Bomba 10
#define Salida_Compresor 16
#define Salida_Cooler 9
#define Salida_Resistencia 15
#define PIN_SWITCH 0

#define Salida_EV1 42
#define Salida_EV2 41
#define Salida_EV3 21
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

typedef enum
{
    UNPRESSED,
    PRESSED
} gpio_state_t;

typedef enum
{
    WAIT_FOR_TOUCH,
    TOUCH_DETECTED,
    VALIDATE_PULSE,
} touch_state_t;

typedef enum
{
    TOUCH_PAD_ATTEN_VOLTAGE1,
    TOUCH_PAD_ATTEN_VOLTAGE2,
} touch_config_state_t;

typedef enum
{
    WAIT_TOUCH_ON,
    TOUCH_ON_PRESSED,
    TOUCH_ON_RELEASED,
} touch_on_state_t;

//-----------Definicion de variables----------------

bool B_Fria_Up = 0;
bool B_Fria_Down = 1;
bool B_Caliente_Up = 0;
bool B_Caliente_Down = 0;
bool B_Bomba = 0;
bool B_Resistencia = 0;
bool B_Compresor = 0;
bool B_Cooler = 0;
bool B_Electrovalvula_CO2 = 0;
bool first_on; // cambiar por 1 despues
bool init_calib_stage = false;
bool once = 0;
bool set_dir = 1;
gpio_state_t press_state = UNPRESSED;

#ifndef VALV_MODUL

bool EV1 = 0;
bool EV2 = 0;
bool EV3 = 0;
bool EV4 = 0;
bool EV5 = 0;
bool EV6 = 0;

#endif

static touch_state_t touch_state = WAIT_FOR_TOUCH;
static touch_config_state_t touch_config_state = TOUCH_PAD_ATTEN_VOLTAGE1;

touch_on_state_t touch_on_state;

static int64_t touch_start_time = 0;
static int64_t touch_duration = 0;
static const int64_t MIN_PULSE_DURATION_MS = 50;   // Tiempo mínimo del toque
static const int64_t MAX_PULSE_DURATION_MS = 1000; // Tiempo máximo del toque

float b_perc = 0.3;
float r_perc = 0.3;
float b_perc_ev = 0.3;
float r_perc_ev = 0.3;

float r = 255 * 0.5;
float b = 255 * 0.5;

uint32_t r_int;
uint32_t b_int;

led_strip_handle_t led_strip;
uint64_t period = 10 * 100000;

static const char *tag2 = "UART";
static const char *TAG3 = "esp_touch_INFO";
static const char *TAG_GPIO = "GPIO_STATE_INFO";
TimerHandle_t xTimers;

//-----------variables IN capacitivas----------------
// valores instantaneos:
uint32_t filtered_Nivel;
uint32_t filtered_Fuga;
uint32_t filtered_Caudal_Up;
uint32_t filtered_Caudal_Down;
uint32_t filtered_Touch_ON;

// valores base:
uint32_t filtered_Caudal_Down_Base;
uint32_t filtered_Caudal_Up_Base;
uint32_t filtered_Nivel_Base;
uint32_t filtered_Fuga_Base;
uint32_t filtered_Touch_ON_Base;

// valores touch:
uint32_t filtered_Caudal_Up_Touch;
uint32_t filtered_Caudal_Down_Touch;
uint32_t filtered_Nivel_Base_Touch;
uint32_t filtered_Fuga_Base_Touch;
uint32_t filtered_ON_Touch;

// valores anteriores (para ver si se bloquean las entradas):
uint32_t filtered_Nivel_Ant;
uint32_t filtered_Fuga_Ant;
uint32_t filtered_Caudal_Down_Ant;
uint32_t filtered_Caudal_Up_Ant;

// valor umbral para detectar toque:
// uint32_t filtered_Nivel_Th;
// uint32_t filtered_Fuga_Th;
// uint32_t filtered_Caudal_Down_Th;
// uint32_t filtered_Caudal_Up_Th;

// valores de toque pendientes de validación:
uint32_t filtered_Nivel_Touch_toValidate;
uint32_t filtered_Fuga_Touch_toValidate;
uint32_t filtered_Caudal_Down_Touch_toValidate;
uint32_t filtered_Caudal_Up_Touch_toValidate;

// valores de toque ya validados:
uint32_t filtered_Nivel_Touch_Validated;
uint32_t filtered_Fuga_Touch_Validated;
uint32_t filtered_Caudal_Down_Touch_Validated = 0;
uint32_t filtered_Caudal_Up_Touch_Validated = 0;

//----------------------------------------------------------------

uint8_t calib = 100;
uint8_t on_off_debounce = 2;

float caudal = 0.6;

int interval = 100;
int timerId = 1;
int count_touch_read = 10;
int calib_stage = CALIB_STAGE_1; // CAMBIAR POR CALIB_STAGE_1 DESPUES
int level;
uint32_t count_mot_off = 100;

uint32_t cal_filt_up_nvs;
uint32_t cal_filt_dwn_nvs;
uint32_t cal_filt_nivel_nvs;
uint32_t cal_filt_fuga_nvs;
uint32_t cal_filt_on_nvs;

TimeOut_t xTimeOut;
TickType_t xTicksToWait;
TickType_t xInitialTimeout = pdMS_TO_TICKS(3000);

TimeOut_t xTimeOut2;
TickType_t xTicksToWait2;
TickType_t xInitialTimeout2 = pdMS_TO_TICKS(1000);

int contador = 0;
int demora = 0;

int prom_caliente = 0;
int prom_fria = 0;
int prom_natural = 0;
int prom_pres_ent = 0;
int prom_pres_sal = 0;
int touch_act_detect = 0;

int pag = 0;
int count = 0;
int cont_pin_pressed = 5;

float Caliente;
float Fria;
float Natural;
float Pres_Ent;
float Pres_Sal;

static esp_adc_cal_characteristics_t adc1_chars;

//--------------------Prototipo de funciones--------------------------------
void inicio_hw(void);
static void init_uart(void);
static void i2c_task_func(void);
void touch_read(void);
void adc_read(void);

void set_pwm_duty(void);
static void UART_task(void *pvParameters);
static void I2C_task(void *pvParameters);
void out_relay(void);
int eval_touch_in(void);
void press_proccess(void);

//-----------------------------------------------------------------------------

/**
 * @brief Funcion que evalua cual entrada capacitiva se activó y devuelve el número correspondiente
 * @param [in] void
 * @return int numero alusivo a la entrada capacitiva detectada
 */

int eval_touch_in()
{
    touch_act_detect = 0;
    B_Fria_Down = 1;
    B_Fria_Up = 1; // Setea estas variables en los valores correspondientes a la no detección de toque

    if ((filtered_Caudal_Down > filtered_Caudal_Down_Touch_Validated * 0.5) /*1.03*/ && (filtered_Caudal_Down > filtered_Caudal_Up) && (filtered_Caudal_Down > 2 * filtered_Caudal_Down_Base))
    // if (filtered_Caudal_Down > 1.1 * filtered_Caudal_Down_Base)
    {
        touch_act_detect = 1;
        // printf("touch 1 detection\n");
        ESP_LOGI(TAG3, "touch 1 detection\n");
        filtered_Caudal_Down_Touch_toValidate = filtered_Caudal_Down; // Debo validar si el toque es valido
    }

    if ((filtered_Caudal_Up > filtered_Caudal_Up_Touch_Validated * 0.5) /*1.03*/ && (filtered_Caudal_Up > filtered_Caudal_Down) && (filtered_Caudal_Up > 2 * filtered_Caudal_Up_Base))
    // if (filtered_Caudal_Up > 1.1 * filtered_Caudal_Up_Base)
    {
        touch_act_detect = 3;
        // printf("touch 3 detection\n");
        ESP_LOGI(TAG3, "touch 3 detection\n");
        filtered_Caudal_Up_Touch_toValidate = filtered_Caudal_Up; // Debo validar si el toque es valido
    }

    //==================================== aca va la deteccion para nivel y fuga ==========================//

    if ((filtered_Nivel > filtered_Nivel_Touch_Validated * 0.5) /*1.03*/ && (filtered_Nivel > filtered_Fuga) && (filtered_Nivel > 2 * filtered_Nivel_Base))
    // if (filtered_Nivel > 1.1 * filtered_Nivel_Base)
    {
        touch_act_detect = 2;
        // printf("touch 1 detection\n");
        ESP_LOGI(TAG3, "touch 2 detection\n");
        filtered_Nivel_Touch_toValidate = filtered_Nivel; // Debo validar si el toque es valido
    }

    if ((filtered_Fuga > filtered_Fuga_Touch_Validated * 0.5) /*1.03*/ && (filtered_Fuga > filtered_Nivel) && (filtered_Fuga > 2 * filtered_Fuga_Base))
    // if (filtered_Fuga > 1.1 * filtered_Fuga_Base)
    {
        touch_act_detect = 4;
        // printf("touch 3 detection\n");
        ESP_LOGI(TAG3, "touch 4 detection\n");
        filtered_Fuga_Touch_toValidate = filtered_Fuga; // Debo validar si el toque es valido
    }

    //==============================================================//
    return touch_act_detect;
}

// COMENTAR CON DOXYGEN

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

/**
 * @brief Funcion para calibrar las entradas capacitivas
 *
 */

void calib_cap_inputs(void)
{
    if (filtered_Caudal_Down > 1.1 * filtered_Caudal_Down_Base)
    {
        filtered_Caudal_Down_Touch = filtered_Caudal_Down;
        filtered_Caudal_Down_Touch_Validated = (uint32_t)(0.9 * filtered_Caudal_Down_Touch);
        printf("touch 1 base = %ld\n ", filtered_Caudal_Down_Touch_Validated);
    }

    if (filtered_Caudal_Up > 1.1 * filtered_Caudal_Up_Base)
    {
        filtered_Caudal_Up_Touch = filtered_Caudal_Up;
        filtered_Caudal_Up_Touch_Validated = (uint32_t)(0.9 * filtered_Caudal_Up_Touch);
        printf("touch 3 base = %ld\n ", filtered_Caudal_Up_Touch_Validated);
    }

    if (filtered_Nivel > 1.1 * filtered_Nivel_Base)
    {
        filtered_Nivel_Base_Touch = filtered_Nivel;
        filtered_Nivel_Touch_Validated = (uint32_t)(0.9 * filtered_Nivel_Base_Touch);
        printf("touch 1 base = %ld\n ", filtered_Nivel_Touch_Validated);
    }

    if (filtered_Fuga > 1.1 * filtered_Fuga_Base)
    {
        filtered_Fuga_Base_Touch = filtered_Fuga;
        filtered_Fuga_Touch_Validated = (uint32_t)(0.9 * filtered_Fuga_Base_Touch);
        printf("touch 3 base = %ld\n ", filtered_Fuga_Touch_Validated);
    }
    if (filtered_Touch_ON > 1.5 * filtered_Touch_ON_Base)
    {
        filtered_ON_Touch = (uint32_t)(0.9 * filtered_Touch_ON);
    }
}

void valve_outputs(void)
{

#ifndef VALV_MODUL
    if (!first_on)
    {
        if (b_perc < 0.1)
        {
            EV1 = 0;
            EV2 = 0;
            EV3 = 0;
            // ESP_LOGI("EV", ANSI_COLOR_BLUE "cold off" ANSI_COLOR_RESET "\n");
        }
        // if (b_perc > 0 && b_perc < 0.1)
        // {
        //     EV1 = 1;
        //     EV2 = 0;
        //     EV3 = 0;
        // }
        if (b_perc >= 0.1 && b_perc < 0.2)
        {
            EV1 = 0;
            EV2 = 1;
            EV3 = 0;
            // ESP_LOGI("EV", ANSI_COLOR_BLUE "cold min" ANSI_COLOR_RESET "\n");
        }
        if (b_perc >= 0.2 && b_perc < 0.3)
        {
            EV1 = 1;
            EV2 = 1;
            EV3 = 0;
            // ESP_LOGI("EV", ANSI_COLOR_BLUE "cold 2" ANSI_COLOR_RESET "\n");
        }
        if (b_perc >= 0.3 && b_perc < 0.4)
        {
            EV1 = 0;
            EV2 = 0;
            EV3 = 1;
            // ESP_LOGI("EV", ANSI_COLOR_BLUE "cold 3" ANSI_COLOR_RESET "\n");
        }
        if (b_perc >= 0.4 && b_perc < 0.5)
        {
            EV1 = 1;
            EV2 = 0;
            EV3 = 1;
            // ESP_LOGI("EV", ANSI_COLOR_BLUE "cold 4" ANSI_COLOR_RESET "\n");
        }
        if (b_perc >= 0.5 && b_perc < 0.6)
        {
            EV1 = 0;
            EV2 = 1;
            EV3 = 1;
            // ESP_LOGI("EV", ANSI_COLOR_BLUE "cold 5" ANSI_COLOR_RESET "\n");
        }
        if (b_perc >= 0.6)
        {
            EV1 = 1;
            EV2 = 1;
            EV3 = 1;
            // ESP_LOGI("EV", ANSI_COLOR_BLUE "cold max" ANSI_COLOR_RESET "\n");
        }
        /////////////////////////////////////////
        if (r_perc < 0.1)
        {
            EV4 = 0;
            EV5 = 0;
            EV6 = 0;
            // ESP_LOGI("EV", ANSI_COLOR_RED "hot off" ANSI_COLOR_RESET "\n");
        }
        // if (r_perc > 0 && r_perc < 0.1)
        // {
        //     EV4 = 1;
        //     EV5 = 0;
        //     EV6 = 0;
        // }
        if (r_perc >= 0.1 && r_perc < 0.2)
        {
            EV4 = 0;
            EV5 = 1;
            EV6 = 0;
            // ESP_LOGI("EV", ANSI_COLOR_RED "hot min" ANSI_COLOR_RESET "\n");
        }
        if (r_perc >= 0.2 && r_perc < 0.3)
        {
            EV4 = 1;
            EV5 = 1;
            EV6 = 0;
            // ESP_LOGI("EV", ANSI_COLOR_RED "hot 2" ANSI_COLOR_RESET "\n");
        }
        if (r_perc >= 0.3 && r_perc < 0.4)
        {
            EV4 = 0;
            EV5 = 0;
            EV6 = 1;
            // ESP_LOGI("EV", ANSI_COLOR_RED "hot 3" ANSI_COLOR_RESET "\n");
        }
        if (r_perc >= 0.4 && r_perc < 0.5)
        {
            EV4 = 1;
            EV5 = 0;
            EV6 = 1;
            // ESP_LOGI("EV", ANSI_COLOR_RED "hot 4" ANSI_COLOR_RESET "\n");
        }
        if (r_perc >= 0.5 && r_perc < 0.6)
        {
            EV4 = 0;
            EV5 = 1;
            EV6 = 1;
            // ESP_LOGI("EV", ANSI_COLOR_RED "hot 5" ANSI_COLOR_RESET "\n");
        }
        if (r_perc >= 0.6)
        {
            EV4 = 1;
            EV5 = 1;
            EV6 = 1;
            // ESP_LOGI("EV", ANSI_COLOR_RED "hot max" ANSI_COLOR_RESET "\n");
        }
    }
    else
    {
        EV1 = 0;
        EV2 = 0;
        EV3 = 0;
        EV4 = 0;
        EV5 = 0;
        EV6 = 0;
        // ESP_LOGI("EV", ANSI_COLOR_GREEN "all off" ANSI_COLOR_RESET "\n");
    }
#endif
}

/**
 * @brief Manejo de pulsación de botón BOOT
 * @param [in] void
 * @return void
 */

void gpio_pin_proccess(void)
{
    level = gpio_get_level(PIN_SWITCH);
    if (level && !once)
    {
        ESP_LOGI(TAG_GPIO, "IN: 1\n");
        once = 1;
        press_state = UNPRESSED;
    }
    if (!level && once)
    {
        ESP_LOGI(TAG_GPIO, "IN: 0\n");
        once = 0;
        press_state = PRESSED;
        set_timer(period);
    }
}

/**
 * @brief Manejo de estados de la pulsación de botón BOOT
 * @param [in] void
 * @return void
 */

void press_proccess(void)
{
    switch (press_state)
    {
    case PRESSED:
        if (set_dir)
        {
            if (cont_pin_pressed < 10)
            {
                cont_pin_pressed++;
                if (r_perc < 1 && b_perc > 0)
                {
                    r_perc += 0.1;
                    r_perc += 0.1;
                    b_perc -= 0.1;
                    r = r_perc * 255;
                    b = b_perc * 255;
                    r_int = (uint32_t)(r);
                    b_int = (uint32_t)(b);
                }
            }
            if (10 == cont_pin_pressed)
            {
                set_dir = 0;
            }
        }
        else
        {
            if (cont_pin_pressed > 0)
            {
                cont_pin_pressed--;
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
            if (0 == cont_pin_pressed)
            {
                set_dir = 1;
            }
        }

        ESP_LOGI(TAG_GPIO, "cont_pin_pressed: %d ", cont_pin_pressed);
        press_state = UNPRESSED;
        break;

    case UNPRESSED:

        break;
    }
}

void app_main()
{
    inicio_hw();
    init_uart();
    set_timer(period);
    ESP_ERROR_CHECK(init_wifi());
    ESP_ERROR_CHECK(init_esp_now());
    ESP_ERROR_CHECK(init_led_strip());
    // ESP_ERROR_CHECK(i2c_master_init());
    // i2c_task_func();
    first_on = 1;

    while (1)
    {
        gpio_pin_proccess();
        press_proccess();
        valve_outputs();
        touch_read();
        //  donde esta first_on es la logica de encendido/apagado del equipo
        if (first_on) // apagado
        {
            r_int = 0;
            b_int = 0;
        }
        else // encendido
        {
            r_int = (uint32_t)r;
            b_int = (uint32_t)b;
        }
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
    ESP_ERROR_CHECK(init_nvs());
    io_config.mode = GPIO_MODE_OUTPUT;
    // io_config.pin_bit_mask = ((1 << Led_R) | (1 << Led_G) | (1 << Led_B) | (1 << Salida_Bomba) | (1 << Salida_Compresor) | (1 << Salida_Cooler) | (1 << Salida_Resistencia) | (1 <<Salida_Fria_Up) | (1 << Salida_Fria_Down) | (1 << Salida_Caliente_Up) | (1 << Salida_Caliente_Down));
    io_config.pin_bit_mask = ((1 << Salida_Bomba) | (1 << Salida_Compresor) | (1 << Salida_Cooler) | (1 << Salida_Resistencia) | (1 << Salida_Fria_Up) | (1 << Salida_Fria_Down) | (1 << Salida_Caliente_Up) | (1 << Salida_Caliente_Down) | (1 << Salida_Electrovalvula_CO2));
    io_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_config.pull_up_en = GPIO_PULLDOWN_DISABLE;
    io_config.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_config);

    gpio_set_direction(PIN_SWITCH, GPIO_MODE_INPUT);
    gpio_pullup_en(PIN_SWITCH);
    gpio_reset_pin(Salida_EV1);
    gpio_reset_pin(Salida_EV2);
    gpio_reset_pin(Salida_EV3);
    gpio_set_direction(Salida_EV1, GPIO_MODE_OUTPUT);
    gpio_set_direction(Salida_EV2, GPIO_MODE_DEF_OUTPUT);
    gpio_set_direction(Salida_EV3, GPIO_MODE_DEF_OUTPUT);
    gpio_set_direction(Salida_EV4, GPIO_MODE_DEF_OUTPUT);
    gpio_set_direction(Salida_EV5, GPIO_MODE_DEF_OUTPUT);
    gpio_set_direction(Salida_EV6, GPIO_MODE_DEF_OUTPUT);

    touch_pad_init();
    // touch_pad_set_voltage(TOUCH_HVOLT_2V4, TOUCH_LVOLT_0V8, TOUCH_HVOLT_ATTEN_1V5);
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_0V5); // Chat GPT
    touch_pad_config(Touch_Nivel);
    touch_pad_config(Touch_Fuga);
    touch_pad_config(Touch_Caudal_Sube);
    touch_pad_config(Touch_Caudal_Baja);
    touch_pad_config(Touch_ON);
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
    // adc1_config_channel_atten(adc_caliente, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(adc_fria, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(adc_natural, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(adc_pres_ent, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(adc_pres_sal, ADC_ATTEN_DB_11);

    // printf("inicio_hw\n");
}

/**
 * @brief Funcion para definir e inicializar la tarea de I2C
 *
 */

static void i2c_task_func(void)
{
    xTaskCreate(I2C_task, "i2c_task", TASK_MEMORY, NULL, 5, NULL);
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
    touch_pad_read_raw_data(Touch_Nivel, &filtered_Nivel);
    // probar que pasa cuando se cambia por touch_pad_read_filtered
    touch_pad_read_raw_data(Touch_Fuga, &filtered_Fuga);
    touch_pad_read_raw_data(Touch_Caudal_Baja, &filtered_Caudal_Down);
    touch_pad_read_raw_data(Touch_Caudal_Sube, &filtered_Caudal_Up);
    touch_pad_read_raw_data(Touch_ON, &filtered_Touch_ON);

    if (calib_stage == CALIB_STAGE_3)
    {
        // uint8_t cont_stuck = 0;
        if (count_touch_read > 0)
        {
            count_touch_read--;
        }
        else
        {
            count_touch_read = 10;
            // if ((filtered_Nivel_Ant == filtered_Nivel) && (filtered_Fuga_Ant == filtered_Fuga) && (filtered_Caudal_Down == filtered_Caudal_Down_Ant) && (filtered_Caudal_Up == filtered_Caudal_Up_Ant))
            // {
            //     cont_stuck++;
            //     if (cont_stuck > 3)
            //     {
            //         esp_restart();
            //     }
            // }
            // ESP_LOGI("TOUCH 1", ANSI_COLOR_RED "touch 1 = %ld" ANSI_COLOR_RESET "\n", filtered_Caudal_Down);
            // ESP_LOGI("TOUCH 1", ANSI_COLOR_RED "touch 1 base = %ld" ANSI_COLOR_RESET "\n", filtered_Caudal_Down_Base);
            // ESP_LOGI("TOUCH 1", ANSI_COLOR_RED "touch 1 th = %ld" ANSI_COLOR_RESET "\n", filtered_Caudal_Down_Touch_Validated);

            // ESP_LOGI("TOUCH 2", ANSI_COLOR_GREEN "touch 2 = %ld" ANSI_COLOR_RESET "\n", filtered_Nivel);
            // ESP_LOGI("TOUCH 2", ANSI_COLOR_GREEN "touch 2 base = %ld" ANSI_COLOR_RESET "\n", filtered_Nivel_Base);
            // ESP_LOGI("TOUCH 2", ANSI_COLOR_GREEN "touch 2 th = %ld" ANSI_COLOR_RESET "\n", filtered_Nivel_Touch_Validated);

            // ESP_LOGI("TOUCH 3", ANSI_COLOR_BLUE "touch 3 = %ld" ANSI_COLOR_RESET "\n", filtered_Caudal_Up);
            // ESP_LOGI("TOUCH 3", ANSI_COLOR_BLUE "touch 3 base = %ld" ANSI_COLOR_RESET "\n", filtered_Caudal_Up_Base);
            // ESP_LOGI("TOUCH 3", ANSI_COLOR_BLUE "touch 3 th = %ld" ANSI_COLOR_RESET "\n", filtered_Caudal_Up_Touch_Validated);

            // ESP_LOGI("TOUCH 4", ANSI_COLOR_YELLOW "touch 4 = %ld" ANSI_COLOR_RESET "\n", filtered_Fuga);
            // ESP_LOGI("TOUCH 4", ANSI_COLOR_YELLOW "touch 4 base = %ld" ANSI_COLOR_RESET "\n", filtered_Fuga_Base);
            // ESP_LOGI("TOUCH 4", ANSI_COLOR_YELLOW "touch 4 th = %ld" ANSI_COLOR_RESET "\n", filtered_Fuga_Touch_Validated);

            ESP_LOGI("TOUCH ON", ANSI_COLOR_MAGENTA "touch ON = %ld" ANSI_COLOR_RESET "\n", filtered_Touch_ON);
            ESP_LOGI("TOUCH ON", ANSI_COLOR_MAGENTA "touch ON base = %ld" ANSI_COLOR_RESET "\n", filtered_Touch_ON_Base);

            touch_pad_read_raw_data(Touch_Nivel, &filtered_Nivel_Ant);
            touch_pad_read_raw_data(Touch_Fuga, &filtered_Fuga_Ant);
            touch_pad_read_raw_data(Touch_Caudal_Baja, &filtered_Caudal_Down_Ant);
            touch_pad_read_raw_data(Touch_Caudal_Sube, &filtered_Caudal_Up_Ant);

#ifdef ESP_NOW
            esp_err_t send_result = esp_now_send(peer_mac, (uint8_t *)&filtered_Caudal_Down, sizeof(filtered_Caudal_Down));
            if (send_result == ESP_OK)
            {
                // ESP_LOGI(TAG, "Data send to peer MAC");
            }
            else
            {
                ESP_LOGE(TAG_WIFI, "Error al enviar datos: %s", esp_err_to_name(send_result));
            }
            send_result = esp_now_send(peer_mac, (uint8_t *)&filtered_Caudal_Up, sizeof(filtered_Caudal_Up));
            if (send_result == ESP_OK)
            {
                // ESP_LOGI(TAG, "Data send to peer MAC");
            }
            else
            {
                ESP_LOGE(TAG_WIFI, "Error al enviar datos: %s", esp_err_to_name(send_result));
            }
            send_result = esp_now_send(peer_mac, (uint8_t *)&filtered_Nivel, sizeof(filtered_Nivel));
            send_result = esp_now_send(peer_mac, (uint8_t *)&filtered_Fuga, sizeof(filtered_Fuga));
#endif
        }
    }

/**
 * @brief PRUEBA CON MARMOL
 *
 */
#ifdef MARMOL_TEST
    if ((filtered_Caudal_Down > filtered_Caudal_Down_Base * 1.013) || (filtered_Caudal_Up > filtered_Caudal_Up_Base * 1.013))
    {
        for (int i = 0; i < LED_STRIP_MAX_LEDS; i++)
        {
            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 255, 255, 255));
            led_strip_refresh(led_strip);
        }
    }
    // if ((filtered_Caudal_Down < filtered_Caudal_Down_Base * 1.012) || (filtered_Caudal_Up < filtered_Caudal_Up_Base * 1.012))
    else
    {
        for (int i = 0; i < LED_STRIP_MAX_LEDS; i++)
        {
            ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 0, 0, 0));
            led_strip_refresh(led_strip);
        }
    }

#endif

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
        printf("touch 2 base = %ld\n ", filtered_Nivel_Base);
        filtered_Fuga_Base = filtered_Fuga;
        printf("touch 4 base = %ld\n ", filtered_Fuga_Base);
        filtered_Touch_ON_Base = filtered_Touch_ON;
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

        calib_cap_inputs();

        // vTaskDelay(1000 / portTICK_PERIOD_MS);
        if (xTaskCheckForTimeOut(&xTimeOut, &xTicksToWait) != pdFALSE)
        {
            printf("Calib stage 2 a 3\n");
            calib_stage = CALIB_STAGE_3;

            if ((!filtered_Caudal_Up_Touch_Validated) || (!filtered_Caudal_Down_Touch_Validated) || (!filtered_Nivel_Touch_Validated) || (!filtered_Fuga_Touch_Validated) || (!filtered_ON_Touch))
            {
                for (int i = 0; i < LED_STRIP_MAX_LEDS; i++)
                {
                    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 255, 0, 0));
                }
                led_strip_refresh(led_strip);
                get_value_from_nvs("storage", "filtUp", &cal_filt_up_nvs);
                get_value_from_nvs("storage", "filtDwn", &cal_filt_dwn_nvs);
                get_value_from_nvs("storage", "filtNiv", &cal_filt_nivel_nvs);
                get_value_from_nvs("storage", "filtFuga", &cal_filt_fuga_nvs);
                get_value_from_nvs("storage", "filtON", &cal_filt_on_nvs);
                filtered_Caudal_Up_Touch_Validated = cal_filt_up_nvs;
                filtered_Caudal_Down_Touch_Validated = cal_filt_dwn_nvs;
                filtered_Nivel_Touch_Validated = cal_filt_nivel_nvs;
                filtered_Fuga_Touch_Validated = cal_filt_fuga_nvs;
                filtered_ON_Touch = cal_filt_on_nvs;
                ESP_LOGI(TAG3, "Filtered Up: %ld\n", filtered_Caudal_Up_Touch_Validated);
                ESP_LOGI(TAG3, "Filtered Dwn: %ld\n", filtered_Caudal_Down_Touch_Validated);
                ESP_LOGI(TAG3, "Filtered Nivel: %ld\n", filtered_Nivel_Touch_Validated);
                ESP_LOGI(TAG3, "Filtered Fuga: %ld\n", filtered_Fuga_Touch_Validated);
                ESP_LOGI(TAG3, "Filtered Touch ON: %ld\n", filtered_ON_Touch);
            }
            else
            {
                for (int i = 0; i < LED_STRIP_MAX_LEDS; i++)
                {
                    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 0, 255, 0));
                }
                led_strip_refresh(led_strip);
                if (save_value_to_nvs("storage", "filtUp", filtered_Caudal_Up_Touch_Validated) != ESP_OK)
                {
                    ESP_LOGE("NVS", "Error en guardado");
                }
                if (save_value_to_nvs("storage", "filtDwn", filtered_Caudal_Down_Touch_Validated) != ESP_OK)
                {
                    ESP_LOGE("NVS", "Error en guardado");
                }
                if (save_value_to_nvs("storage", "filtNiv", filtered_Nivel_Touch_Validated) != ESP_OK)
                {
                    ESP_LOGE("NVS", "Error en guardado");
                }
                if (save_value_to_nvs("storage", "filtFuga", filtered_Fuga_Touch_Validated) != ESP_OK)
                {
                    ESP_LOGE("NVS", "Error en guardado");
                }
                if (save_value_to_nvs("storage", "filtON", filtered_ON_Touch) != ESP_OK)
                {
                    ESP_LOGE("NVS", "Error en guardado");
                }
            }

            vTaskDelay(1000 / portTICK_PERIOD_MS);

            for (int i = 0; i < LED_STRIP_MAX_LEDS; i++)
            {
                ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 0, 0, 0));
            }
            led_strip_refresh(led_strip);

            if (filtered_Fuga_Touch_Validated == 0 || filtered_Caudal_Up_Touch_Validated == 0 || filtered_Nivel_Touch_Validated == 0 || filtered_Caudal_Down_Touch_Validated == 0 || filtered_ON_Touch == 0)
            {
                calib_stage = CALIB_STAGE_1;
            }
        }
        break;
    case CALIB_STAGE_3:
        switch (touch_on_state)
        {
        case WAIT_TOUCH_ON:
            if (filtered_Touch_ON > 0.8 * filtered_ON_Touch)
            {
                touch_start_time = esp_timer_get_time();
                ESP_LOGI(TAG3, "TOUCH ON PRESSED = %ld\n", filtered_Touch_ON);
                touch_on_state = TOUCH_ON_PRESSED;
            }
            break;
        case TOUCH_ON_PRESSED:
            if (filtered_Touch_ON <= 0.8 * filtered_ON_Touch)
            {
                ESP_LOGI(TAG3, "TOUCH ON RELEASED = %ld\n", filtered_Touch_ON);
                touch_on_state = TOUCH_ON_RELEASED;
            }
            break;
        case TOUCH_ON_RELEASED:
            touch_duration = (esp_timer_get_time() - touch_start_time) / 1000;
            if (touch_duration >= MIN_PULSE_DURATION_MS && touch_duration <= MAX_PULSE_DURATION_MS)
            {
                first_on = !first_on; // cambia de apagado a encendido y viceversa
                if (first_on)
                {
                    printf("ESTADO OFF");
#ifdef VALV_MODUL
                    if (!count_mot_off)
                    {
                        count_mot_off = 100; // para apagado de motores
                    }
#endif
                    r_perc = 0;
                    b_perc = 0;
                    r_perc_ev = 0;
                    b_perc_ev = 0;
                }
                else
                {
                    printf("ESTADO ON");
                    r_perc = 0.3;
                    b_perc = 0.3;
                    // r_perc_ev = 0.5;
                    // b_perc_ev = 0.5;
                }
                r = r_perc * 255;
                b = b_perc * 255;
                r_int = (uint32_t)(r);
                b_int = (uint32_t)(b);
            }
            touch_on_state = WAIT_TOUCH_ON;
            break;
        }

        break;
    }

    if ((calib_stage == CALIB_STAGE_3) && (!first_on))
    {
        // maquina de estados para deteccion de pulsos de toque:
        switch (touch_state)
        {
        case WAIT_FOR_TOUCH:

            touch_act_detect = eval_touch_in();
            if (touch_act_detect)
            {
                touch_start_time = esp_timer_get_time();
                touch_state = TOUCH_DETECTED;
            }
            break;
        case TOUCH_DETECTED:

            switch (touch_act_detect)
            {
            case 1:
                if (filtered_Caudal_Down <= 1.1 * filtered_Caudal_Down_Base)
                {
                    touch_duration = (esp_timer_get_time() - touch_start_time) / 1000;
                    if (touch_duration >= MIN_PULSE_DURATION_MS && touch_duration <= MAX_PULSE_DURATION_MS)
                    {
                        touch_state = VALIDATE_PULSE;
                    }
                    else
                    {
                        touch_state = WAIT_FOR_TOUCH;
                        B_Fria_Down = 1;
                        printf("touch 1 rejected\n");
                        printf("touch 1 prev validated value: %ld\n", filtered_Caudal_Down_Touch_Validated);
                    }
                }
                break;
            case 2:
                if (filtered_Nivel <= 1.1 * filtered_Nivel_Base)
                {
                    touch_duration = (esp_timer_get_time() - touch_start_time) / 1000;
                    if (touch_duration >= MIN_PULSE_DURATION_MS && touch_duration <= MAX_PULSE_DURATION_MS)
                    {
                        touch_state = VALIDATE_PULSE;
                    }
                    else
                    {
                        touch_state = WAIT_FOR_TOUCH;
                        B_Caliente_Down = 1;
                        printf("touch 2 rejected\n");
                        printf("touch 2 prev validated value: %ld\n", filtered_Nivel_Touch_Validated);
                    }
                }
                break;

            case 3:
                if (filtered_Caudal_Up <= 1.1 * filtered_Caudal_Up_Base)
                {
                    touch_duration = (esp_timer_get_time() - touch_start_time) / 1000;
                    if (touch_duration >= MIN_PULSE_DURATION_MS && touch_duration <= MAX_PULSE_DURATION_MS)
                    {
                        touch_state = VALIDATE_PULSE;
                    }
                    else
                    {
                        touch_state = WAIT_FOR_TOUCH;
                        B_Fria_Up = 1;
                        printf("touch 3 rejected\n");
                        printf("touch 3 prev validated value: %ld\n", filtered_Caudal_Up_Touch_Validated);
                    }
                }
                break;

            case 4:
                if (filtered_Fuga <= 1.1 * filtered_Fuga_Base)
                {
                    touch_duration = (esp_timer_get_time() - touch_start_time) / 1000;
                    if (touch_duration >= MIN_PULSE_DURATION_MS && touch_duration <= MAX_PULSE_DURATION_MS)
                    {
                        touch_state = VALIDATE_PULSE;
                    }
                    else
                    {
                        touch_state = WAIT_FOR_TOUCH;
                        B_Caliente_Up = 1;
                        printf("touch 4 rejected\n");
                        printf("touch 4 prev validated value: %ld\n", filtered_Fuga_Touch_Validated);
                    }
                }
                break;
            }

            break;
        case VALIDATE_PULSE:
            // esto va dentro del bloque de entrada 1 detectada ------------------
            switch (touch_act_detect)
            {
            case 1:
#ifdef VALV_MODUL
                if (B_Caliente_Down && B_Caliente_Up)
                {
                    B_Fria_Down = 0;
                }
#endif
                // if (r_perc < 1 && b_perc > 0)
                // {
                //     r_perc += 0.1;
                //     b_perc -= 0.1;
                //     r = r_perc * 255;
                //     b = b_perc * 255;
                //     r_int = (uint32_t)(r);
                //     b_int = (uint32_t)(b);
                //     if (b_perc_ev > 0)
                //         b_perc_ev -= 0.1;
                //     // r_perc -= 0.1;
                // }
                if (b > 0.1)
                {
                    // r_perc += 0.1;
                    b_perc -= 0.1;
                    // r = r_perc * 255;
                    b = b_perc * 255;
                    // r_int = (uint32_t)(r);
                    b_int = (uint32_t)(b);
                    ESP_LOGI("TEMP CHG", "cold minus = %.2f\n", b_perc);
                }
                else
                {
                    ESP_LOGI("TEMP CHG", "cold min = %.2f\n", b_perc);
                    for (int i = 0; i < LED_STRIP_MAX_LEDS; i++)
                    {
                        ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 0, 0, 0));
                    }
                    led_strip_refresh(led_strip);
                    vTaskDelay(500 / portTICK_PERIOD_MS);
                }

                filtered_Caudal_Down_Touch_Validated = filtered_Caudal_Down_Touch_toValidate; // valido toque
                printf("touch 1 validated\n");
                printf("touch 1 validated value: %ld\n", filtered_Caudal_Down_Touch_Validated);
                // ESP_ERROR_CHECK(save_value_to_nvs("storage", "filtered_Caudal_Down_Touch_Validated", filtered_Caudal_Down_Touch_Validated));
                break;

            case 2:

#ifdef VALV_MODUL
                B_Caliente_Down = 0;
                B_Caliente_Up = 1;
#endif

                // if (b_perc < 1 && r_perc > 0)
                // {
                //     b_perc += 0.1;
                //     r_perc -= 0.1;
                //     r = r_perc * 255;
                //     b = b_perc * 255;
                //     r_int = (uint32_t)(r);
                //     b_int = (uint32_t)(b);
                //     if (r_perc_ev > 0)
                //         r_perc_ev -= 0.1;
                //     // b_perc -= 0.1;
                // }

                if (r_perc > 0.1)
                {
                    // b_perc += 0.1;
                    r_perc -= 0.1;
                    r = r_perc * 255;
                    // b = b_perc * 255;
                    r_int = (uint32_t)(r);
                    // b_int = (uint32_t)(b);
                    ESP_LOGI("TEMP CHG", "hot minus = %.2f\n", r_perc);
                }
                else
                {
                    ESP_LOGI("TEMP CHG", "hot min = %.2f\n", r_perc);
                    for (int i = 0; i < LED_STRIP_MAX_LEDS; i++)
                    {
                        ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 0, 0, 0));
                    }
                    led_strip_refresh(led_strip);
                    vTaskDelay(200 / portTICK_PERIOD_MS);
                }

                filtered_Nivel_Touch_Validated = filtered_Nivel_Touch_toValidate; // valido toque
                printf("touch 2 validated\n");
                printf("touch 2 validated value: %ld\n", filtered_Nivel_Touch_Validated);
                break;

            case 3:
#ifdef VALV_MODUL
                if (B_Caliente_Down && B_Caliente_Up)
                {
                    B_Fria_Up = 0;
                }
#endif

                // if (b_perc < 1 && r_perc > 0)
                // {
                //     b_perc += 0.1;
                //     r_perc -= 0.1;
                //     r = r_perc * 255;
                //     b = b_perc * 255;
                //     r_int = (uint32_t)(r);
                //     b_int = (uint32_t)(b);
                //     if (b_perc_ev < 1)
                //         b_perc_ev += 0.1;
                //     // r_perc += 0.1;
                // }

                if (b_perc < 1)
                {
                    b_perc += 0.1;
                    // r_perc -= 0.1;
                    // r = r_perc * 255;
                    b = b_perc * 255;
                    // r_int = (uint32_t)(r);
                    b_int = (uint32_t)(b);
                    ESP_LOGI("TEMP CHG", "cold add = %.2f\n", b_perc);
                }
                else
                {
                    ESP_LOGI("TEMP CHG", "cold max = %.2f\n", b_perc);
                    for (int i = 0; i < LED_STRIP_MAX_LEDS; i++)
                    {
                        ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 0, 0, 0));
                    }
                    led_strip_refresh(led_strip);
                    vTaskDelay(200 / portTICK_PERIOD_MS);
                }

                filtered_Caudal_Up_Touch_Validated = filtered_Caudal_Up_Touch_toValidate; // valido toque
                printf("touch 3 validated\n");
                printf("touch 3 validated value: %ld\n", filtered_Caudal_Up_Touch_Validated);
                break;

            case 4:

#ifdef VALV_MODUL
                B_Caliente_Down = 1;
                B_Caliente_Up = 0;
#endif

                // if (r_perc < 1 && b_perc > 0)
                // {
                //     r_perc += 0.1;
                //     b_perc -= 0.1;
                //     r = r_perc * 255;
                //     b = b_perc * 255;
                //     r_int = (uint32_t)(r);
                //     b_int = (uint32_t)(b);
                //     if (r_perc_ev < 1)
                //         r_perc_ev += 0.1;
                //     // b_perc += 0.1;
                // }

                if (r_perc < 1)
                {
                    r_perc += 0.1;
                    // b_perc -= 0.1;
                    r = r_perc * 255;
                    // b = b_perc * 255;
                    r_int = (uint32_t)(r);
                    // b_int = (uint32_t)(b);
                    ESP_LOGI("TEMP CHG", "hot add = %.2f\n", r_perc);
                }
                else
                {
                    ESP_LOGI("TEMP CHG", "hot max = %.2f\n", r_perc);
                    for (int i = 0; i < LED_STRIP_MAX_LEDS; i++)
                    {
                        ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, i, 0, 0, 0));
                    }
                    led_strip_refresh(led_strip);
                    vTaskDelay(500 / portTICK_PERIOD_MS);
                }

                filtered_Fuga_Touch_Validated = filtered_Fuga_Touch_toValidate; // valido toque
                printf("touch 4 validated\n");
                printf("touch 4 validated value: %ld\n", filtered_Fuga_Touch_Validated);
                break;
            }
            touch_state = WAIT_FOR_TOUCH;
            break;
        }
    }
}

void adc_read(void)
{
    for (int i = 0; i < Samples; i++)
    {
        // prom_caliente += adc1_get_raw(adc_caliente);
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

void set_pwm_duty(void)
{
}

static void I2C_task(void *pvParameters)
{
    uint8_t data_to_send[] = {0xAA, 0xBB, 0xCC}; // datos de prueba I2C
    uint8_t data_received[3];
    while (1)
    {
        esp_err_t ret = i2c_master_write_slave(data_to_send, sizeof(data_to_send));
        if (ret == ESP_OK)
        {
            ESP_LOGI(TAG_I2C, "Datos enviados correctamente");
        }
        memset(data_received, 0, sizeof(data_received));
        ret = i2c_master_read_slave(data_received, sizeof(data_received));
        if (ret == ESP_OK)
        {
            // ESP_LOGI(TAG_I2C, "Datos recibidos del esclavo: %02X %02X %02X", data_received[0], data_received[1], data_received[2]);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay 1 segundo
    }
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
    gpio_set_level(Salida_EV1, EV1);
    gpio_set_level(Salida_EV2, EV2);
    gpio_set_level(Salida_EV3, EV3);
    gpio_set_level(Salida_EV4, EV4);
    gpio_set_level(Salida_EV5, EV5);
    gpio_set_level(Salida_EV6, EV6);
#endif
    // gpio_set_level(Salida_Resistencia, !B_Resistencia);
    if (first_on && count_mot_off > 0)
    {
        gpio_set_level(Salida_Fria_Down, 1);
        gpio_set_level(Salida_Caliente_Down, 1);
        count_mot_off--;
    }
    // else
    // {
    //     first_on = 0;
    //     gpio_set_level(Salida_Fria_Down, !B_Fria_Down);
    //     gpio_set_level(Salida_Caliente_Down, !B_Caliente_Down);
    // }
}