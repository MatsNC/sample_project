#include <stdio.h>
#include <stdlib.h>
#include "driver/touch_pad.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "../nvs/nvs.h"
#include "esp_err.h"
#include "../led_strip_local/led_strip_local.h"

#define Touch_Test TOUCH_PAD_NUM2
#define NUM_SAMPLES 5             // Tamaño del filtro de media móvil
#define CALIBRATION_TIME_MS 3000  // Tiempo de calibración en milisegundos
#define CALIBRATION_SAMPLES (CALIBRATION_TIME_MS / 10)  // Cantidad de muestras en calibración
////////////////////////////////////////////////////////////////////////
#define BUFFER_SIZE 10 // Tamaño del buffer para el promedio móvil

#define MIN_TOUCH_TIME_MS 100     // Tiempo mínimo válido para un toque
#define MAX_TOUCH_TIME_MS 2000     // Tiempo máximo válido para un toque

#define ANSI_COLOR_RED "\x1b[31m"
#define ANSI_COLOR_GREEN "\x1b[32m"
#define ANSI_COLOR_YELLOW "\x1b[33m"
#define ANSI_COLOR_BLUE "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_RESET "\x1b[0m"

static const char *TAG = "TOUCH_DERIVADA";
static int history[NUM_SAMPLES] = {0};  
static int indice = 0;
static int dynamic_threshold = 1000;  // Umbral de derivada dinámico

static uint32_t touch_buffer[BUFFER_SIZE] = {0}; // Buffer circular
static int buffer_index = 0;                     // Índice actual en el buffer
static uint32_t touch_avg = 0;                   // Promedio de las lecturas

static uint32_t filtered_Cold_Up;
static uint32_t filtered_Cold_Up_Base;
static uint32_t filtered_Cold_Up_Touch;
static uint32_t filtered_Cold_Up_Ant;
static uint32_t filtered_Cold_Up_Touch_toValidate; // Fria +
static uint32_t filtered_Cold_Up_Touch_Validated = 0;
static int last_value = 0;
static int sum = 0;
static uint32_t cal_filt_on_test;
static bool touch_detected = false;
static uint32_t touch_start = 0;
static int filtered_value;

void update_touch_average(uint32_t);
void touch_initialize(void);
void calibrate_touch(void);
void touch_derivative_detection(void);
void moving_average_filter(uint32_t);