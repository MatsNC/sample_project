#include "capacitivos.h"
#include "led_strip_local/led_strip_local.h"

bool touch_active = false;

void update_touch_average(uint32_t new_value)
{
    // Sustituir el valor más antiguo en el buffer
    touch_buffer[buffer_index] = new_value;
    buffer_index = (buffer_index + 1) % BUFFER_SIZE;

    // Calcular el promedio
    uint64_t sum = 0;
    for (int i = 0; i < BUFFER_SIZE; i++)
    {
        sum += touch_buffer[i];
    }
    touch_avg = sum / BUFFER_SIZE;
}

void touch_initialize(void)
{
    // Inicializar el sensor capacitivo
    touch_pad_init();
    // touch_pad_set_voltage(TOUCH_HVOLT_2V4, TOUCH_LVOLT_0V8, TOUCH_HVOLT_ATTEN_1V5);
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_0V5); // Chat GPT
}

void calibrate_touch()
{
    led_strip_set_pixel(led_strip, 0, 0, 255, 0);
    led_strip_refresh(led_strip);
    ESP_LOGI(TAG, ANSI_COLOR_YELLOW "Iniciando calibración... TOQUE el sensor." ANSI_COLOR_RESET);

    int last_value = 0;
    int max_derivative = 0;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++)
    {
        int raw_value;
        touch_pad_read_raw_data(Touch_Test, &raw_value);

        // Primera lectura, inicializa el valor base
        if (i == 0)
        {
            last_value = raw_value;
            continue;
        }

        // Calcular derivada
        int derivative = raw_value - last_value;
        last_value = raw_value;

        // Guardar el máximo cambio detectado
        if (abs(derivative) > max_derivative)
        {
            max_derivative = abs(derivative);
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Leer cada 10ms
    }

    // Definir el umbral dinámico como un porcentaje del máximo detectado
    dynamic_threshold = max_derivative * 0.3; // Ajusta este factor según pruebas
    if (dynamic_threshold < 30)
    {
        dynamic_threshold = 30; // Valor mínimo de seguridad
    }

    ESP_LOGI(TAG, ANSI_COLOR_YELLOW "Calibración completa. Umbral dinámico: %d" ANSI_COLOR_RESET, dynamic_threshold);
    led_strip_set_pixel(led_strip, 0, 0, 0, 0);
    led_strip_refresh(led_strip);
    // if (save_value_to_nvs("storage", "Touch_Test", dynamic_threshold) != ESP_OK)
    // {
    //     ESP_LOGE("NVS", "Error en guardado");
    // }
}

void moving_average_filter(uint32_t raw_value)
{
    // Aplicar media móvil
    sum -= history[indice];
    history[indice] = raw_value;
    sum += history[indice];
    indice = (indice + 1) % NUM_SAMPLES;
    filtered_value = sum / NUM_SAMPLES;
}

void touch_derivative_detection()
{
    uint32_t raw_value;
    touch_pad_read_raw_data(Touch_Test, &raw_value);

    // Aplicar media móvil
    moving_average_filter(raw_value);

    // Calcular derivada
    int derivative = filtered_value - last_value;
    last_value = filtered_value;

    // ESP_LOGE(TAG, "filtered: %d", filtered_value);

    if (derivative > dynamic_threshold && !touch_active)
    {
        touch_active = true;
        touch_start = esp_log_timestamp();
        ESP_LOGW(TAG, "🔥 TOQUE DETECTADO");
    }

    if (derivative < dynamic_threshold && touch_active)
    {
        uint32_t touch_duration = esp_log_timestamp() - touch_start;
        touch_active = false;
        ESP_LOGW(TAG, "🛑 TOQUE SOLTADO (Duración: %ld ms)", touch_duration);

        if (touch_duration >= MIN_TOUCH_TIME_MS && touch_duration <= MAX_TOUCH_TIME_MS)
        {
            ESP_LOGW(TAG, "✅ ¡TOQUE VÁLIDO!");
            led_strip_set_pixel(led_strip, 0, 255, 255, 255);
            led_strip_refresh(led_strip);
        }
        else
        {
            ESP_LOGE(TAG, "❌ Toque rechazado.");
        }
    }
    vTaskDelay(pdMS_TO_TICKS(10)); // Leer cada 10ms
}