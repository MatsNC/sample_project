#include "i2c.h"

static const char *TAG = "I2C_MASTER";

/**
 * @brief Inicializa driver I2C
 * @param none
 * @return void
 */

esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "I2C param config failed");
        return err;
    }
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

/**
 * @brief Envía datos al esclavo a través de I2C.
 *
 * Esta función envía un array de bytes al esclavo utilizando la interfaz I2C.
 *
 * @param[in] data Puntero al array de datos que se desea enviar.
 * @param[in] len Longitud del array de datos a enviar.
 * @return
 *      - ESP_OK si los datos fueron enviados exitosamente.
 *      - Otro valor de error de `esp_err_t` si ocurrió un error durante la transmisión.
 */

esp_err_t i2c_master_write_slave(uint8_t *data, size_t len)
{
    esp_err_t err = i2c_master_write_to_device(I2C_MASTER_NUM, I2C_SLAVE_ADDR, data, sizeof(data), 1000 / portTICK_PERIOD_MS);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error al escribir al esclavo");
    }
    return err;
}

/**
 * @brief Lee datos del esclavo a través de I2C.
 *
 * Esta función lee un array de bytes desde el esclavo utilizando la interfaz I2C.
 *
 * @param[out] data Puntero al array donde se almacenarán los datos leídos.
 * @param[in] len Longitud del array de datos a leer.
 * @return
 *      - ESP_OK si los datos fueron leídos exitosamente.
 *      - Otro valor de error de `esp_err_t` si ocurrió un error durante la recepción.
 */

esp_err_t i2c_master_read_slave(uint8_t *data, size_t len)
{
    esp_err_t err = i2c_master_read_from_device(I2C_MASTER_NUM, I2C_SLAVE_ADDR, data, len, 1000 / portTICK_PERIOD_MS);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error al leer del esclavo");
    }
    return err;
}