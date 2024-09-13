#include "i2c.h"
#include "driver/i2c.h"

static const char *TAG = "I2C_MASTER";

/**
 * @brief Inicializa driver I2C
 * @param none
 * @return void
 */

void i2c_master_init(void)
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
        ESP_LOGE(TAG, "Error configurando I2C: %s", esp_err_to_name(err));
        return;
    }
    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error instalando el controlador I2C: %s\n", esp_err_to_name(err));
        return;
    }
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

esp_err_t i2c_write_to_device(uint8_t* data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_SLAVE_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "ERROR ESCRIBIENDO EN SLAVE: %s\n", esp_err_to_name(ret));
    }
    return ret;
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

esp_err_t i2c_read_from_device(uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (I2C_SLAVE_ADDRESS << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "ERROR LEYENDO DE SLAVE: %s\n", esp_err_to_name(ret));
    }
    return ret;
}