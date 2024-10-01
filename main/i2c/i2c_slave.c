#include <stdio.h>
#include "i2c.h"

/**
 * @brief Inicialización del puerto I2C slave
 */
static esp_err_t i2c_slave_init(void)
{
    i2c_config_t conf_slave = {
        .mode = I2C_MODE_SLAVE,
        .sda_io_num = I2C_SLAVE_SDA_IO,
        .scl_io_num = I2C_SLAVE_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .slave.addr_10bit_en = 0,
        .slave.slave_addr = I2C_SLAVE_ADDR,
    };
    esp_err_t err = i2c_param_config(I2C_SLAVE_NUM, &conf_slave);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG_I2C_SLV, "I2C param config failed");
        return err;
    }
    return i2c_driver_install(I2C_SLAVE_NUM, conf_slave.mode, 128, 128, 0);
}

/**
 * @brief Función para recibir datos del maestro
 */
static void i2c_slave_read_task(void *arg)
{
    while (1)
    {
        int read_bytes = i2c_slave_read_buffer(I2C_SLAVE_NUM, slave_data, DATA_LENGTH, 1000 / portTICK_PERIOD_MS);
        if (read_bytes > 0)
        {
            ESP_LOGI(TAG_I2C_SLV, "Datos recibidos del maestro: 0x%X 0x%X 0x%X", slave_data[0], slave_data[1], slave_data[2]);
        }
    }
}

/**
 * @brief Función para enviar los datos almacenados al maestro
 */
static void i2c_slave_write_task(void *arg)
{
    while (1)
    {
        int written_bytes = i2c_slave_write_buffer(I2C_SLAVE_NUM, slave_data, DATA_LENGTH, 1000 / portTICK_PERIOD_MS);
        if (written_bytes > 0)
        {
            ESP_LOGI(TAG_I2C_SLV, "Datos enviados al maestro");
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief Funcion que inicializa el driver I2C Slave y crea las task de escritura y lectura
 *
 */

void i2c_slave_task_start(void)
{
    ESP_ERROR_CHECK(i2c_slave_init());
    xTaskCreate(i2c_slave_read_task, "i2c_slave_read_task", 1024 * 2, NULL, 10, NULL);
    xTaskCreate(i2c_slave_write_task, "i2c_slave_write_task", 1024 * 2, NULL, 10, NULL);
}