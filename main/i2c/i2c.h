#ifndef I2C_H
#define I2C_H

#include "esp_log.h"
#include "esp_err.h"

#define I2C_MASTER_SCL_IO 22        // GPIO del ESP32-S3 para SCL
#define I2C_MASTER_SDA_IO 21        // GPIO del ESP32-S3 para SDA
#define I2C_MASTER_NUM I2C_NUM_0    // I2C numero 0
#define I2C_MASTER_FREQ_HZ 400000   // Frecuencia de I2C (100 kHz)
#define I2C_MASTER_TX_BUF_DISABLE 0 // No se usa buffer de TX
#define I2C_MASTER_RX_BUF_DISABLE 0 // No se usa buffer de RX
#define I2C_MASTER_TIMEOUT_MS 1000  // Timeout en ms
#define I2C_SLAVE_ADDRESS 0x28      // Dirección I2C del esclavo
#define I2C_MASTER_TIMEOUT_MS 1000  // Timeout en ms

static const char *TAG_I2C = "I2C_MASTER";

void i2c_master_init(void);
esp_err_t i2c_write_to_device(uint8_t* data, size_t len);
esp_err_t i2c_read_from_device(uint8_t *data, size_t len);

#endif // I2C_H
