#ifndef I2C_H
#define I2C_H

#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c.h"

#define I2C_MASTER_SCL_IO 8         // GPIO del ESP32-S3 para SCL
#define I2C_MASTER_SDA_IO 3         // GPIO del ESP32-S3 para SDA
#define I2C_SLAVE_SCL_IO 8          // GPIO del ESP32-S3 para SCL SLAVE
#define I2C_SLAVE_SDA_IO 3          // GPIO del ESP32-S3 para SDA SLAVE

#define I2C_MASTER_NUM I2C_NUM_0    // I2C numero 0
#define I2C_SLAVE_NUM I2C_NUM_0     // I2C Port 0 para ESP32-C3

//! @brief Frecuencia de I2C (100 kHz)
#define I2C_MASTER_FREQ_HZ 100000    

//! @brief No se usa buffer de TX
#define I2C_MASTER_TX_BUF_DISABLE 0 

//! @brief No se usa buffer de RX
#define I2C_MASTER_RX_BUF_DISABLE 0 
#define I2C_MASTER_TIMEOUT_MS 1000  // Timeout en ms
#define I2C_SLAVE_ADDR 0x28         // Dirección I2C del esclavo
#define ACK_CHECK_EN 0x1            /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0           /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                 /*!< I2C ack value */
#define NACK_VAL 0x1                /*!< I2C nack value */
#define DATA_LENGTH 3               // Tamaño de datos a enviar para slave

static const char *TAG_I2C = "I2C_MASTER";
static const char *TAG_I2C_SLV = "I2C_SLAVE";
static uint8_t slave_data[DATA_LENGTH]; // Buffer de datos del esclavo

esp_err_t i2c_master_init(void);
esp_err_t i2c_master_write_slave(uint8_t *, size_t);
esp_err_t i2c_master_read_slave(uint8_t *, size_t);
static esp_err_t i2c_slave_init(void);
static void i2c_slave_read_task(void *);
static void i2c_slave_write_task(void *);
void i2c_slave_task_start(void);

#endif // I2C_H
