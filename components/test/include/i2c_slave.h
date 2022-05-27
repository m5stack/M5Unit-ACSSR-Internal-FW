#pragma once

#ifdef __cplusplus
extern "C" {
#endif
#include "sdkconfig.h"

#define CTRL_ADDR     0x00
#define MODSLAVE_ADDR 0x10
#define FUNC_MODE     0x05

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num)  _I2C_NUMBER(num)

#define DATA_LENGTH    512 /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 128 /*!< Data length for r/w test, [0,DATA_LENGTH] */
#define DELAY_TIME_BETWEEN_ITEMS_MS \
    1000 /*!< delay time between different test items */

#define I2C_SLAVE_SCL_IO \
    CONFIG_I2C_SLAVE_SCL /*!< gpio number for i2c slave clock */
#define I2C_SLAVE_SDA_IO \
    CONFIG_I2C_SLAVE_SDA /*!< gpio number for i2c slave data */
#define I2C_SLAVE_NUM \
    CONFIG_I2C_SLAVE_PORT_NUM /*!< I2C port number for slave dev */
#define I2C_SLAVE_TX_BUF_LEN                        \
    (2 * DATA_LENGTH) /*!< I2C slave tx buffer size \
                       */
#define I2C_SLAVE_RX_BUF_LEN                        \
    (2 * DATA_LENGTH) /*!< I2C slave rx buffer size \
                       */

#define ESP_SLAVE_ADDR                                                      \
    CONFIG_I2C_SLAVE_ADDRESS /*!< ESP32 slave address, you can set any 7bit \
                                value */

#define ACK_CHECK_EN  0x1 /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0 /*!< I2C master will not check ack from slave */
#define ACK_VAL       0x0 /*!< I2C ack value */
#define NACK_VAL      0x1 /*!< I2C nack value */

#define OUTDRIVE_REG  CONFIG_OUTPUT_REG  /*!< Coil Register */
#define NEO_REG_START CONFIG_NEOLED_ADDR /*!< Neo-Pixel Register*/
#define NEO_REG_END   (CONFIG_NEOLED_ADDR + 2)
#define VERSION_REG   (uint8_t) CONFIG_VERSION_ADDR /*!< Version Register*/
#define MOD_I2C_REG   CONFIG_MODIFY_I2C_ADDR

#define register_size 256

esp_err_t i2c_slave_init(void);
void i2c_slave_update();
void i2c_slave_task(void *Arg);

typedef struct i2c_slave_struct {
    uint8_t rw_reg;
    uint8_t reg_length;
    uint8_t rw_register_data[register_size];  // Register data
    // uint8_t wr_register_data[register_size]; // Register data
} i2c_data;

i2c_data i2c_reg_data;

#ifdef __cplusplus
}
#endif
