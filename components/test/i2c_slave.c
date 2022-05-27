/* i2c - Example

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "soc/i2c_struct.h"
#include "soc/i2c_reg.h"
#include <driver/periph_ctrl.h>
#include <soc/i2c_periph.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "i2c_slave.h"
#include "modbusslave.h"
#include "boardio.h"
#include "neopixel.h"
#include "nvs_memory.h"

static const char *TAG                                      = "I2C_SLAVE";
static DRAM_ATTR i2c_dev_t *const I2C_INSTANCE[I2C_NUM_MAX] = {&I2C0};
static bool update_reg_index                                = true;
static bool reg_modified                                    = false;
size_t current_reg_index                                    = 0;
size_t write_reg_index = 0;  // I2C register number to write to
size_t read_reg_index  = 0;
static uint32_t i2c_rx_fifo_full_thresh_val =
    1;  // Interrupt occurs when the receive FIFO buffer exceeds this number
static uint32_t i2c_tx_fifo_empty_thresh_val =
    2;  // Interrupt occurs when the transmit FIFO buffer falls below this
        // number
static uint32_t i2c_slave_sda_sample_val =
    4;  // SDA sample time after SCL startup
static uint32_t i2c_slave_sda_hold_val = 4;  // SDA hold time after SCL fall
uint16_t incr                          = 0;

static QueueHandle_t I2C_ISR_Evt_Queue = NULL;
SemaphoreHandle_t i2c_mutex            = NULL;

static void IRAM_ATTR i2c_slave_isr_handler(void *arg) {
    int i2c_num = I2C_SLAVE_NUM;
    i2c_data *i2c_send_data;
    i2c_send_data = &i2c_reg_data;
    typeof(I2C_INSTANCE[i2c_num]->int_status) int_sts;
    int_sts.val                        = I2C_INSTANCE[i2c_num]->int_status.val;
    I2C_INSTANCE[i2c_num]->int_clr.val = int_sts.val;

    int rx_fifo_cnt = I2C_INSTANCE[i2c_num]->sr.rx_fifo_cnt;
    if (rx_fifo_cnt) {
        // / Treat the first data in the received data string as a register
        // number ets_printf("rx_fifo_cnt :%d\n", rx_fifo_cnt);
        if (update_reg_index) {
            update_reg_index = false;
            // / Update both write and read register numbers
            write_reg_index   = (I2C_INSTANCE[i2c_num]->fifo_data.val) & 255;
            read_reg_index    = write_reg_index;
            current_reg_index = write_reg_index;
            i2c_send_data->rw_reg = current_reg_index;
            // / Clear transmit FIFO buffer
            I2C_INSTANCE[i2c_num]->fifo_conf.tx_fifo_rst = 1;
            I2C_INSTANCE[i2c_num]->fifo_conf.tx_fifo_rst = 0;
            --rx_fifo_cnt;
        }
        // / The rest of the data is reflected in the register data
        if (rx_fifo_cnt) {
            // / Set register data update flag
            reg_modified = true;
            do {
                uint32_t data = I2C_INSTANCE[i2c_num]->fifo_data.val;
                if (write_reg_index < register_size) {
                    if (write_reg_index != VERSION_REG)
                        i2c_send_data->rw_register_data[write_reg_index] = data;
                    // ets_printf("wr_data :%d\n",
                    // i2c_send_data->rw_register_data[write_reg_index]);
                    ++write_reg_index;
                }
            } while (--rx_fifo_cnt);
            i2c_send_data->reg_length = write_reg_index - current_reg_index;
        }
    }
    // / Store data in the transmit FIFO buffer
    while (I2C_INSTANCE[i2c_num]->sr.tx_fifo_cnt < 8) {
        I2C_INSTANCE[i2c_num]->fifo_data.val =
            (read_reg_index < register_size)
                ? i2c_send_data->rw_register_data[read_reg_index]
                : 0;
        // ets_printf("tx_fifo_cnt :%d\n",
        // I2C_INSTANCE[i2c_num]->sr.tx_fifo_cnt); ets_printf("rd_data :%d\n",
        // i2c_send_data->rw_register_data[read_reg_index]);
        // ets_printf("rd_reg:%d\n", read_reg_index);
        ++read_reg_index;
    }

    // / When communication is terminated
    if (int_sts.trans_complete || int_sts.arbitration_lost) {
        //  Set a flag to handle the next received data as a register number
        update_reg_index = true;
        if (reg_modified) {
            xQueueSendFromISR(I2C_ISR_Evt_Queue, (void *)&i2c_send_data, NULL);
            // incr++;
            // ets_printf("empty data: %d\n",incr);
            //  Notify the main task if there is a register data update

            reg_modified = false;
            portYIELD_FROM_ISR();
        }
    }
}

/*********************************************************/
/*****************I2C Slave Receive Task******************/
/*********************************************************/
void i2c_slave_task(void *Arg) {
    while (1) {
        xSemaphoreTake(i2c_mutex, portMAX_DELAY);
        // vPortEnterCritical();
        i2c_slave_update();
        // ets_printf("empty1...\n");
        // vPortExitCritical();
        xSemaphoreGive(i2c_mutex);
    }
    vSemaphoreDelete(i2c_mutex);
    vTaskDelete(NULL);
}

/*********************************************************/
/******************I2C Slave Initialized******************/
/*********************************************************/
esp_err_t i2c_slave_init() {
    int i2c_num      = I2C_SLAVE_NUM;
    uint8_t i2c_addr = ESP_SLAVE_ADDR;
    if ((ESP_OK == i2c_set_pin(I2C_SLAVE_NUM, I2C_SLAVE_SDA_IO,
                               I2C_SLAVE_SCL_IO, GPIO_PULLUP_ENABLE,
                               GPIO_PULLUP_ENABLE, I2C_MODE_SLAVE)) &&
        (ESP_OK == esp_intr_alloc(ETS_I2C_EXT0_INTR_SOURCE,
                                  ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL3,
                                  i2c_slave_isr_handler, NULL, NULL))) {
        I2C_INSTANCE[i2c_num]->int_ena.val = 0;

        periph_module_enable(PERIPH_I2C0_MODULE);

        I2C_INSTANCE[i2c_num]->fifo_conf.tx_fifo_rst = 1;
        I2C_INSTANCE[i2c_num]->fifo_conf.tx_fifo_rst = 0;
        I2C_INSTANCE[i2c_num]->fifo_conf.rx_fifo_rst = 1;
        I2C_INSTANCE[i2c_num]->fifo_conf.rx_fifo_rst = 0;

        typeof(I2C_INSTANCE[i2c_num]->ctr) ctrl_reg;
        ctrl_reg.val                   = 0;
        ctrl_reg.sda_force_out         = 1;
        ctrl_reg.scl_force_out         = 1;
        I2C_INSTANCE[i2c_num]->ctr.val = ctrl_reg.val;

        i2c_addr = nvs_read_data_from_flash("i2c_addr");
        // ets_printf("i2c slave addr: %d\n", i2c_addr);
        I2C_INSTANCE[i2c_num]->slave_addr.addr     = i2c_addr;
        I2C_INSTANCE[i2c_num]->slave_addr.en_10bit = 0;

        I2C_INSTANCE[i2c_num]->sda_hold.time   = i2c_slave_sda_hold_val;
        I2C_INSTANCE[i2c_num]->sda_sample.time = i2c_slave_sda_sample_val;

        I2C_INSTANCE[i2c_num]->ctr.slv_tx_auto_start_en = 1;

        I2C_INSTANCE[i2c_num]->timeout.time_out_value = 31;
        I2C_INSTANCE[i2c_num]->timeout.time_out_en    = 0;

        I2C_INSTANCE[i2c_num]->filter_cfg.val       = 0;
        I2C_INSTANCE[i2c_num]->filter_cfg.scl_en    = 1;
        I2C_INSTANCE[i2c_num]->filter_cfg.scl_thres = 0;
        I2C_INSTANCE[i2c_num]->filter_cfg.sda_en    = 1;
        I2C_INSTANCE[i2c_num]->filter_cfg.sda_thres = 0;

        typeof(I2C_INSTANCE[i2c_num]->fifo_conf) fifo_conf;
        fifo_conf.val                        = 0;
        fifo_conf.rx_fifo_wm_thrhd           = i2c_rx_fifo_full_thresh_val;
        fifo_conf.tx_fifo_wm_thrhd           = i2c_tx_fifo_empty_thresh_val;
        I2C_INSTANCE[i2c_num]->fifo_conf.val = fifo_conf.val;

        I2C_INSTANCE[i2c_num]->int_ena.val =
            I2C_TRANS_COMPLETE_INT_ENA | I2C_ARBITRATION_LOST_INT_ENA |
            I2C_BYTE_TRANS_DONE_INT_ENA | I2C_TXFIFO_WM_INT_ENA |
            I2C_RXFIFO_WM_INT_ENA;

        i2c_reg_data.rw_register_data[VERSION_REG] =
            (uint8_t)CONFIG_DEVICE_VERSION;
        i2c_reg_data.rw_register_data[MOD_I2C_REG] = i2c_addr;
        I2C_INSTANCE[i2c_num]->ctr.conf_upgate     = 1;
        ESP_LOGI(TAG, "ESP32 Slave Initizial Finish\n");
        I2C_ISR_Evt_Queue =
            xQueueCreate(520, sizeof(struct i2c_slave_struct *));
        i2c_mutex = xSemaphoreCreateMutex();
        return ESP_OK;
    }
    return ESP_ERR_INVALID_ARG;
}

/*********************************************************/
/*****I2C Slave Write Function of Drive and NeoPixel******/
/*********************************************************/
void i2c_slave_update() {
    i2c_data *i2c_recv_data;
    i2c_recv_data = &i2c_reg_data;
    if (xQueueReceive(I2C_ISR_Evt_Queue, (void *)&i2c_recv_data,
                      portMAX_DELAY)) {
        if (((uint8_t)OUTDRIVE_REG) == i2c_recv_data->rw_reg) {
            if (i2c_recv_data->rw_register_data[i2c_recv_data->rw_reg]) {
                ctrl_set_op(1);
            } else {
                ctrl_set_op(0);
            }
        } else if ((i2c_recv_data->rw_reg >= NEO_REG_START) &
                   (i2c_recv_data->rw_reg <= NEO_REG_END)) {
            if (i2c_recv_data->reg_length == 1) {
                if (i2c_recv_data->rw_reg == NEO_REG_START) {
                    flash_color =
                        ((i2c_recv_data->rw_register_data[i2c_recv_data->rw_reg]
                          << 16) |
                         (flash_color & 0x00ffff));
                } else if (i2c_recv_data->rw_reg == (NEO_REG_START + 1)) {
                    flash_color =
                        ((i2c_recv_data->rw_register_data[i2c_recv_data->rw_reg]
                          << 8) |
                         (flash_color & 0xff00ff));
                } else if (i2c_recv_data->rw_reg == NEO_REG_END) {
                    flash_color =
                        i2c_recv_data->rw_register_data[i2c_recv_data->rw_reg] |
                        (flash_color & 0xffff00);
                }
            } else if (i2c_recv_data->reg_length > 1) {
                flash_color =
                    ((i2c_recv_data->rw_register_data[i2c_recv_data->rw_reg]
                      << 16) |
                     (i2c_recv_data->rw_register_data[i2c_recv_data->rw_reg + 1]
                      << 8) |
                     i2c_recv_data
                         ->rw_register_data[i2c_recv_data->rw_reg + 2]);
            }
            // Led_Ctrl_State(1, 0, 0, flash_color);
            neo_color_all(flash_color, CONFIG_LED_BRIGHTNESS);
            // ets_printf("r:%d, g:%d, b:%d\n", (flash_color >> 16),
            // ((flash_color >> 8) & 0xff), (flash_color & 0xff));
        } else if (i2c_recv_data->rw_reg == MOD_I2C_REG) {
            nvs_write_data_to_flash(
                "i2c_addr", (uint8_t)i2c_recv_data
                                ->rw_register_data[i2c_recv_data->rw_reg]);
            vTaskDelay(pdMS_TO_TICKS(250));
            esp_restart();
        }
        // ets_printf("rw_reg :%d\n", i2c_recv_data->rw_reg);
        // ets_printf("reg_length :%d\n", i2c_recv_data->reg_length);
        // ets_printf("rw_reg1 :%d\n",
        // i2c_recv_data->rw_register_data[i2c_recv_data->rw_reg]);
        // ets_printf("rw_reg2 :%d\n",
        // i2c_recv_data->rw_register_data[i2c_recv_data->rw_reg+1]);
        // ets_printf("rw_reg3 :%d\n",
        // i2c_recv_data->rw_register_data[i2c_recv_data->rw_reg+2]);
        i2c_reset_rx_fifo(I2C_SLAVE_NUM);
    }
}
