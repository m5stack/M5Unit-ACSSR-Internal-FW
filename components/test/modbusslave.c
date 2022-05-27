#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "esp_intr_alloc.h"
#include <driver/periph_ctrl.h>
#include <soc/uart_periph.h>
#include "soc/uart_struct.h"
#include "soc/uart_reg.h"
#include "modbusslave.h"
#include "boardio.h"
#include "neopixel.h"
#include "nvs_memory.h"

static const char *TAG = "MODBUS_UART";

#define EX_UART_NUM UART_NUM_1
#define PATTERN_CHR_NUM                                                      \
    (3) /*!< Set the number of consecutive and identical characters received \
           by receiver which defines a UART pattern*/

#define TXD_PIN CONFIG_TXD_PIN
#define RXD_PIN CONFIG_RXD_PIN

#define BUF_SIZE    (1024)
#define RD_BUF_SIZE (BUF_SIZE)

#define MDBSLAVE_ADDR CONFIG_MOD_SLAVE_ADDRESS

#define WCSSR_ADDR   CONFIG_SSR_ADDR
#define WRLED_ADDR   CONFIG_LED_ADDR
#define WRVER_ADDR   CONFIG_VER_ADDR
#define WRSLAVE_ADDR CONFIG_SLAVE_ADDR
#define COIL_ON      (0xFF00)
#define COIL_OFF     (0x00)

static QueueHandle_t UART_ISR_Evt_Queue = NULL;
static intr_handle_t handle_console;
SemaphoreHandle_t uart_mutex = NULL;

uint8_t CRCBytes[2];
uint8_t slave_addr    = 1;
uint8_t function_code = 1;
uint16_t reg_address  = 0;
uint16_t quantity     = 1;
uint16_t coil_status  = 0x0000;

uint8_t mdbslave_addr = 1;

size_t recv_index  = 0;
uint8_t data_check = 0;

static void IRAM_ATTR uart_intr_handle(void *arg) {
    volatile uart_dev_t *uart = &UART1;
    uint16_t rx_fifo_len;
    uart_data *uart_send_data;
    uart_send_data            = &uart_reg_data;
    uart->int_clr.rxfifo_full = 1;
    uart->int_clr.frm_err     = 1;
    uart->int_clr.rxfifo_tout = 1;
    rx_fifo_len               = uart->status.rxfifo_cnt;
    recv_index                = 0;
    if (rx_fifo_len) {
        uart_send_data->buff_length = rx_fifo_len;
    }

    while (rx_fifo_len) {
        uart_send_data->recv_buff[recv_index] = uart->ahb_fifo.rw_byte;
        // ets_printf("uart_data :%d\n", uart_send_data->recv_buff[recv_index]);
        rx_fifo_len--;
        recv_index++;
    }
    if (recv_index) {
        recv_index = 0;
        xQueueSendFromISR(UART_ISR_Evt_Queue, (void *)&uart_send_data, NULL);
        // after reading bytes from buffer clear UART interrupt status
        uart_clear_intr_status(
            EX_UART_NUM, UART_RXFIFO_FULL_INT_CLR | UART_RXFIFO_TOUT_INT_CLR);
        portYIELD_FROM_ISR();
    }
}

/*********************************************************/
/*****************Uart Modbus Initialized*****************/
/*********************************************************/
esp_err_t modbusuart_init(void) {
    esp_log_level_set(TAG, ESP_LOG_INFO);
    ESP_LOGI(TAG, "Configure the uart");

    uart_config_t uart_config = {
        .baud_rate  = 115200,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    esp_err_t err = uart_param_config(EX_UART_NUM, &uart_config);

    // Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    // Set UART pins (using UART0 default pins ie no changes.)
    err = uart_set_pin(EX_UART_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE);

    // Install UART driver, and get the queue.
    err = uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL,
                              0);
    // release the pre registered UART handler/subroutine
    err = uart_isr_free(EX_UART_NUM);
    //  register new UART ISR subroutine
    err = uart_isr_register(EX_UART_NUM, uart_intr_handle, NULL,
                            (ESP_INTR_FLAG_LEVEL3 | ESP_INTR_FLAG_IRAM),
                            &handle_console);
    // err = esp_intr_alloc(ETS_UART1_INTR_SOURCE, ESP_INTR_FLAG_IRAM |
    // ESP_INTR_FLAG_LEVEL3, uart_intr_handle,NULL, NULL); enable RX interrupt
    err = uart_enable_rx_intr(EX_UART_NUM);

    err                = uart_set_rx_timeout(EX_UART_NUM, 10);
    UART_ISR_Evt_Queue = xQueueCreate(256, sizeof(struct uart_slave_struct *));
    uart_mutex         = xSemaphoreCreateMutex();
    mdbslave_addr      = nvs_read_data_from_flash("slave_addr");
    return err;
}

/*********************************************************/
/********************Uart Receive Task********************/
/*********************************************************/
void modbus_slave_task(void *arg) {
    while (1) {
        xSemaphoreTake(uart_mutex, portMAX_DELAY);
        // ets_printf("uart task\n");
        receive_req_create_pdu();

        xSemaphoreGive(uart_mutex);
    }
    vSemaphoreDelete(uart_mutex);
    vTaskDelete(NULL);
}

/*********************************************************/
/***********Validate from Master Request ADU**************/
/*********************************************************/
uint8_t validate_req_hdr(uint8_t *data, uint8_t size) {
    unsigned short crc16;
    crc16       = Get_CRC16(data, (size - 2));
    CRCBytes[0] = crc16 & 0xff;
    CRCBytes[1] = crc16 >> 8;
    if (CRCBytes[0] != data[6] || CRCBytes[1] != data[7]) {
        ESP_LOGI(TAG, "invalid CRC value");
        return 0;
    }
    if (data[0] != mdbslave_addr) {
        ESP_LOGI(TAG, "wrong slave address");
        return 0;
    }
    slave_addr = data[0];
    return data[1];
}

/*********************************************************/
/****************Create to Slave Response*****************/
/*********************************************************/
void create_slave_response(uint8_t func_code, uint16_t startaddr,
                           uint16_t data) {
    uint8_t slave_adu[8];
    unsigned short crc16;
    uint8_t slave_len = 0;
    // slave_adu = (uint8_t *) malloc(9);
    if (WRITE_SINGLE_COIL == func_code || WRITE_SINGLE_REGISTER == func_code) {
        slave_adu[0] = slave_addr;
        slave_adu[1] = func_code;
        slave_adu[2] = (startaddr >> 8) & 0xff;
        slave_adu[3] = startaddr & 0xff;
        slave_adu[4] = (data >> 8) & 0xff;
        slave_adu[5] = data & 0xff;
        crc16        = Get_CRC16(slave_adu, 6);
        slave_adu[6] = crc16 & 0xff;
        slave_adu[7] = crc16 >> 8;
        slave_len    = 8;
    } else if (READ_HOLDING_REGISTERS == func_code) {
        slave_adu[0] = slave_addr;
        slave_adu[1] = func_code;
        slave_adu[2] = 0x02;
        slave_adu[3] = (data >> 8) & 0xff;
        slave_adu[4] = data & 0xff;
        crc16        = Get_CRC16(slave_adu, 5);
        slave_adu[5] = crc16 & 0xff;
        slave_adu[6] = crc16 >> 8;
        slave_len    = 7;
    } else if (READ_COIL_STATUS == func_code) {
        slave_adu[0] = slave_addr;
        slave_adu[1] = func_code;
        slave_adu[2] = 0x01;
        slave_adu[3] = data & 0xff;
        crc16        = Get_CRC16(slave_adu, 4);
        slave_adu[4] = crc16 & 0xff;
        slave_adu[5] = crc16 >> 8;
        slave_len    = 6;
    }
    // use this uart_tx_chars
    uart_tx_chars(EX_UART_NUM, (char *)slave_adu, slave_len);
}

/*********************************************************/
/*********Check the Function Code, Address, Datas*********/
/*********************************************************/
uint8_t push_datavalue(uint8_t func_code, uint16_t startaddr, uint16_t data) {
    if (WRITE_SINGLE_COIL == func_code) {
        if (WCSSR_ADDR == startaddr) {
            if (data == 0xff00) {
                ctrl_set_op(1);
                coil_status = 0x0001;
            } else if (data == 0x00) {
                ctrl_set_op(0);
                coil_status = 0x0000;
            }
            return 1;
        }
    } else if (WRITE_SINGLE_REGISTER == func_code) {
        if (WRLED_ADDR == startaddr) {
            flash_color = (((((data >> 11 & 0x1f) * 0xff) / 0x1f) << 16) |
                           ((((data >> 5 & 0x3f) * 0xff) / 0x3f) << 8) |
                           (((data & 0x1f) * 0xff) / 0x1f));
            // Led_Ctrl_State(1, 0, 0, flash_color);
            neo_color_all(flash_color, CONFIG_LED_BRIGHTNESS);
            return 1;
        } else if (WRSLAVE_ADDR == startaddr) {
            nvs_write_data_to_flash("slave_addr", (uint8_t)data);
            return 1;
        }
    } else if (READ_COIL_STATUS == func_code) {
        if (WCSSR_ADDR == startaddr) {
            return 1;
        }
    } else if (READ_HOLDING_REGISTERS == func_code) {
        if (WRVER_ADDR == startaddr || WRLED_ADDR == startaddr ||
            WRSLAVE_ADDR == startaddr) {
            return 1;
        }
    }

    return 0;
}

/*********************************************************/
/****Validate Master Request and Create Slave Response****/
/*********************************************************/
void receive_req_create_pdu(void) {
    uart_data *uart_recv_data;
    uart_recv_data = &uart_reg_data;
    if (xQueueReceive(UART_ISR_Evt_Queue, (void *)&uart_recv_data,
                      portMAX_DELAY)) {
        function_code = validate_req_hdr(&uart_recv_data->recv_buff[0],
                                         uart_recv_data->buff_length);
        reg_address   = byte_2_int_big(&uart_recv_data->recv_buff[2]);
        quantity      = 1;
        if (WRITE_SINGLE_COIL == function_code ||
            WRITE_SINGLE_REGISTER == function_code) {
            uint16_t intdata = byte_2_int_big(&uart_recv_data->recv_buff[4]);
            if (push_datavalue(function_code, reg_address, intdata))
                create_slave_response(function_code, reg_address, intdata);
        } else if (READ_HOLDING_REGISTERS == function_code) {
            uint16_t intdata = 0;
            if (WRLED_ADDR == reg_address) {
                intdata = (((((flash_color >> 16) & 0xff) >> 3) << 11) |
                           ((((flash_color >> 8) & 0xff) >> 2) << 5) |
                           ((flash_color & 0xff) >> 3));
            } else if (WRVER_ADDR == reg_address) {
                intdata = (uint16_t)CONFIG_DEVICE_VERSION;
            } else if (WRSLAVE_ADDR == reg_address) {
                intdata = nvs_read_data_from_flash("slave_addr");
            }
            if (push_datavalue(function_code, reg_address, intdata))
                create_slave_response(function_code, reg_address, intdata);
        } else if (READ_COIL_STATUS == function_code) {
            if (WCSSR_ADDR == reg_address) {
                if (push_datavalue(function_code, reg_address, coil_status))
                    create_slave_response(function_code, reg_address,
                                          coil_status);
            }
        }
    }
}

/*********************************************************/
/**********Bytes to Big Endian Integer Converter**********/
/*********************************************************/
uint16_t byte_2_int_big(uint8_t *bytes) {
    return (bytes[1] + (bytes[0] << 8));
}

/*********************************************************/
/*****************Check the Modbus CRC16******************/
/*********************************************************/
uint16_t Get_CRC16(volatile uint8_t *ptr, uint8_t len) {
    unsigned char i;
    uint16_t crc = 0xFFFF;
    if (len == 0) len = 1;

    while (len--) {
        crc ^= *ptr;
        for (i = 0; i < 8; i++) {
            if (crc & 1) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
        ptr++;
    }
    return (crc);
}
