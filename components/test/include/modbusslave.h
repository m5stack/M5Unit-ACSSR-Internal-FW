#pragma once
#ifdef __cplusplus
extern "C" {
#endif

#define RX_BUF_SIZE 1024

#define READ_COIL_STATUS         01
#define READ_INPUT_STATUS        02
#define READ_HOLDING_REGISTERS   03
#define READ_INPUT_REGISTERS     04
#define WRITE_SINGLE_COIL        05
#define WRITE_SINGLE_REGISTER    06
#define WRITE_MULTIPLE_COILS     15
#define WRITE_MULTIPLE_REGISTERS 16
#define MAX_REGI_SIZE            256

typedef struct {
    uint8_t addr;
    uint8_t cmd;
    uint8_t *data;  // big encoder
    uint16_t crc;
    uint16_t reg_addr;  // big encoder
    uint16_t len;
} modbus_frame_t;

typedef struct uart_slave_struct {
    uint8_t buff_length;
    uint8_t recv_buff[MAX_REGI_SIZE];
} uart_data;

uart_data uart_reg_data;

esp_err_t modbusuart_init(void);
void modbus_slave_task(void *arg);
void receive_req_create_pdu(void);
unsigned short Get_CRC16(volatile unsigned char *ptr, unsigned char len);
uint8_t push_datavalue(uint8_t func_code, uint16_t startaddr, uint16_t data);
unsigned short byte_2_int_big(uint8_t *bytes);

#ifdef __cplusplus
}
#endif
