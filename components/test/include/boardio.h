#pragma once
#ifdef __cplusplus
extern "C" {
#endif
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

extern int flash_delay;
extern uint8_t flash_enable;
extern uint32_t flash_color;
extern uint8_t led_task_enable;

extern uint8_t op_state;

void boardio_init(uint8_t setup);
void ctrl_set_op(char dir);
// void Button_Press_Task(void* arg);
void button_task(void* arg);
esp_err_t key_scan(void);
// QueueHandle_t create_button_queue(void);
void LED_Ctrl_Task(void* parameter);
void Led_Ctrl_State(uint8_t task_en, uint8_t flash_en, int delay,
                    uint32_t color);
int64_t millis();

typedef enum {
    SHORT_PRESS = 1,
    LONG_PRESS  = 2,

} key_type_t;

#ifdef __cplusplus
}
#endif
