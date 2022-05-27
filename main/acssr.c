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
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_system.h"
#include <esp_task_wdt.h>
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "boardio.h"
#include "i2c_slave.h"
#include "modbusslave.h"
#include "neopixel.h"
#include "nvs_memory.h"
#include "esp_timer.h"

#define UDATA_LENGTH 1024
#define TOTAL_MENU   3

#define SLOW_FLASH CONFIG_SLOW_FLASH
#define FAST_FLASH CONFIG_FAST_FLASH

#define SETUP_RETRY_TIMER 4000

static const char *TAG      = "AC-SSR";
SemaphoreHandle_t ssr_mutex = NULL;
SemaphoreHandle_t io_mutex  = NULL;

static TaskHandle_t task_i2c    = NULL;
static TaskHandle_t task_modbus = NULL;
static TaskHandle_t task_led    = NULL;

TimerHandle_t setup_retry_timer = NULL;

// static QueueHandle_t button_Tx_queue = NULL;

uint8_t state_op = 0;

typedef enum {
    KNOWN_MODE = 0,
    SETUP_MODE = 1,
    RUN_MODE   = 2,
} menu_type_t;

typedef enum {
    BUTTON_MENU = 1,
    I2C_MENU    = 2,
    MODBUS_MENU = 3,
} sub_menu_type_t;

uint8_t Current_Mode  = 0;
uint8_t Selected_Menu = 0;
extern uint32_t flash_color;
extern uint8_t flash_enable;
extern bool AP_Mode;

extern uint8_t op_state;

void SSR_Setup_Task(void) {
    esp_err_t level   = 0;
    uint8_t Next_Menu = 0;
    while (1) {
        xSemaphoreTake(ssr_mutex, portMAX_DELAY);
        level = key_scan();
        if (level == -1) vTaskDelete(NULL);
        if (level == LONG_PRESS && Current_Mode == RUN_MODE) {
            Current_Mode = SETUP_MODE;
            Led_Ctrl_State(1, 1, FAST_FLASH, GREEN_COLOR);
            Next_Menu = 1;
        } else if (level == SHORT_PRESS && Current_Mode == RUN_MODE) {
            vSemaphoreDelete(ssr_mutex);
            break;
        } else if (level == LONG_PRESS && Current_Mode == SETUP_MODE) {
            flash_enable = 0;
            nvs_write_data_to_flash("ctrl", (int8_t)0);
            switch (Next_Menu) {
                case BUTTON_MENU:
                    ESP_LOGI(TAG, "BUTTON TASK");
                    nvs_write_data_to_flash("mode", (int8_t)BUTTON_MENU);
                    esp_restart();
                    break;
                case I2C_MENU:
                    ESP_LOGI(TAG, "I2C TASK");
                    nvs_write_data_to_flash("mode", (int8_t)I2C_MENU);
                    esp_restart();
                    break;
                case MODBUS_MENU:
                    ESP_LOGI(TAG, "MODBUS TASK");
                    nvs_write_data_to_flash("mode", (int8_t)MODBUS_MENU);
                    esp_restart();
                    break;
                default:
                    break;
            }
        } else if (level == SHORT_PRESS && Current_Mode == SETUP_MODE) {
            Next_Menu++;
            if (TOTAL_MENU < Next_Menu) Next_Menu = 1;
            switch (Next_Menu) {
                case BUTTON_MENU:
                    flash_color = GREEN_COLOR;
                    break;
                case I2C_MENU:
                    flash_color = RED_COLOR;
                    break;
                case MODBUS_MENU:
                    flash_color = YELLOW_COLOR;
                    break;
                default:
                    break;
            }
            Led_Ctrl_State(1, 1, FAST_FLASH, flash_color);
        }
        xSemaphoreGive(ssr_mutex);
    }
}

void initial_control(void) {
    uint8_t select_ctrl;
    Selected_Menu = nvs_read_data_from_flash("mode");
    // select_ctrl = nvs_read_data_from_flash(1);
    select_ctrl = 0;
    if (Selected_Menu == BUTTON_MENU) {
        flash_color = GREEN_COLOR;
    } else if (Selected_Menu == I2C_MENU) {
        flash_color = RED_COLOR;
    } else if (Selected_Menu == MODBUS_MENU) {
        flash_color = YELLOW_COLOR;
    }
    Led_Ctrl_State(1, 1, SLOW_FLASH, flash_color);
    vTaskDelay(pdMS_TO_TICKS(3000));
    if (Selected_Menu != BUTTON_MENU) {
        flash_color = 0x00;
        neo_color_all(flash_color, CONFIG_LED_BRIGHTNESS);
    }
    Led_Ctrl_State(1, 0, SLOW_FLASH, flash_color);
    ctrl_set_op(select_ctrl);
    op_state = select_ctrl;
}

void app_main(void) {
    ssr_mutex    = xSemaphoreCreateMutex();
    io_mutex     = xSemaphoreCreateMutex();
    Current_Mode = RUN_MODE;
    // NVS Flash Initialized and Read Flash
    ESP_ERROR_CHECK(nvs_flash_init());
    // nvs_write_data_to_flash(0,(int8_t)1);
    // nvs_write_data_to_flash(1,(int8_t)1);

    // Button Input and Drive Output Initialized
    ESP_LOGI(TAG, "Button Input and Drive Output Initialized\n");
    boardio_init(1);

    // Ws2812 NeoPixel Initialized
    ESP_LOGI(TAG, "Ws2812 NeoPixel Initialized\n");
    neopixel_init();

    esp_timer_init();

    xTaskCreatePinnedToCore(LED_Ctrl_Task, "LED_Ctrl_Task", 2048, NULL, 9,
                            &task_led, 0);
    if (gpio_get_level(CONFIG_PUSH_BUTTON) == 0) SSR_Setup_Task();

    initial_control();
    if (task_led != NULL) vTaskDelete(task_led);

    if (Current_Mode == RUN_MODE && gpio_get_level(CONFIG_PUSH_BUTTON) == 1) {
        if (Selected_Menu == BUTTON_MENU) {
            ESP_LOGI(TAG, "Button Task is Selected\n");
            boardio_init(2);
            xTaskCreatePinnedToCore(button_task, "button_task", 1024 * 2, NULL,
                                    10, NULL, 0);
        } else if (Selected_Menu == I2C_MENU) {
            ESP_LOGI(TAG, "I2C Slave Task is Selected\n");
            // Slave Mode I2C Initialized
            ESP_LOGI(TAG, "Slave Mode I2C Initialized\n");
            ESP_ERROR_CHECK(i2c_slave_init());
            xTaskCreatePinnedToCore(i2c_slave_task, "i2c_slave_task", 1024 * 2,
                                    NULL, 10, &task_i2c, 0);
        } else if (Selected_Menu == MODBUS_MENU) {
            ESP_LOGI(TAG, "Modbus Task is Selected\n");
            // Uart and Modbus Initialized
            ESP_LOGI(TAG, "Uart and Modbus Initialized\n");
            ESP_ERROR_CHECK(modbusuart_init());
            xTaskCreatePinnedToCore(modbus_slave_task, "modbus_slave_task",
                                    1024 * 2, NULL, 10, &task_modbus, 0);
        }
    }
}
