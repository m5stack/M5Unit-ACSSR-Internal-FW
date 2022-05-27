#include "boardio.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "neopixel.h"
#include "nvs_memory.h"

#define GPIO_BUTTON           CONFIG_PUSH_BUTTON
#define GPIO_CONTROL          CONFIG_OUTPUT_CONTROL
#define GPIO_OUTPUT_PIN_SEL   (1ULL << GPIO_CONTROL)
#define GPIO_INPUT_PIN_SEL    (1ULL << GPIO_BUTTON)
#define ESP_INTR_FLAG_DEFAULT 0

#define LONG_PRESS_TIME 2000

static const char* TAG = "GPIO-IO";

static QueueHandle_t gpio_evt_queue = NULL;
SemaphoreHandle_t button_semphr     = NULL;
SemaphoreHandle_t led_mutex         = NULL;

int flash_delay         = 200;
uint8_t flash_enable    = 0;
uint32_t flash_color    = 0;
uint8_t led_task_enable = 0;

BaseType_t Press_Key      = pdFALSE;
BaseType_t Long_Press_Key = pdFALSE;
int64_t Holdtime          = 0;

uint8_t op_state = 0;

/*********************************************************/
/*********************ISR GPIO Handler********************/
/*********************************************************/
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t)arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

int64_t millis() {
    return esp_timer_get_time() / 1000;
}

// QueueHandle_t create_button_queue(void)
// {
//     button_Tx_queue = xQueueCreate(2, sizeof(uint32_t));
//     return button_Tx_queue;
// }

esp_err_t key_scan(void) {
    while (1) {
        if (gpio_get_level(GPIO_BUTTON) == 0 && Press_Key == pdFALSE) {
            Press_Key      = pdTRUE;
            Long_Press_Key = pdFALSE;
            Holdtime       = millis();
        } else if (gpio_get_level(GPIO_BUTTON) == 0 && Press_Key == pdTRUE &&
                   Long_Press_Key == pdFALSE &&
                   LONG_PRESS_TIME < (millis() - Holdtime)) {
            ESP_LOGI(TAG, " Hold Time: %lld ms ", (millis() - Holdtime));
            Holdtime       = 0;
            Long_Press_Key = pdTRUE;
            return LONG_PRESS;
            // Release_Key = pdTRUE;
            // ESP_LOGI(TAG, " Hold Time: %lld ms " , Holdtime);
        } else if (gpio_get_level(GPIO_BUTTON) == 1 && Press_Key == pdTRUE &&
                   Long_Press_Key == pdTRUE) {
            Press_Key = pdFALSE;
        } else if (gpio_get_level(GPIO_BUTTON) == 1 && Press_Key == pdTRUE &&
                   Long_Press_Key == pdFALSE && (100 < (millis() - Holdtime))) {
            ESP_LOGI(TAG, " Hold Time: %lld ms ", (millis() - Holdtime));
            Holdtime  = 0;
            Press_Key = pdFALSE;
            return SHORT_PRESS;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void Led_Ctrl_State(uint8_t task_en, uint8_t flash_en, int delay,
                    uint32_t color) {
    led_task_enable = task_en;
    flash_enable    = flash_en;
    flash_delay     = delay;
    flash_color     = color;
}

void LED_Ctrl_Task(void* parameter) {
    static bool rgbState = 0;
    while (1) {
        xSemaphoreTake(led_mutex, portMAX_DELAY);
        // vPortEnterCritical();
        if (led_task_enable) {
            if (flash_enable) {
                neo_color_all((flash_color * rgbState), CONFIG_LED_BRIGHTNESS);
                vTaskDelay(flash_delay / portTICK_PERIOD_MS);
                rgbState = !rgbState;
            } else {
                neo_color_all(flash_color, CONFIG_LED_BRIGHTNESS);
                vTaskDelay(10 / portTICK_PERIOD_MS);
            }
        } else {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        // vPortExitCritical();
        xSemaphoreGive(led_mutex);
    }
    vSemaphoreDelete(led_mutex);
    vTaskDelete(NULL);
}

void button_task(void* arg) {
    uint32_t io_num;
    uint8_t current_level = 0;
    uint8_t last_level    = 0;
    while (1) {
        xSemaphoreTake(button_semphr, portMAX_DELAY);
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            current_level = gpio_get_level(io_num);
            if (current_level == 1 && last_level == 0) {
                op_state = !op_state;
                ctrl_set_op((op_state));
                if (op_state)
                    // Led_Ctrl_State(1, 0, CONFIG_SLOW_FLASH, RED_COLOR);
                    neo_color_all(RED_COLOR, CONFIG_LED_BRIGHTNESS);
                else
                    // Led_Ctrl_State(1, 0, CONFIG_SLOW_FLASH, GREEN_COLOR);
                    neo_color_all(GREEN_COLOR, CONFIG_LED_BRIGHTNESS);
            }
            last_level = current_level;
        }
        xSemaphoreGive(button_semphr);
    }
    vSemaphoreDelete(button_semphr);
    vTaskDelete(NULL);
}

/*********************************************************/
/******************Control the Drive GPIO*****************/
/*********************************************************/
void ctrl_set_op(char dir) {
    if (dir) {
        gpio_set_level(GPIO_CONTROL, 1);
        // Led_Ctrl_State(1, 0, CONFIG_SLOW_FLASH, RED_COLOR);
        ESP_LOGI(TAG, "**DRIVER ON**");
    } else {
        gpio_set_level(GPIO_CONTROL, 0);
        // Led_Ctrl_State(1, 0, CONFIG_SLOW_FLASH, GREEN_COLOR);
        ESP_LOGI(TAG, "**DRIVER OFF**");
    }
    // nvs_write_data_to_flash(1,(int8_t)dir);
}

/*********************************************************/
/********Board Drive and Control GPIO Initialized*********/
/*********************************************************/
void boardio_init(uint8_t setup) {
    if (setup == 1) {
        gpio_config_t io_conf;
        // disable interrupt
        io_conf.intr_type = GPIO_INTR_DISABLE;
        // bit mask of the pins, use GPIO3 here
        io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
        // set as input mode
        io_conf.mode = GPIO_MODE_INPUT;
        // enable pull-up mode
        io_conf.pull_up_en = 1;
        gpio_config(&io_conf);

        // disable interrupt
        io_conf.intr_type = GPIO_INTR_DISABLE;
        // set as output mode
        io_conf.mode = GPIO_MODE_OUTPUT;
        // bit mask of the pins that you want to set,e.g.GPIO4
        io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
        // disable pull-down mode
        io_conf.pull_down_en = 0;
        // disable pull-up mode
        io_conf.pull_up_en = 0;
        // configure GPIO with the given settings
        gpio_config(&io_conf);

        button_semphr = xSemaphoreCreateMutex();
        led_mutex     = xSemaphoreCreateMutex();
    } else if (setup == 2) {
        gpio_config_t io_conf;
        // interrupt of any edge
        io_conf.intr_type = GPIO_INTR_ANYEDGE;
        // bit mask of the pins, use GPIO3 here
        io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
        // set as input mode
        io_conf.mode = GPIO_MODE_INPUT;
        // enable pull-up mode
        io_conf.pull_up_en = 1;
        // disable pull-down mode
        io_conf.pull_down_en = 0;
        gpio_config(&io_conf);

        gpio_evt_queue = xQueueCreate(2, sizeof(uint32_t));

        // install gpio isr service
        gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
        // hook isr handler for specific gpio pin
        gpio_isr_handler_add(GPIO_BUTTON, gpio_isr_handler, (void*)GPIO_BUTTON);
    }
}