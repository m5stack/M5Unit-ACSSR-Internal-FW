#pragma once
#ifdef __cplusplus
extern "C" {
#endif
#include "esp_system.h"
#include "esp_log.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs.h"
#include "nvs_flash.h"

void nvs_write_data_to_flash(char *key, uint8_t value);
uint8_t nvs_read_data_from_flash(char *key);

#ifdef __cplusplus
}
#endif