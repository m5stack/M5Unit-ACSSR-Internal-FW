#include "nvs_memory.h"
#include "esp_err.h"
#include "esp_log.h"

esp_err_t err;

/*
key1 = "mode"
key2 = "ctrl"
key3 = "i2c_addr"
key4 = "slave_addr"
*/

void nvs_write_data_to_flash(char *key, uint8_t value) {
    nvs_handle handle;
    const char NVS_SELECT[] = "SELECT_MODE";
    ESP_ERROR_CHECK(nvs_open(NVS_SELECT, NVS_READWRITE, &handle));
    ESP_ERROR_CHECK(nvs_set_u8(handle, key, value));
    ESP_ERROR_CHECK(nvs_commit(handle));
    nvs_close(handle);
}

uint8_t nvs_read_data_from_flash(char *key) {
    nvs_handle handle;
    const char NVS_SELECT[] = "SELECT_MODE";
    uint8_t value           = 0;

    ESP_ERROR_CHECK(nvs_open(NVS_SELECT, NVS_READWRITE, &handle));
    ESP_ERROR_CHECK(nvs_get_u8(handle, key, &value));
    nvs_close(handle);
    return value;
}