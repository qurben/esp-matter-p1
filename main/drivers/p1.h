/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

// This file implements the PIR sensor driver.
// This is implemented keeping the Matter requirements in mind.
#pragma once

#include <esp_err.h>

using p1_sensor_cb_t = void (*)(uint16_t endpoint_id, uint16_t value, void *user_data);

typedef struct {
    struct {
        p1_sensor_cb_t cb = NULL;
        uint16_t endpoint_id;
    } usage;

    // user data
    void *user_data = NULL;

    // polling interval in milliseconds, defaults to 5000 ms
    uint32_t interval_ms = 5000;
} p1_sensor_config_t;

/**
 * @brief Initialize sensor driver. This function should be called only once
 *
 * @param config sensor configurations. This should last for the lifetime of the driver
 *               as driver layer do not make a copy of this object.
 *
 * @return esp_err_t - ESP_OK on success,
 *                     ESP_ERR_INVALID_ARG if config is NULL
 *                     ESP_ERR_INVALID_STATE if driver is already initialized
 *                     appropriate error code otherwise
 */
esp_err_t p1_sensor_init(p1_sensor_config_t *config);
