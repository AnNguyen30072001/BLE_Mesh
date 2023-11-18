/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

#define TAG "TEST"

/**
 * Fake data array to send to server. The order of data:
 * test_data_arr[0]: room number
 * test_data_arr[1]: battery percentage
 * test_data_arr[2]: Temperature value
 * test_data_arr[3]: Smoke value
 * test_data_arr[4]: Temperature alarm flag. Value is 1 if temperature is too high => fire
 * test_data_arr[5]: Flame sensor alarm flag. Value is 1 if detect flame => fire
 * test_data_arr[6]: Smoke alarm flag. Value is 1 if detect smoke is too high => fire
 */
uint8_t test_data_arr[7] = {101, 85, 29, 45, 0, 0, 0};

// Delay for a number of seconds
void delay_seconds(uint32_t seconds) {
    vTaskDelay(seconds * portTICK_PERIOD_MS * 10);
}

void app_main(void)
{
    while(1) {
        for(int i=0; i<7; i++) {
            printf("%d\n", test_data_arr[i]);
        }
        delay_seconds(5);
    }

    /* Neu code tren ko chay, dung code nay */
    // while(1) {
    //     for(int i=0; i<7; i++) {
    //         ESP_LOGW(TAG, "%d\n", test_data_arr[i]);
    //     }
    //     delay_seconds(5);
    // }

}
