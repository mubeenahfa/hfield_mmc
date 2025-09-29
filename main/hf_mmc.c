#include <stdio.h>
#include <string.h>
#include "algo.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"


#define MOS_RED   GPIO_NUM_18
#define MOS_GREEN GPIO_NUM_19

typedef struct {
    int arr[20];
    size_t size;
    int threshold;
} algo_input_t;

static QueueHandle_t x_queue, y_queue, z_queue;


//bunu sadece suanlik boyle yaptim sonra sensordan okuyup ekleyecegim
void sensor_task_dummy(void *pvParameter) {
    while (1) {
        mmc_meas_t meas;
        
        // Example waveforms for X, Y, Z (20-step pattern reused)
        static int step = 0;
        int pattern[20] = {
            250,265,279,290,298,
            300,298,290,279,265,
            250,234,221,210,202,
            200,202,210,221,234
        };

        meas.mX = (float)pattern[step];
        meas.mY = (float)pattern[(step + 5) % 20];   // phase shifted
        meas.mZ = (float)pattern[(step + 10) % 20];  // different phase shift

        // Broadcast to all axis queues
        xQueueSend(x_queue, &meas, 0);
        xQueueSend(y_queue, &meas, 0);
        xQueueSend(z_queue, &meas, 0);

        step = (step + 1) % 20; // cycle through pattern
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void sensor_task(void *pvParameter) {
    while (1) {
        mmc_meas_t meas;
        if (mmc_read_meas() == ESP_OK) {
            mmc_get_meas(&meas);
            xQueueSend(x_queue, &meas, 0);
            xQueueSend(y_queue, &meas, 0);
            xQueueSend(z_queue, &meas, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(200)); // sampling rate
    }
}

void axis_task(void *pvParameter) {
    QueueHandle_t q = (QueueHandle_t)pvParameter;
    algorithm_t chosen = BLOCKM;
    algo_func_t func = get_algorithm(chosen);

    while (1) {
        mmc_meas_t meas;
        if (xQueueReceive(q, &meas, portMAX_DELAY) == pdPASS) {
            int value = 0;

            if (q == x_queue) value = (int)meas.mX;
            else if (q == y_queue) value = (int)meas.mY;
            else if (q == z_queue) value = (int)meas.mZ;

            bool result = func(&value, 1, 10);

            if (result) {
                gpio_set_level(MOS_RED, 1);
                gpio_set_level(MOS_GREEN, 0);
            } else {
                gpio_set_level(MOS_RED, 0);
                gpio_set_level(MOS_GREEN, 1);
            }
        }
    }
}

void app_main(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MOS_RED) | (1ULL << MOS_GREEN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    if (mmc_init() != ESP_OK) {
    printf("MMC init failed!\n");
    return;
    }

    x_queue = xQueueCreate(5, sizeof(mmc_meas_t));
    y_queue = xQueueCreate(5, sizeof(mmc_meas_t));
    z_queue = xQueueCreate(5, sizeof(mmc_meas_t));
    if (!x_queue || !y_queue || !z_queue) return;

    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 5, NULL);
    xTaskCreate(axis_task, "x_task", 4096, (void *)x_queue, 5, NULL);
    xTaskCreate(axis_task, "y_task", 4096, (void *)y_queue, 5, NULL);
    xTaskCreate(axis_task, "z_task", 4096, (void *)z_queue, 5, NULL);
}
