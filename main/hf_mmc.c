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

static QueueHandle_t algo_queue;
//bunu sadece suanlik boyle yaptim sonra sensordan okuyup ekleyecegim
void sensor_task(void *pvParameter) {
    while (1) {
        algo_input_t input;
        int temp[20] = {
            250,265,279,290,298,
            300,298,290,279,265,
            250,234,221,210,202,
            200,202,210,221,234
        };
        memcpy(input.arr, temp, sizeof(temp));
        input.size = 20;
        input.threshold = 10;
        xQueueSend(algo_queue, &input, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void algo_task(void *pvParameter) {
    algorithm_t chosen = BLOCKM;
    algo_func_t func = get_algorithm(chosen);
    algo_input_t input;
    while (1) {
        if (xQueueReceive(algo_queue, &input, portMAX_DELAY) == pdPASS) {
            if (func) {
                bool result = func(input.arr, input.size, input.threshold);
                printf("Algorithm returned: %s\n", result ? "true" : "false");
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

    algo_queue = xQueueCreate(5, sizeof(algo_input_t));
    if (!algo_queue) return;

    xTaskCreate(sensor_task, "sensor_task", 2048, NULL, 5, NULL);
    xTaskCreate(algo_task, "algo_task", 2048, NULL, 5, NULL);
}
