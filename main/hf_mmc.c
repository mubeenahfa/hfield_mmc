#include <stdio.h>
#include "algo.h"

void app_main(void) {
    int data[20] = {
    250, 265, 279, 290, 298,
    300, 298, 290, 279, 265,
    250, 234, 221, 210, 202,
    200, 202, 210, 221, 234
};
    size_t size = sizeof(data) / sizeof(data[0]);
    int threshold = 10;

    algorithm_t chosen = BLOCKM;   // could also be set via Kconfig / runtime input
    algo_func_t func = get_algorithm(chosen);

    if (func) {
        bool result = func(data, size, threshold);
        printf("Algorithm returned: %s\n", result ? "true" : "false");
    } else {
        printf("Invalid algorithm!\n");
    }
}