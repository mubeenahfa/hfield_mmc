// algo.h
#pragma once
#include <stdbool.h>
#include <stdio.h>
#include <stddef.h>
#include <math.h>

typedef enum {
    MINMAXNODC,
    MINMAX,
    GOERTZEL,
    BLOCKM
} algorithm_t;


typedef bool (*algo_func_t)(int *arr, size_t size, int threshold);


algo_func_t get_algorithm(algorithm_t algo);
