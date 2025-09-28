// algo.c
#include "algo.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stddef.h>   // for size_t
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
//gotta decide if I will allow floats here or will manage it with ints all the way

#define FS 1000.0 // sample rate in Hz 
#define F0 50.0 // target frequency (Hz)
/*
static bool algo_a(int *arr, size_t size, int threshold) {
    printf("Running Algorithm A\n");
    for (size_t i = 0; i < size; i++) {
        if (arr[i] > threshold) {
            return true;
        }
    }
    return false;
}
*/

static bool minmax_nodc(int *buf, int N, int threshold_vpp) {
    printf("Running simple minmax with no dc removal method\n");
    if (N <= 0) return false;

    float min_val = buf[0];
    float max_val = buf[0];

    for (int n = 1; n < N; n++) {
        float y = buf[n];
        if (y < min_val) min_val = y;
        if (y > max_val) max_val = y;
    }

    float Vpp = max_val - min_val;
    printf("Vpp = %.2f\n", Vpp);
    //for debugging im printing will remove later

    return (Vpp >= threshold_vpp);
}

static bool minmax(int *buf, int N, int threshold_vpp) {
    printf("Running simple minmax method\n");
    if (N <= 0) return false;

    //burada dc silmek icin average aldim
    float sum = 0.0f;
    for (int n = 0; n < N; n++) {
        sum += buf[n];
    }
    float mu = sum / (float)N;


    float min_val = buf[0] - mu;
    float max_val = min_val;
    for (int n = 1; n < N; n++) {
        float y = buf[n] - mu;
        if (y < min_val) min_val = y;
        if (y > max_val) max_val = y;
    }


    float Vpp = max_val - min_val;
    printf("Vpp = %.2f\n", Vpp);


    return (Vpp >= threshold_vpp);
}

static bool block_method(int *buf, int size, float threshold_vpp) {
    printf("Running Algorithm Block method\n");
    if (size <= 0) return false;

    //burda dc siliyorum. istemiyorsaniz komentleyin (eger filtreli bir sey kulaniyorsaniz)
    float sum = 0.0f;
    for (int n = 0; n < size; n++) {
        sum += (float)buf[n];
    }
    float mu = sum / (float)size;

    float S = 0.0f, C = 0.0f;
    for (int n = 0; n < size; n++) {
        float y = (float)buf[n] - mu;
        float t = (float)n / FS;
        float w = 2.0f * M_PI * F0 * t;
        S += y * sinf(w);
        C += y * cosf(w);
    }

    float A_peak = (2.0f / (float)size) * sqrtf(S*S + C*C);
    float Vpp = 2.0f * A_peak;

    printf("Vpp = %.3f\n", Vpp);

    return (Vpp >= threshold_vpp);
}


static bool goertzel(int *buf, int N, float threshold_vpp) {
    printf("Running Algorithm goertzel\n");
    if (N <= 0) return false;

    //burda dc komponent hesapliyorum
    float sum = 0.0f;
    for (int n = 0; n < N; n++) {
        sum += (float)buf[n];
    }
    float mu = sum / (float)N;

    int k = (int)lroundf(F0 * (float)N / FS);  
    float w = 2.0f * M_PI * (float)k / (float)N;
    float coeff = 2.0f * cosf(w);

    float s0, s1 = 0.0f, s2 = 0.0f;
    for (int n = 0; n < N; n++) {
        float x = (float)buf[n] - mu;   // DC burda siliyom
        s0 = x + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }

    float real = s1 - s2 * cosf(w);
    float imag = s2 * sinf(w);
    float mag  = sqrtf(real*real + imag*imag);

    float A_peak = (2.0f / (float)N) * mag;
    float Vpp = 2.0f * A_peak;

    printf("Vpp = %.3f\n", Vpp);

    return (Vpp >= threshold_vpp);
}

algo_func_t get_algorithm(algorithm_t algo) {
    switch (algo) {
        case MINMAXNODC: return minmax_nodc;
        case MINMAX: return minmax;
        case GOERTZEL: return goertzel;
        case BLOCKM: return block_method;
        default:     return NULL;
    }
}

