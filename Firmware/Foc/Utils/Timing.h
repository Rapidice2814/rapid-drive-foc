#ifndef TIMING_H
#define TIMING_H

#include "main.h"

typedef struct {
    uint32_t adc_max;
    uint32_t log_max;
    uint32_t state_max;
    uint32_t usb_debug_max;
    uint32_t loop_max;
} ExecutionTimeTypeDef;

void FunctionTimer_Init();
uint32_t get_current_time();
void calculate_execution_time(uint32_t *max_time, uint32_t start_time);

#endif // TIMING_H