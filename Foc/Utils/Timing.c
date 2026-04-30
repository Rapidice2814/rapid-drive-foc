#include "Timing.h"

extern TIM_HandleTypeDef htim2;

void FunctionTimer_Init(){
    HAL_TIM_Base_Start(&htim2);
}

uint32_t get_current_time(){
    return __HAL_TIM_GET_COUNTER(&htim2);
}

void calculate_execution_time(uint32_t *max_time, uint32_t start_time){
    uint32_t execution_time = get_current_time() - start_time;
    if (execution_time > *max_time) {
        *max_time = execution_time;
    }
}