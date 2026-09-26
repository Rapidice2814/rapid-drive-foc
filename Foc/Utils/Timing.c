#include "Timing.h"
#include "main.h"

TIM_HandleTypeDef* hfunction_timer;

void FunctionTimer_Init(TIM_HandleTypeDef *htim){
    hfunction_timer = htim;
    HAL_TIM_Base_Start(hfunction_timer);
}

uint32_t get_current_time(){
    if(hfunction_timer == NULL) return 0;
    return __HAL_TIM_GET_COUNTER(hfunction_timer);
}

void calculate_execution_time(uint32_t *max_time, uint32_t start_time){
    uint32_t execution_time = get_current_time() - start_time;
    if (execution_time > *max_time) {
        *max_time = execution_time;
    }
}