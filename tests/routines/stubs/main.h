#ifndef TEST_MAIN_H
#define TEST_MAIN_H
#include <stdint.h>
#include <stddef.h>
#define UNUSED(x) ((void)(x))
#define CURRENT_LOOP_FREQUENCY 8000
#define SPEED_LOOP_CLOCK_DIVIDER 8
#define ENCODER_PULSES_PER_ROTATION 4096
typedef struct {int unused;} FDCAN_HandleTypeDef;
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t);
#endif
