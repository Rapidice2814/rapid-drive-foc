#ifndef FOC_LOOPS_H
#define FOC_LOOPS_H

#include "main.h"
#include "FOC_Driver.h"

uint8_t FOC_MotorIdentification(FOC_HandleTypeDef *hfoc);
uint8_t FOC_OpenLoop(FOC_HandleTypeDef *hfoc, float espeed, float magnitude, float loop_frequency);

#endif // FOC_LOOPS_H