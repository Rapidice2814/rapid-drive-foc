#ifndef WS2812B_DRIVER_H
#define WS2812B_DRIVER_H

#include "main.h"

#define WS2812B_NUMBER_OF_LEDS 4


void WS2812b_Setup(TIM_HandleTypeDef *htim, uint32_t channel);
void WS2812b_SetColor(uint8_t led, uint8_t r, uint8_t g, uint8_t b);
void WS2812b_Send();
void WS2812b_PulseFinishedCallback();


#endif // WS2812B_DRIVER_H