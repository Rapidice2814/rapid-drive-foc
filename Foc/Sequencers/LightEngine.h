#ifndef LiGHT_ENGINE_H
#define LiGHT_ENGINE_H

#include "WS2812b_Driver.h"

typedef enum{
	LIGHT_IN_PROGRESS,
	LIGHT_COMPLETED
}Light_StatusTypeDef;

typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} RGBColorTypeDef;

typedef struct {
    RGBColorTypeDef leds[WS2812B_NUMBER_OF_LEDS];
    uint16_t duration_ms;
} LightCommandTypeDef;

Light_StatusTypeDef LightEngine_PlaySequence(const LightCommandTypeDef *sequence_array, uint16_t sequence_length);


#endif // LiGHT_ENGINE_H