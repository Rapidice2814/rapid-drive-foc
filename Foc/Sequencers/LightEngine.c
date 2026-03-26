#include "LightEngine.h"

/** 
 * @brief Plays a light sequence
 * @param sequence_array Pointer to the array of light commands
 * @param sequence_length Length of the sequence array
 * @retval Light_StatusTypeDef: Status of the light sequence playback
 */
Light_StatusTypeDef LightEngine_PlaySequence(const LightCommandTypeDef *sequence_array, uint16_t sequence_length){
    static uint16_t step = 0;
    static uint32_t next_step_time = 0;
    
    if(step < sequence_length){
        if(HAL_GetTick() >= next_step_time){
            for(int i = 0; i < WS2812B_NUMBER_OF_LEDS; i++){
                uint8_t r = sequence_array[step].leds[i].r;
                uint8_t g = sequence_array[step].leds[i].g;
                uint8_t b = sequence_array[step].leds[i].b;

                WS2812b_SetColor(i, r, g, b);
            }

            WS2812b_Send();
            uint16_t duration = sequence_array[step].duration_ms;
            next_step_time = HAL_GetTick() + duration;
            step++;
        }
    } else{
        step = 0;
        return LIGHT_COMPLETED;
    }

    return LIGHT_IN_PROGRESS;
}