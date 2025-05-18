#include "FOC_Loops.h"
#include <math.h>

/**
* @brief This is alligns the encoder zero to the motor zero (electrical angle).
* @note 
* @param None
* @retval uint8_t: 0 if the loop is not complete, 1 if the loop is complete, 2 if there is an error
*/
uint8_t FOC_Alignment(FOC_HandleTypeDef *hfoc, float magnitude){

    static uint8_t step = 0;
    static uint32_t next_step_time = 0;
    magnitude = 0.0f;

    switch(step){
        case 0:
            if(HAL_GetTick() >= next_step_time){
                FOC_SetPhaseVoltages(hfoc, FOC_InvClarke_transform((ABVoltagesTypeDef){magnitude, 0.0f}));
                step++;
                next_step_time = HAL_GetTick() + 500; //wait before the next step
            }
            break;
        case 1:
            if(HAL_GetTick() >= next_step_time){
                if(FOC_SetEncoderZero(hfoc) != FOC_OK){
                    FOC_SetPhaseVoltages(hfoc, FOC_InvClarke_transform((ABVoltagesTypeDef){0.0f, 0.0f}));
                    step = 0;
                    return 2; //error
                }

                step++;
                next_step_time = HAL_GetTick() + 100; //wait before the next step
            }
            break;
        case 2:
            if(HAL_GetTick() >= next_step_time){
                FOC_SetPhaseVoltages(hfoc, FOC_InvClarke_transform((ABVoltagesTypeDef){0.0f, 0.0f}));
                
                step++;
                next_step_time = HAL_GetTick() + 100; //wait before the next step
            }
            break;
        default:
            if(HAL_GetTick() >= next_step_time){
                step = 0;
                return 1; // complete
            }
            break;
        }

    return 0;
}