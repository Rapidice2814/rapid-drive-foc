#include "FOC_Loops.h"
#include <math.h>

// extern TIM_HandleTypeDef htim2;
// static uint32_t start_time = 0;
// static uint32_t execution_time = 0;
// static uint32_t max_execution_time = 0;



#define ALIGNMENT_STEPS 10
#define STARTING_ANGLE 0.2f //starting angle for the alignment in radians
#define ANGLE_STEP (STARTING_ANGLE / ALIGNMENT_STEPS)
/**
* @brief This is alligns the encoder zero to the motor zero (electrical angle).
* @note 
* @param None
* @retval uint8_t: 0 if the loop is not complete, 1 if the loop is complete, 2 if there is an error
*/
FOC_LoopStatusTypeDef FOC_Alignment(FOC_HandleTypeDef *hfoc, float magnitude){

    static uint8_t step = 0;
    static uint32_t next_step_time = 0;
    static float angle = 0.0f;

    switch(step){
        case 0:
            if(HAL_GetTick() >= next_step_time){
                Log_printf("Starting alignment with %dmV\n", (int)(magnitude * 1000));            

                angle = STARTING_ANGLE;
                step++;
                next_step_time = HAL_GetTick() + 100; //wait before the next step
            }
            break;
        case 1:
            if(HAL_GetTick() >= next_step_time){
                static uint8_t substep = 0;

                if(substep < ALIGNMENT_STEPS){
                    ABVoltagesTypeDef ab_voltage; 
                    ab_voltage.alpha = cosf(angle) * magnitude;
                    ab_voltage.beta = sinf(angle) * magnitude;
                    FOC_SetPhaseVoltages(hfoc, FOC_InvClarke_transform(ab_voltage));
                    angle = -angle - (angle > 0 ? -ANGLE_STEP : ANGLE_STEP);
                    substep++;
                    next_step_time = HAL_GetTick() + 100;
                }else if(substep == ALIGNMENT_STEPS){
                    FOC_SetPhaseVoltages(hfoc, FOC_InvClarke_transform((ABVoltagesTypeDef){magnitude, 0.0f}));
                    substep++;
                    next_step_time = HAL_GetTick() + 100;
                }else{
                    substep = 0;
                    step++;
                    next_step_time = HAL_GetTick() + 100; //wait before the next step
                }
            }
            break;
        case 2:
            if(HAL_GetTick() >= next_step_time){
                // start_time = __HAL_TIM_GET_COUNTER(&htim2);
                FOC_StatusTypeDef retval =  FOC_SetEncoderZero(hfoc);
                // execution_time = __HAL_TIM_GET_COUNTER(&htim2) - start_time;

                if(retval != FOC_OK){
                    FOC_SetPhaseVoltages(hfoc, FOC_InvClarke_transform((ABVoltagesTypeDef){0.0f, 0.0f}));
                    Log_printf("Error setting encoder zero\n");
                    step = 0;
                    return FOC_LOOP_ERROR;
                }else{
                    Log_printf("Encoder zero set successfully\n");
                    Log_printf("Encoder angle mechanical offset: %dmRad\n", (int)(hfoc->flash_data.encoder_angle_mechanical_offset * 1000));
                    step++;
                    next_step_time = HAL_GetTick() + 100; //wait before the next step
                }
                // Log_printf("Encoder zero set in %d us\n", (int)execution_time);

            }
            break;
        case 3:
            if(HAL_GetTick() >= next_step_time){
                FOC_SetPhaseVoltages(hfoc, FOC_InvClarke_transform((ABVoltagesTypeDef){0.0f, 0.0f}));
                
                step++;
                next_step_time = HAL_GetTick() + 100; //wait before the next step
            }
            break;
        default:
            if(HAL_GetTick() >= next_step_time){
                step = 0;
                return FOC_LOOP_COMPLETED;
            }
            break;
        }

    return FOC_LOOP_IN_PROGRESS;
}