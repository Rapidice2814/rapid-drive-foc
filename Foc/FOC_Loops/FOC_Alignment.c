#include "FOC_Loops.h"
#include <math.h>
#include "Cordic.h"

#define ALIGNMENT_STEPS 10
#define STARTING_PHASE 0.2f //starting phase for the alignment in radians
#define PHASE_STEP (STARTING_PHASE / ALIGNMENT_STEPS)
/**
* @brief This is alligns the encoder zero to the motor zero (electrical angle).
* @note 
* @param None
* @retval uint8_t: 0 if the loop is not complete, 1 if the loop is complete, 2 if there is an error
*/
FOC_LoopStatusTypeDef FOC_Alignment(FOC_HandleTypeDef *hfoc, float magnitude){

    static uint8_t step = 0;
    static uint32_t next_step_time = 0;
    static float phase = 0.0f;

    switch(step){
        case 0:
            if(HAL_GetTick() >= next_step_time){
                USB_printf("Starting alignment with %dmV\n", (int)(magnitude * 1000));            

                phase = STARTING_PHASE;
                step++;
                next_step_time = HAL_GetTick() + 100; //wait before the next step
            }
            break;
        case 1:
            if(HAL_GetTick() >= next_step_time){
                static uint8_t substep = 0;

                if(substep < ALIGNMENT_STEPS){
                    ABVoltagesTypeDef ab_voltage; 

                    float cos_phase, sin_phase;
                    Cordic_CalculateSinCos(phase, &cos_phase, &sin_phase);

                    ab_voltage.alpha = cos_phase * magnitude;
                    ab_voltage.beta = sin_phase * magnitude;

                    FOC_SetPhaseVoltages(hfoc, InvClarke_transform(ab_voltage));
                    phase = -phase - (phase > 0 ? -PHASE_STEP : PHASE_STEP);
                    substep++;
                    next_step_time = HAL_GetTick() + 100;
                }else if(substep == ALIGNMENT_STEPS){
                    FOC_SetPhaseVoltages(hfoc, InvClarke_transform((ABVoltagesTypeDef){magnitude, 0.0f}));
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
                FOC_StatusTypeDef retval =  FOC_SetEncoderZero(hfoc);

                if(retval != FOC_OK){
                    FOC_SetPhaseVoltages(hfoc, InvClarke_transform((ABVoltagesTypeDef){0.0f, 0.0f}));
                    USB_printf("Error setting encoder zero\n");
                    step = 0;
                    return FOC_LOOP_ERROR;
                }else{
                    USB_printf("Encoder zero set successfully\n");
                    USB_printf("Encoder angle mechanical offset: %dmRad\n", (int)(hfoc->flash_data.encoder.mechanical_offset * 1000));
                    step++;
                    next_step_time = HAL_GetTick() + 100; //wait before the next step
                }
                // USB_printf("Encoder zero set in %d us\n", (int)execution_time);

            }
            break;
        case 3:
            if(HAL_GetTick() >= next_step_time){
                FOC_SetPhaseVoltages(hfoc, InvClarke_transform((ABVoltagesTypeDef){0.0f, 0.0f}));
                
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