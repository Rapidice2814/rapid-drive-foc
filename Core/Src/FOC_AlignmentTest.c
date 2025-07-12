
#include "FOC_Loops.h"
#include <math.h>

/**
  * @brief Spins the motor and checks the encoder values. Used to check the encoder and the motor direction, and the accuracy of the encoder. 
  * @note 
  * @param None
  * @retval uint8_t: 0 if the loop is not complete, 1 if the loop is complete
  */
FOC_LoopStatusTypeDef Alignment_Test_Loop(FOC_HandleTypeDef *hfoc, float magnitude){

    static uint8_t step = 0;
    static uint32_t next_step_time = 0;
    static uint32_t substep_counter = 0;

    static float reference_angle = 0.0f;
    static float reference_electrical_angle = 0.0f;
    static uint8_t direction = 0;
    static uint8_t dir_swapped = 0;

    static float abs_diff_cw = 0.0f;
    static float abs_diff_ccw = 0.0f;
    static uint8_t attempt = 0;

    switch(step){
        case 0:
            if(HAL_GetTick() >= next_step_time){
                abs_diff_cw = 0.0f;
                abs_diff_ccw = 0.0f;

                FOC_SetPhaseVoltages(hfoc, FOC_InvClarke_transform((ABVoltagesTypeDef){0.3f, 0.0f}));
                step++;
                next_step_time = HAL_GetTick() + 10; 
            }
            break;
        case 1:
            if(HAL_GetTick() >= next_step_time){

                float diff;
                if(hfoc->flash_data.motor_direction_swapped_flag == 1){
                    diff = 2*M_PI - reference_electrical_angle - hfoc->encoder_angle_electrical;
                } else{
                    diff = reference_electrical_angle - hfoc->encoder_angle_electrical;
                }
                normalize_angle2(&diff);

                if(!direction){//cw
                    reference_angle += 2*M_PIF / 200.0f;
                    abs_diff_cw += fabsf(diff) / 200.0f;
                }else{//ccw
                    reference_angle -= 2*M_PIF / 200.0f;
                    abs_diff_ccw += fabsf(diff) / 200.0f;
                }
                normalize_angle(&reference_angle);
                reference_electrical_angle = reference_angle * hfoc->flash_data.motor_pole_pairs;
                normalize_angle(&reference_electrical_angle);

                ABVoltagesTypeDef Vab;
                Vab.alpha = magnitude * cosf(reference_electrical_angle);
                Vab.beta = magnitude * sinf(reference_electrical_angle);
                PhaseVoltagesTypeDef phase_voltages = FOC_InvClarke_transform(Vab);
                FOC_SetPhaseVoltages(hfoc, phase_voltages);

                substep_counter++;
                if(substep_counter > 200){
                    substep_counter = 0;
                    // HAL_GPIO_WritePin(DEBUG_LED0_GPIO_Port, DEBUG_LED0_Pin, 1);

                    if(!direction){
                        if(hfoc->encoder_speed_electrical < 0.0f){
                            dir_swapped = 1;
                        }else{
                            dir_swapped = 0;
                        }
                        direction = 1;
                    }else{
                        if(hfoc->encoder_speed_electrical > 0.0f){
                            if(dir_swapped){
                               hfoc->flash_data.motor_direction_swapped_flag = 1;
                            }else{
                                hfoc->flash_data.motor_direction_swapped_flag = 0;
                            }
                        }
                        direction = 0;
                        step++;
                    }
                    next_step_time = HAL_GetTick() + 100; //next step
                }else{
                    next_step_time = HAL_GetTick() + 10; //next substep
                }
            }
            break;
        case 2:
            if(HAL_GetTick() >= next_step_time){

                FOC_SetPhaseVoltages(hfoc, FOC_InvClarke_transform((ABVoltagesTypeDef){0.0f, 0.0f}));

                Log_printf("Alignment test:\nAbs Diff CW:%d, CCW:%d\nDirection:%d\n", (int)(abs_diff_cw * 1000), (int)(abs_diff_ccw * 1000), hfoc->flash_data.motor_direction_swapped_flag);
                if(abs_diff_cw < 0.3f && abs_diff_ccw < 0.3f){
                    Log_printf("Alignment test passed!\n");
                    step++;
                    next_step_time = HAL_GetTick() + 100;
                } else{
                    Log_printf("Alignment test failed!\n");
                    if(attempt < 3){
                        attempt++;
                        step = 0;
                        next_step_time = HAL_GetTick() + 1;
                    } else{
                        attempt = 0;
                        Log_printf("Alignment test failed after 3 attempts!\n");
                        return FOC_LOOP_ERROR;
                    }
                }
            }
            break;

        default:
            if(HAL_GetTick() >= next_step_time){
                step = 0;
                return FOC_LOOP_COMPLETED; // complete
            }
            break;
    }

return FOC_LOOP_IN_PROGRESS;
}