#include "FOC_Loops.h"
#include <math.h>
#include "FOC_Utils.h"
#include "FOC_Flash.h"
#include "FOC_Config.h"



FOC_LoopStatusTypeDef FOC_AntiCoggingMeasurement(FOC_HandleTypeDef *hfoc){

    Current_Loop(hfoc);

    static float current = 0.0f;
    current = current * (1 - 0.01f) + 0.01f * hfoc->dq_current.q;

    static uint8_t current_loop_counter = 0;
    if(++current_loop_counter >= SPEED_LOOP_CLOCK_DIVIDER){
        current_loop_counter = 0;

        static PIDValuesTypeDef old_pid_gains = {0};
        
        static uint8_t step = 0;
        static uint32_t next_step_time = 0;
        static uint8_t direction = 0;
        
        static uint32_t start_time = 0;


        
        switch(step){
            case 0:
                start_time = HAL_GetTick();
                hfoc->flash_data.position_PID_enabled_flag = 1;
                hfoc->flash_data.speed_PID_enabled_flag = 0;

                old_pid_gains = hfoc->flash_data.PID_gains_position; //change the PID gains temporarily
                hfoc->flash_data.PID_gains_position.Kp = 10.0f;
                hfoc->flash_data.PID_gains_position.Kd = 0.2f;
                hfoc->flash_data.PID_gains_position.Ki = 20.0f;


                step++;
                break;
            case 1:
                hfoc->angle_setpoint = 0.0f; //reset the angle setpoint
                step++;
                next_step_time = HAL_GetTick() + 100;
                break;
            case 2:
                if(HAL_GetTick() >= next_step_time){
                    static uint16_t substep = 0;
                    static uint8_t hold_counter = 0;
                    
                    if(substep < NUMBER_OF_ANTICOG_MEASUREMENTS){
                        float error = hfoc->angle_setpoint - hfoc->encoder_angle_mechanical;
                        normalize_angle2(&error);

                        if(fabsf(hfoc->encoder_speed_mechanical) < 0.001f && fabsf(error) < 0.005f){
                            hold_counter++;
                        }else{
                            hold_counter = 0;
                        }
                        if(hold_counter > 10){
                            if(direction == 0){
                                hfoc->angle_setpoint = substep * ANTICOG_ANGLE_STEP;
                                hfoc->flash_data.anticogging_measurements[direction][substep] = current;
                            }else{
                                hfoc->angle_setpoint = (NUMBER_OF_ANTICOG_MEASUREMENTS-substep-1) * ANTICOG_ANGLE_STEP;
                                hfoc->flash_data.anticogging_measurements[direction][NUMBER_OF_ANTICOG_MEASUREMENTS-substep-1] = current;
                            }

                            Log_printf("Measurement %d: Target:%dmRad, Actual:%dmRad, Current:%dmA, Delta:%d\n", 
                                substep, (int)(hfoc->angle_setpoint * 1000), (int)(hfoc->encoder_angle_mechanical * 1000), 
                                (int)(current * 1000), (int)((error) * 1000));
                            substep++;

                            hold_counter = 0;
                        }

                        next_step_time = HAL_GetTick() + 2;
                    } else{
                        substep = 0;
                        if(direction == 0){
                            direction = 1;
                            step = 1;
                        }else{
                            step++;
                            next_step_time = HAL_GetTick() + 100;
                        }
                    }
                }
                break;
            case 3:
                if(HAL_GetTick() >= next_step_time){
                    static uint16_t substep = 0;
                    static uint8_t dir = 0;

                    if(substep == 0){
                        Log_printf("Current measurements for direction %d:\n", dir);
                        substep++;
                        next_step_time = HAL_GetTick() + 10;
                    }else if(substep <= NUMBER_OF_ANTICOG_MEASUREMENTS){
                        Log_printf("%d,", (int)(hfoc->flash_data.anticogging_measurements[dir][substep-1] * 1000));
                        substep++;
                        next_step_time = HAL_GetTick() + 2;
                    }else{
                        Log_printf("\n");
                        substep = 0;
                        if(dir == 0){
                            dir = 1;
                        }else{
                            Log_printf("Measurement completed in %ds\n", (int)(HAL_GetTick() - start_time)/1000);
                            dir = 0;
                            step++;
                            next_step_time = HAL_GetTick() + 1000;
                        }
                    }
                }
                break;
            default:
                if(HAL_GetTick() >= next_step_time){
                    step = 0;
                    hfoc->flash_data.PID_gains_position = old_pid_gains; //restore the PID gains
                    hfoc->flash_data.position_PID_enabled_flag = 0;
                    return FOC_LOOP_COMPLETED;
                }
                break;
        }
        Speed_Loop(hfoc);
    }

    return FOC_LOOP_IN_PROGRESS;
}