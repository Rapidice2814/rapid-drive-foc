#include "FOC_Loops.h"

/**
  * @brief Sets the Current controller PI gains based on the motor parameters
  * @param Handle to the FOC structure
  * @note 
  * @retval FOC_StatusTypeDef
  */
FOC_StatusTypeDef FOC_TuneCurrentPID(FOC_HandleTypeDef *hfoc){
    if(hfoc->flash_data.controller.current_control_bandwidth <= 0.0f || hfoc->flash_data.controller.current_control_bandwidth > 5000) return FOC_ERROR; // check if the bandwidth is in range
    if(hfoc->flash_data.motor.phase_resistance_valid != 1 || hfoc->flash_data.motor.phase_inductance_valid != 1) return FOC_ERROR; // check if the motor parameters are valid

    hfoc->flash_data.controller.PID_gains_d.Kp = hfoc->flash_data.motor.phase_inductance * hfoc->flash_data.controller.current_control_bandwidth;
    hfoc->flash_data.controller.PID_gains_q.Kp = hfoc->flash_data.motor.phase_inductance * hfoc->flash_data.controller.current_control_bandwidth;

    hfoc->flash_data.controller.PID_gains_d.Ki = hfoc->flash_data.motor.phase_resistance * hfoc->flash_data.controller.current_control_bandwidth;
    hfoc->flash_data.controller.PID_gains_q.Ki = hfoc->flash_data.motor.phase_resistance * hfoc->flash_data.controller.current_control_bandwidth;

    return FOC_OK;
}


/**
  * @brief 
  * @note 
  * @param hfoc Handle to the FOC structure
  * @retval FOC_LoopStatusTypeDef
  */
FOC_LoopStatusTypeDef FOC_PIDAutotune(FOC_HandleTypeDef *hfoc){

    static uint8_t step = 0;
    static uint32_t next_step_time = 0;

    switch(step){
        case 0:
            if(HAL_GetTick() >= next_step_time){
                Log_printf("Starting PID Autotune\n");
                Log_printf("Current Control Bandwidth: %d rad/s\n", (int)(hfoc->flash_data.controller.current_control_bandwidth));

                step++;
                next_step_time = HAL_GetTick() + 10;
            }
            break;
        case 1:
            if(HAL_GetTick() >= next_step_time){
                if(FOC_TuneCurrentPID(hfoc) != FOC_OK){
                    step = 0;
                    Log_printf("Error tuning PID gains\n");
                    return FOC_LOOP_ERROR;
                } else{
                    step = 0;
                    Log_printf("PID gains tuned successfully\n");
                    Log_printf("Kp_d: %de-3, Ki_d: %d\n", (int)(hfoc->flash_data.controller.PID_gains_d.Kp * 1000), (int)(hfoc->flash_data.controller.PID_gains_d.Ki));
                    Log_printf("Kp_q: %de-3, Ki_q: %d\n", (int)(hfoc->flash_data.controller.PID_gains_q.Kp * 1000), (int)(hfoc->flash_data.controller.PID_gains_q.Ki));
                    return FOC_LOOP_COMPLETED;
                }
            }
            break;
    }

return FOC_LOOP_IN_PROGRESS;
}