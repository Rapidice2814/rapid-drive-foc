#include "FOC_Loops.h"

/**
  * @brief Calibratess the current sensor offset
  * @note 
  * @param hfoc Handle to the FOC structure
  * @retval FOC_StatusTypeDef
  */
uint8_t FOC_CurrentSensorCalibration(FOC_HandleTypeDef *hfoc){

    static uint8_t step = 0;
    static uint32_t next_step_time = 0;
    static uint8_t measurement_step_counter = 0;

    switch(step){
        case 0:
            if(HAL_GetTick() >= next_step_time){
                Log_printf("Calibrating current sensor\n");

                step++;
                next_step_time = HAL_GetTick() + 1;
            }
            break;
        case 1:
            if(HAL_GetTick() >= next_step_time){

                DRV8323_CSACALStart(&hfoc->hdrv8323);

                step++;
                next_step_time = HAL_GetTick() + 1; //waoit 1ms before the next step
            }
            break;
        case 2:
            if(HAL_GetTick() >= next_step_time){
                measurement_step_counter++;

                hfoc->phase_current_offset.a += hfoc->phase_current.a;
                hfoc->phase_current_offset.b += hfoc->phase_current.b;
                hfoc->phase_current_offset.c += hfoc->phase_current.c;

                if(measurement_step_counter >= 100){

                    DRV8323_CSACALStop(&hfoc->hdrv8323);
                    measurement_step_counter = 0; //reset the counter

                    step++;
                    next_step_time = HAL_GetTick() + 1;
                }
            }
            break;
        case 3:
            if(HAL_GetTick() >= next_step_time){
                Log_printf("Current sensor calibration done\n");
                Log_printf("Phase current offsets: a: %d, b: %d, c: %d\n",
                         (int)(hfoc->phase_current_offset.a * 1000),
                         (int)(hfoc->phase_current_offset.b * 1000),
                         (int)(hfoc->phase_current_offset.c * 1000));

                step++;
                next_step_time = HAL_GetTick() + 10;
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