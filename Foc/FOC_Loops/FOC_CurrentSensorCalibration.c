#include "FOC_Loops.h"
#include "FOC_Handle.h"
#include "FOC_ADC.h"
#include "FOC_USB.h"

#define CURRENT_SENSOR_CALIBRATION_STEPS 100

/**
  * @brief Calibratess the current sensor offset
  * @note 
  * @param hfoc Handle to the FOC structure
  * @retval FOC_StatusTypeDef
  */
FOC_LoopStatusTypeDef FOC_CurrentSensorCalibration(FOC_HandleTypeDef *hfoc){

    static uint8_t step = 0;
    static uint32_t next_step_time = 0;
    static uint16_t measurement_step_counter = 0;

    static float acc_current_offset_a = 0;
    static float acc_current_offset_b = 0;
    static float acc_current_offset_c = 0;

    switch(step){
        case 0:
            if(HAL_GetTick() >= next_step_time){
                USB_printf("Calibrating current sensor\n");

                step++;
                next_step_time = HAL_GetTick() + 1;
            }
            break;
        case 1:
            if(HAL_GetTick() >= next_step_time){

                DRV8323_CSACALStart(&hfoc->hdrv8323);

                acc_current_offset_a = 0;
                acc_current_offset_b = 0;
                acc_current_offset_c = 0;

                step++;
                next_step_time = HAL_GetTick() + 1; //wait 1ms before the next step
            }
            break;
        case 2:
            if(HAL_GetTick() >= next_step_time){
                measurement_step_counter++;

                acc_current_offset_a += hfoc->adc_values.phase_current.a;
                acc_current_offset_b += hfoc->adc_values.phase_current.b;
                acc_current_offset_c += hfoc->adc_values.phase_current.c;

                if(measurement_step_counter >= CURRENT_SENSOR_CALIBRATION_STEPS){

                    DRV8323_CSACALStop(&hfoc->hdrv8323);
                    measurement_step_counter = 0; //reset the counter

                    PhaseCurrentsTypeDef current_offset_values = {
                        .a = acc_current_offset_a / (float)CURRENT_SENSOR_CALIBRATION_STEPS,
                        .b = acc_current_offset_b / (float)CURRENT_SENSOR_CALIBRATION_STEPS,
                        .c = acc_current_offset_c / (float)CURRENT_SENSOR_CALIBRATION_STEPS
                    };
                    
                    FOC_ADC_SetPhaseCurrentOffsets(&current_offset_values);

                    step++;
                    next_step_time = HAL_GetTick() + 1;
                }
            }
            break;
        case 3:
            if(HAL_GetTick() >= next_step_time){
                USB_printf("Current sensor calibration complete!\n");
                
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