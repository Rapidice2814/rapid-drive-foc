#include "FOC_Loops.h"
#include "FOC.h"
#include <math.h>
#include <stdio.h>
#include <string.h>



/**
  * @brief 
  * @note 
  * @param t the time in seconds
  * @param R the resistance in ohms
  * @param L the inductance in henries
  * @param V the voltage in volts
  * @retval the current at time t in amperes
  * This function models the current in an RL circuit with a step input voltage
  */
float model_current(float t, float R, float L, float V) {
    return (V / R) * (1.0 - exp(-R * t / L));
}

float compute_error(float *predArr, float *measArr, int n) {
    float error = 0.0;
    for (int i = 0; i < n; ++i) {
        float diff = predArr[i] - measArr[i];
        error += diff * diff;
    }
    return error;
}

float percentDifferenceMaxMin(float* data, int size) {
    if (size <= 1) return 0.0f;

    float min = data[0];
    float max = data[0];

    for (int i = 1; i < size; i++) {
        if (data[i] < min) min = data[i];
        if (data[i] > max) max = data[i];
    }

    if (max == 0.0f) return 0.0f; // avoid divide by zero

    float diff = max - min;
    float percentDiff = (diff / max) * 100.0f;

    return percentDiff;
}



#define MEASUREMENT_VOLTAGE 1.0f
#define MEASUREMENT_STEPS 25

/**
  * @brief Identifies the motor resistance and inductance by applying a voltage and measuring the current response.
  * @note 
  * @param hfoc Handle to the FOC structure
  * @retval FOC_LoopStatusTypeDef
  */
FOC_LoopStatusTypeDef FOC_MotorIdentification(FOC_HandleTypeDef *hfoc){

    static uint8_t step = 0;
    static uint32_t next_step_time = 0;
    
    static uint8_t selector = 0;

    static float timeArray[MEASUREMENT_STEPS] = {0.0f};
    static float measuredCurrentArray[MEASUREMENT_STEPS] = {0.0f};
    static float predictedCurrentArray[MEASUREMENT_STEPS] = {0.0f};

    static float estimatedR = 0.0f;
    static float estimatedL = 0.0f;

    //a measurement has 2 phases in paraller and 1 in series, so this is 3/2 of the phase resistace
    static float RArray[3] = {0.0f}; //contains the estimated resistance for each measurement
    static float LArray[3] = {0.0f}; //contains the estimated inductance for each measurement
    // static float EArray[3] = {0.0f}; //contains the error for each measurement
    

    static float R = 0.0f;
    static float L = 0.0f;
    static float error = 0.0f;
    static float gradient_R = 0, gradient_L = 0;

    static uint8_t attempt = 0;


    switch(step){
        case 0:
            Log_printf("Starting motor identification attempt %d\n", attempt);

            FOC_SetPhaseVoltages(hfoc, (PhaseVoltagesTypeDef){0.0f, 0.0f, 0.0f});
            
            step++;
            next_step_time = HAL_GetTick() + 100;
            break;
        case 1:
            if(HAL_GetTick() >= next_step_time){

                estimatedR = MOTOR_STATOR_RESISTANCE * (3.0f/2);
                estimatedL = MOTOR_STATOR_INDUCTANCE * (3.0f/2);

                for(int i = 0; i < MEASUREMENT_STEPS; i++){
                    timeArray[i] = (float)i / (float)CURRENT_LOOP_FREQUENCY;
                    measuredCurrentArray[i] = 0.0f;
                }

                if(selector == 0){
                    FOC_SetPhaseVoltages(hfoc, (PhaseVoltagesTypeDef){MEASUREMENT_VOLTAGE, 0.0f, 0.0f});
                } else if(selector == 1){
                    FOC_SetPhaseVoltages(hfoc, (PhaseVoltagesTypeDef){0.0f, MEASUREMENT_VOLTAGE, 0.0f});
                } else if(selector == 2){
                    FOC_SetPhaseVoltages(hfoc, (PhaseVoltagesTypeDef){0.0f, 0.0f, MEASUREMENT_VOLTAGE});
                }

                step++;
                next_step_time = HAL_GetTick() + 100;
            }
            break;
        case 2:
            if(HAL_GetTick() >= next_step_time){

                FOC_SetPhaseVoltages(hfoc, (PhaseVoltagesTypeDef){0.0f, 0.0f, 0.0f});

                step++;
                next_step_time = HAL_GetTick() + 100;
            }
            break;
        case 3:
            if(HAL_GetTick() >= next_step_time){

                step++;
                next_step_time = HAL_GetTick() + 0;
            }
            break;
        case 4:
            if(HAL_GetTick() >= next_step_time){
                static uint8_t substep = 0;
                if(substep == 0){
                    if(selector == 0){
                        FOC_SetPhaseVoltages(hfoc, (PhaseVoltagesTypeDef){MEASUREMENT_VOLTAGE, 0.0f, 0.0f});
                    } else if(selector == 1){
                        FOC_SetPhaseVoltages(hfoc, (PhaseVoltagesTypeDef){0.0f, MEASUREMENT_VOLTAGE, 0.0f});
                    } else if(selector == 2){
                        FOC_SetPhaseVoltages(hfoc, (PhaseVoltagesTypeDef){0.0f, 0.0f, MEASUREMENT_VOLTAGE});
                    }
                    substep++;
                } else if(substep <= MEASUREMENT_STEPS){
                    if(selector == 0){
                        measuredCurrentArray[substep-1] = hfoc->phase_current.a;
                    } else if(selector == 1){
                        measuredCurrentArray[substep-1] = hfoc->phase_current.b;
                    } else if(selector == 2){
                        measuredCurrentArray[substep-1] = hfoc->phase_current.c;
                    }                 
                    substep++;
                } else{
                    substep = 0;
                    step++;
                    FOC_SetPhaseVoltages(hfoc, (PhaseVoltagesTypeDef){0.0f, 0.0f, 0.0f});
                    next_step_time = HAL_GetTick() + 1;
                }
            }
            break;
        case 5:
            {
                static uint8_t substep = 0;
                static float error_default;
                static float error_R, error_L;

                switch(substep){
                    case 0:
                        L = estimatedL;
                        R = estimatedR;
                        substep++;
                        step++;
                        break;
                    case 1:
                        error_default = error;
                        L = estimatedL + 1e-6f;
                        R = estimatedR;
                        substep++;
                        step++;
                        break;
                    case 2:
                        error_L = error;
                        L = estimatedL;
                        R = estimatedR + 1e-2f;
                        substep++;
                        step++;
                        break;
                    case 3:
                        error_R = error;
                        gradient_L = (error_L - error_default) * 1e-5f;
                        gradient_R = (error_R - error_default) * 1e-3f;

                        estimatedL -= gradient_L;
                        estimatedR -= gradient_R;

                        substep = 0;

                        if(error < 0.5f || (fabsf(gradient_L) < 1e-7f && fabsf(gradient_R) < 1e-4f)){ //wat until the error is below 0.5 or until the system converges
                            RArray[selector] = estimatedR;
                            LArray[selector] = estimatedL;
                            // EArray[selector] = error;

                            Log_printf("Motor Identification step: %d, Resistance: %dmOhm, Inductance: %duH, Error: %d\n",
                                 selector, (int)(estimatedR * 1000), (int)(estimatedL * 1000000), (int)(error * 1000));

                            step += 2; //go to the next step
                            next_step_time = HAL_GetTick() + 10;
                        }else{
                            // Log_printf("EError: %d\n", (int)(error * 1000));
                        }
                        break;
                    default:
                        break;
                }
            }
            break;
        case 6:
            {
                static uint8_t substep = 0;
                if(substep < MEASUREMENT_STEPS){
                    predictedCurrentArray[substep] = model_current(timeArray[substep], R, L, MEASUREMENT_VOLTAGE);
                    substep++;
                } else if(substep == MEASUREMENT_STEPS){
                    error = compute_error(predictedCurrentArray, measuredCurrentArray, MEASUREMENT_STEPS);
                    // Log_printf("Error: %d\n", (int)(error * 1000));
                    substep++;
                } else{
                    substep = 0;
                    step--;
                }
            }
            break;
        case 7:
            if(HAL_GetTick() >= next_step_time){
                if(selector == 2){
                    selector = 0;
                    step++;
                } else{
                    selector++;
                    step = 1;
                }
            }
            break;
        case 8:
            if(HAL_GetTick() >= next_step_time){
                float spreadR = percentDifferenceMaxMin(RArray, 3);
                float spreadL = percentDifferenceMaxMin(LArray, 3);
                Log_printf("Percent deviation of Resistance: %d%%, Inductance: %d%%\n", (int)(spreadR), (int)(spreadL));

                if(spreadR < 30.0f && spreadL < 30.0f){
                    // Log_printf("Deviation is acceptable\n");
                    float sumR = 0.0f;
                    float sumL = 0.0f;
                    for(int i = 0; i < 3; i++){
                        sumR += RArray[i];
                        sumL += LArray[i];
                    }

                    hfoc->flash_data.motor.phase_resistance = sumR * (2.0f / (3.0f * 3.0f));
                    hfoc->flash_data.motor.phase_inductance = sumL * (2.0f / (3.0f * 3.0f));
                    hfoc->flash_data.motor.phase_resistance_valid = 1;
                    hfoc->flash_data.motor.phase_inductance_valid = 1;

                    Log_printf("Motor Identified! Resistance: %dmOhm, Inductance: %duH\n", 
                        (int)(hfoc->flash_data.motor.phase_resistance * 1000), 
                        (int)(hfoc->flash_data.motor.phase_inductance * 1000000));

                    step++;
                    next_step_time = HAL_GetTick() + 10;
                } else{
                    Log_printf("Deviation is too high, retrying...\n");
                    step = 0;
                    attempt++;
                    if(attempt > 3){
                        Log_printf("Motor identification failed after 3 attempts\n");

                        hfoc->flash_data.motor.phase_resistance_valid = 0;
                        hfoc->flash_data.motor.phase_inductance_valid = 0;

                        attempt = 0;
                        return FOC_LOOP_ERROR; // error
                    }
                    next_step_time = HAL_GetTick() + 10;
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