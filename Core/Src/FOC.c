#include <math.h>
#include <stdio.h>
#include <string.h>

#include "FOC.h"
#include "FOC_Driver.h"
#include "DRV8323_Driver.h"
#include "FOC_Utils.h"
#include "PID.h"
#include "FOC_Flash.h"
#include "WS2812b_Driver.h"
#include "Debug.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

extern FDCAN_HandleTypeDef hfdcan1;

extern  I2C_HandleTypeDef hi2c1;

extern  SPI_HandleTypeDef hspi2;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern  TIM_HandleTypeDef htim4;

extern UART_HandleTypeDef huart3;

extern PCD_HandleTypeDef hpcd_USB_FS;


FOC_HandleTypeDef hfoc = {0};
FOC_State Current_FOC_State = FOC_INIT;



static uint8_t Current_Sensor_Calibration_Loop();
static uint8_t Alignment_Loop();
static uint8_t General_Test_Loop();
static uint8_t Alignment_Test_Loop();



void Current_Loop();

/* flags, used for the interrupts*/
volatile uint8_t adc1_complete_flag = 0;
volatile uint8_t adc1_half_complete_flag = 0;

volatile uint8_t foc_adc1_measurement_flag = 0;
volatile uint8_t foc_outer_loop_flag = 0;
volatile uint8_t debug_loop_flag = 0;


/* ADC */
#define ADC_LOOP_ALPHA (2.0f/(CURRENT_LOOP_CLOCK_DIVIDER+1))
#define CURRENT_SENSE_CONVERSION_FACTOR 0.00100708007f // 4096.0f * 3.3f / (0.02f * 40.0f)
#define VOLTAGE_SENSE_CONVERSION_FACTOR 11.0f/1.0f // 100:10 voltage divider

static volatile uint32_t adc1_miss_counter = 0;
static volatile uint16_t adc1_buffer[ADC1_CHANNELS * CURRENT_LOOP_CLOCK_DIVIDER * 2] = {0};



void FOC_Setup(){
    HAL_Delay(100);
    // HAL_GetUIDw0();
    // HAL_GetUIDw1();
    // HAL_GetUIDw2();

    // FOC_FLASH_ReadData(&flash_data);
    // if(flash_data.contains_data != 1){
    //     flash_data.contains_data = 1;
    //     FOC_FLASH_WriteData(&flash_data);
    // }

    WS2812b_Setup(&htim4, TIM_CHANNEL_1);

    hfoc.motor_pole_pairs = MOTOR_POLE_PAIRS;
    hfoc.motor_stator_resistance = MOTOR_STATOR_RESISTANCE;
    hfoc.motor_stator_inductance = MOTOR_STATOR_INDUCTANCE;
    hfoc.motor_magnet_flux_linkage = MOTOR_MAGNET_FLUX_LINKAGE;


    if(DRV8323_SetPins(&hfoc.hdrv8323, &hspi2, DRV_NCS_GPIO_Port, DRV_NCS_Pin, DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, DRV_NFAULT_GPIO_Port, DRV_NFAULT_Pin) != DRV8323_OK){
        while(1){};
    }

    if(DRV8323_Init(&hfoc.hdrv8323) != DRV8323_OK){
        while(1){
            HAL_GPIO_TogglePin(DEBUG_LED0_GPIO_Port, DEBUG_LED0_Pin);
            HAL_Delay(100);
        };
    }

    if(AS5047P_SetPins(&hfoc.has5047p, &hspi2, AS_NCS_GPIO_Port, AS_NCS_Pin) != AS5047P_OK){
        while(1){};
    }

    if(AS5047P_Init(&hfoc.has5047p) != AS5047P_OK){
        while(1){
            HAL_GPIO_TogglePin(DEBUG_LED1_GPIO_Port, DEBUG_LED1_Pin);
            HAL_Delay(100);
        };
    }

    FOC_SetEncoderPointer(&hfoc, &htim3.Instance->CNT);
    HAL_TIM_Base_Start(&htim3); //start encoder timer


    FOC_SetPWMCCRPointers(&hfoc, &htim1.Instance->CCR3, &htim1.Instance->CCR2, &htim1.Instance->CCR1, PWM_CLOCK_DIVIDER); //set the pwm ccr register pointers
    FOC_SetInputVoltage(&hfoc, INPUT_VOLTAGE); //set the input voltage, as default
    FOC_SetVoltageLimit(&hfoc, VOLTAGE_LIMIT); //set the voltage limit, as default

    FOC_SetPhaseVoltages(&hfoc, FOC_InvClarke_transform((AlphaBetaVoltages){0.0f, 0.0f}));

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4); //this triggers the adc

    HAL_GPIO_WritePin(INL_ALL_GPIO_Port, INL_ALL_Pin, GPIO_PIN_SET);

    HAL_TIM_Base_Start(&htim1); //pwm timer and also adc trigger
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_buffer, ADC1_CHANNELS * CURRENT_LOOP_CLOCK_DIVIDER * 2); //start adc in dma mode

    
    HAL_TIM_Base_Start(&htim2); //timer for the counter


    // /* PID controllers */
    // PID_Init(&hfoc.pid_current_d, (1.0f/CURRENT_LOOP_FREQUENCY), 0.01f, -PID_LIMIT, PID_LIMIT, -PID_INT_LIMIT, PID_INT_LIMIT);
    // PID_Init(&hfoc.pid_current_q, (1.0f/CURRENT_LOOP_FREQUENCY), 0.01f, -PID_LIMIT, PID_LIMIT, -PID_INT_LIMIT, PID_INT_LIMIT);
    // // PID_SetK(&hfoc.pid_current_d, 3.0f, 10.0f, 0.0f);
    // // PID_SetK(&hfoc.pid_current_q, 1.0f, 200.0f, 0.0f);
    // PID_SetK(&hfoc.pid_current_d, 0.0f, 0.0f, 0.0f);
    // PID_SetK(&hfoc.pid_current_q, 0.0f, 0.0f, 0.0f);

    // PID_Init(&hfoc.pid_speed, (1.0f/DEBUG_FREQUENCY), 0.01f, -2.0f, 2.0f, -2.0f, 2.0f);
    // PID_SetK(&hfoc.pid_speed, 0.01f, 0.0f, 0.0f);

    // hfoc.speed_setpoint = 300.0f;

    // /* UART */
    // Debug_Setup();

    // adc1_miss_counter = 0;

    
    // hfoc.dq_current_setpoint = (DQCurrents){0.0f, 0.0f}; //set the current setpoint


}


static uint32_t start_time = 0;
static uint32_t execution_time = 0;
static uint32_t max_execution_time = 0;

static uint32_t adc1_start_time = 0;
static uint32_t adc1_time = 0;
static uint32_t max_adc1_time = 0;

// static uint32_t current_loop_start_time = 0;
// static uint32_t current_loop_time = 0;
// static uint32_t max_current_loop_time = 0;

// static uint32_t debug_loop_start_time = 0;
// static uint32_t debug_loop_time = 0;
// static uint32_t max_debug_loop_time = 0;



char usart3_tx_buffer[200];


void FOC_Loop(){
    start_time = __HAL_TIM_GET_COUNTER(&htim2);

    if(adc1_half_complete_flag || adc1_complete_flag){ //This loop runs at (PWM frequency / CURRENT_LOOP_CLOCK_SCALER)
        adc1_start_time = __HAL_TIM_GET_COUNTER(&htim2);

            int start_index = adc1_half_complete_flag ? 0 : CURRENT_LOOP_CLOCK_DIVIDER * ADC1_CHANNELS;
            int end_index = start_index + CURRENT_LOOP_CLOCK_DIVIDER * ADC1_CHANNELS;

            for (int i = start_index; i < end_index; i += ADC1_CHANNELS) {
                hfoc.phase_current.a = hfoc.phase_current.a * (1 - ADC_LOOP_ALPHA) + ADC_LOOP_ALPHA * CURRENT_SENSE_CONVERSION_FACTOR * (float)(adc1_buffer[i + 0] - 2048) - hfoc.phase_current_offset.a;
                hfoc.phase_current.b = hfoc.phase_current.b * (1 - ADC_LOOP_ALPHA) + ADC_LOOP_ALPHA * CURRENT_SENSE_CONVERSION_FACTOR * (float)(adc1_buffer[i + 1] - 2048) - hfoc.phase_current_offset.b;
                hfoc.phase_current.c = hfoc.phase_current.c * (1 - ADC_LOOP_ALPHA) + ADC_LOOP_ALPHA * CURRENT_SENSE_CONVERSION_FACTOR * (float)(adc1_buffer[i + 2] - 2048) - hfoc.phase_current_offset.c;
                hfoc.vbus            = hfoc.vbus            * (1 - ADC_LOOP_ALPHA) + ADC_LOOP_ALPHA * VOLTAGE_SENSE_CONVERSION_FACTOR * (float)(adc1_buffer[i + 3] * 3.3f / 4096.0f) - hfoc.vbus_offset;
            }

            HAL_GPIO_TogglePin(PB2_GPIO_Port, PB2_Pin);

            foc_adc1_measurement_flag = 1; //triggered after the adc conversion is complete (either first or second half of the buffer)
            adc1_half_complete_flag = 0;
            adc1_complete_flag = 0;

        adc1_time = __HAL_TIM_GET_COUNTER(&htim2) - adc1_start_time;
        if (adc1_time > max_adc1_time) {
            max_adc1_time = adc1_time;
        }
    }

    switch(Current_FOC_State){
        
        case FOC_INIT:
            __NOP();
            Current_FOC_State = FOC_CURRENT_SENSOR_CALIBRATION;
            break;
        case FOC_CURRENT_SENSOR_CALIBRATION:
            if(Current_Sensor_Calibration_Loop()){
                Current_FOC_State = FOC_ALIGNMENT;
                hfoc.adc_calibrated = 1;
            }
            break;
        case FOC_GENERAL_TEST:
            if(General_Test_Loop()){
                // Current_FOC_State = FOC_ALIGNMENT;
                __NOP();
            }
            break;
        case FOC_CALIBRATION:
            break;
        case FOC_ALIGNMENT:
            if(Alignment_Loop()){
                Current_FOC_State = FOC_ALIGNMENT_TEST;
            }
            break;
        case FOC_ALIGNMENT_TEST:
            if(Alignment_Test_Loop()){
                Current_FOC_State = FOC_GENERAL_TEST;
            }
            break;
        case FOC_RUN:
            break;
    }

    execution_time = __HAL_TIM_GET_COUNTER(&htim2) - start_time;
    if (execution_time > max_execution_time) {
        max_execution_time = execution_time;
    }
    if(execution_time > 100){
        snprintf(usart3_tx_buffer, sizeof(usart3_tx_buffer), "Execution time: %lu us\r\n", execution_time);
        HAL_UART_Transmit_DMA(&huart3, (uint8_t*)usart3_tx_buffer, strlen(usart3_tx_buffer));
    }
}

// void Current_Loop(){
//     if(DRV8323_CheckFault(&hfoc.hdrv8323)){ //check for motor driver fault
//         hfoc.motor_disable_flag = 1;
//     }

//     FOC_UpdateEncoderAngle(&hfoc);
//     FOC_UpdateEncoderSpeed(&hfoc, CURRENT_LOOP_FREQUENCY);

//     hfoc.ab_current = FOC_Clarke_transform(hfoc.phase_current);
//     hfoc.dq_current = FOC_Park_transform(hfoc.ab_current, hfoc.encoder_angle_electric);

//     hfoc.dq_voltage.d = PID_Update(&hfoc.pid_current_d, hfoc.dq_current_setpoint.d, hfoc.dq_current.d);
//     hfoc.dq_voltage.q = PID_Update(&hfoc.pid_current_q, hfoc.dq_current_setpoint.q, hfoc.dq_current.q);

//     hfoc.ab_voltage = FOC_InvPark_transform(hfoc.dq_voltage, hfoc.encoder_angle_electric);
//     PhaseVoltages phase_voltages = FOC_InvClarke_transform(hfoc.ab_voltage);
//     FOC_SetPhaseVoltages(&hfoc, phase_voltages);
// }

    // if(foc_adc1_measurement_flag && !alignment_test_mode){
    //     // HAL_GPIO_WritePin(User_LED_GPIO_Port, User_LED_Pin, 1);
    //     current_loop_start_time = __HAL_TIM_GET_COUNTER(&htim2);
    //         Current_Loop();
    //     current_loop_time = __HAL_TIM_GET_COUNTER(&htim2) - current_loop_start_time;
    //     if (current_loop_time > max_current_loop_time) {
    //         max_current_loop_time = current_loop_time;
    //     }

    //     foc_adc1_measurement_flag = 0;
    //     // HAL_GPIO_WritePin(User_LED_GPIO_Port, User_LED_Pin, 0);
    // } else {
    //     debug_loop_start_time = __HAL_TIM_GET_COUNTER(&htim2);
    //         Debug_Loop();
    //     debug_loop_time = __HAL_TIM_GET_COUNTER(&htim2) - debug_loop_start_time;
    //     if (debug_loop_time > max_debug_loop_time) {
    //         max_debug_loop_time = debug_loop_time;
    //     }
    // }

// uint32_t alignment_test_mode_counter = 0;
// static uint8_t debug_state = 0;
// static uint8_t setpoint_counter = 0;
// static AlphaBetaVoltages Vab_setpoint[4] = {{2.0f, 0.0f}, {0.0f, 2.0f}, {-2.0f, 0.0f}, {0.0f, -2.0f}};
// static AlphaBetaVoltages Vab_setpoint[4] = {{1.0f, 0.0f}, {0.0f, 1.0f}, {-1.0f, 0.0f}, {0.0f, -1.0f}};
// static AlphaBetaVoltages Vab_setpoint[4] = {{0.5, 0.0f}, {0.0f, 0.5f}, {-0.5f, 0.0f}, {0.0f, -0.5f}};
// static AlphaBetaVoltages Vab_setpoint[4] = {{0.5f, 0.5f}, {-0.5f, 0.5f}, {-0.5f, -0.5f}, {0.5f, -0.5f}};

// static PhaseVoltages phase_voltages_setpoint[4] = {0};
// static AlphaBetaCurrents Iab_result[4] = {0};
// static float Iab_polar_result_mag[4] = {0};
// static float Iab_polar_result_arg[4] = {0};
// static float Angle_result[4] = {0};
// static float Angle_error[4] = {0};
// static DQCurrents Idq_result[4] = {0};
// static AlphaBetaVoltages Vab_result[4] = {0};

    // if(debug_loop_flag){
    //         Debug_Queue(&hfoc);

    //         // hfoc.dq_current_setpoint.q = PID_Update(&hfoc.pid_speed, hfoc.speed_setpoint, hfoc.encoder_speed_electric);

    //         if(alignment_test_mode){
    //             if(alignment_test_mode_counter > 10){
    //                 if(debug_state == 0){
    //                     phase_voltages_setpoint[setpoint_counter] = FOC_InvClarke_transform(Vab_setpoint[setpoint_counter]);
    //                     FOC_SetPhaseVoltages(&hfoc, phase_voltages_setpoint[setpoint_counter]);
    //                     debug_state++;
    //                 } else if(debug_state == 1){
    //                     Iab_result[setpoint_counter] = FOC_Clarke_transform(hfoc.phase_current);
                        // Iab_polar_result_mag[setpoint_counter] = sqrtf(Iab_result[setpoint_counter].alpha * Iab_result[setpoint_counter].alpha + Iab_result[setpoint_counter].beta * Iab_result[setpoint_counter].beta);
                        // Iab_polar_result_arg[setpoint_counter] = atan2f(Iab_result[setpoint_counter].beta, Iab_result[setpoint_counter].alpha);
        
    //                     if(Iab_polar_result_arg[setpoint_counter] < 0){
    //                         Iab_polar_result_arg[setpoint_counter] += 2*M_PIF;
    //                     }
    //                     FOC_UpdateEncoderAngle(&hfoc);
    //                     Idq_result[setpoint_counter] = FOC_Park_transform(Iab_result[setpoint_counter], hfoc.encoder_angle_electric);
    //                     Angle_result[setpoint_counter] = hfoc.encoder_angle_electric;
        
    //                     Angle_error[setpoint_counter] = Iab_polar_result_arg[setpoint_counter] - Angle_result[setpoint_counter];
    //                     if(Angle_error[setpoint_counter] < 0){
    //                         Angle_error[setpoint_counter] += 2*M_PIF;
    //                     }
                        
    //                     DQVoltages Vdq;
    //                     Vdq.d = Idq_result[setpoint_counter].d;
    //                     Vdq.q = Idq_result[setpoint_counter].q;
        
    //                     Vab_result[setpoint_counter] = FOC_InvPark_transform(Vdq, hfoc.encoder_angle_electric);
        
                        
    //                     setpoint_counter++;
    //                     if(setpoint_counter > 3){
    //                         setpoint_counter = 0;
    //                     }
    //                     debug_state++;
    //                 } else if(debug_state == 2){
    //                     FOC_SetPhaseVoltages(&hfoc, FOC_InvClarke_transform((AlphaBetaVoltages){0.0f, 0.0f}));
    //                     debug_state = 0;
    //                 }
        
    //                 alignment_test_mode_counter = 0;
    //             }
                
    //         }
    //         alignment_test_mode_counter++;

    //     debug_loop_flag = 0;
    // }

    // if(hfoc.motor_disable_flag){
    //     FOC_SetPhaseVoltages(&hfoc, (PhaseVoltages){0.0f, 0.0f, 0.0f});
    //     DRV8323_Disable(&hfoc.hdrv8323);
    //     while(1){
    //         // HAL_GPIO_TogglePin(User_LED_GPIO_Port, User_LED_Pin);
    //         HAL_Delay(200);
    //     }
    // }




/**
  * @brief Current sensor calibration loop
  * @note This function is used to calibrate the current sensor. It takes 100 samples of the current sensor and calculates the offset.
  * @param None
  * @retval uint8_t: 0 if the calibration is not complete, 1 if the calibration is complete
  */
static uint8_t Current_Sensor_Calibration_Loop(){

    static uint8_t step = 0;
    static uint32_t next_step_time = 0;
    static uint8_t measurement_step_counter = 0;
    static uint8_t step_complete = 0;

    switch(step){
        case 0:
            if(!step_complete){

                DRV8323_CSACALStart(&hfoc.hdrv8323);

                step_complete = 1;
                next_step_time = HAL_GetTick() + 1; //wait 1ms before the next step
            }
            break;
        case 1:
            if(!step_complete){
                if(foc_adc1_measurement_flag){
                    measurement_step_counter++;
    
                    hfoc.phase_current_offset.a += hfoc.phase_current.a;
                    hfoc.phase_current_offset.b += hfoc.phase_current.b;
                    hfoc.phase_current_offset.c += hfoc.phase_current.c;
    
                    foc_adc1_measurement_flag = 0;
                    if(measurement_step_counter >= 100){

                        DRV8323_CSACALStop(&hfoc.hdrv8323);
                        measurement_step_counter = 0; //reset the counter

                        step_complete = 1;
                        next_step_time = HAL_GetTick() + 1; //wait 1ms before the next step
                    }
                }
            }
            break;
        case 2:
        default:
            step = 0;
            return 1; //calibration complete
            break;
    }

    if(step_complete && (HAL_GetTick() >= next_step_time)){
        step_complete = 0;
        step++;
    }
    return 0;
}

/**
  * @brief 
  * @note 
  * @param None
  * @retval uint8_t: 0 if the calibration is not complete, 1 if the calibration is complete
  */
 static uint8_t Alignment_Loop(){

    static uint8_t step = 0;
    static uint32_t next_step_time = 0;
    static uint8_t step_complete = 0;
    
    switch(step){
        case 0:
            if(!step_complete){
                FOC_SetPhaseVoltages(&hfoc, FOC_InvClarke_transform((AlphaBetaVoltages){0.3f, 0.0f}));
                step_complete = 1;
                next_step_time = HAL_GetTick() + 300; //wait 1ms before the next step
            }
            break;
        case 1:
            if(!step_complete){
                FOC_SetEncoderZero(&hfoc);
                step_complete = 1;
                next_step_time = HAL_GetTick() + 10; //wait 1ms before the next step
            }
            break;
        case 2:
            if(!step_complete){
                FOC_SetPhaseVoltages(&hfoc, FOC_InvClarke_transform((AlphaBetaVoltages){0.0f, 0.0f}));
                step_complete = 1;
                next_step_time = HAL_GetTick() + 1; //wait 1ms before the next step
            }
            break;
        default:
            step = 0;
            return 1; // complete
            break;
        }
    
        if(step_complete && (HAL_GetTick() >= next_step_time)){
            step_complete = 0;
            step++;
        }
    return 0;
    }

/**
  * @brief 
  * @note 
  * @param None
  * @retval uint8_t: 0 if the calibration is not complete, 1 if the calibration is complete
  */
static uint8_t General_Test_Loop(){

    static uint8_t step = 0;
    static uint32_t next_step_time = 0;
    static uint8_t step_complete = 0;

    switch(step){
        case 0:
            if(!step_complete){
                FOC_UpdateEncoderAngle(&hfoc);
                
                HAL_GPIO_WritePin(DEBUG_LED0_GPIO_Port, DEBUG_LED0_Pin, 1);
                for(int i = 0; i < WS2812B_NUMBER_OF_LEDS; i++){
                    WS2812b_SetColor(i, 0, 25, 0); //green
                }
                WS2812b_Send();

                step_complete = 1;
                next_step_time = HAL_GetTick() + 100; //wait 1ms before the next step
            }
            break;
        case 1:
            if(!step_complete){

                HAL_GPIO_WritePin(DEBUG_LED0_GPIO_Port, DEBUG_LED0_Pin, 0);
                for(int i = 0; i < WS2812B_NUMBER_OF_LEDS; i++){
                    WS2812b_SetColor(i, 0, 0, 0); 
                }
                WS2812b_Send();
                
                step_complete = 1;
                next_step_time = HAL_GetTick() + 100; //wait 1ms before the next step
            }
            break;
        case 2:
        default:
            step = 0;
            return 1; // complete
            break;
    }

    if(step_complete && (HAL_GetTick() >= next_step_time)){
        step_complete = 0;
        step++;
    }
return 0;
}


/**
  * @brief 
  * @note 
  * @param None
  * @retval uint8_t: 0 if the calibration is not complete, 1 if the calibration is complete
  */
 static uint8_t Alignment_Test_Loop(){

    static uint8_t step = 0;
    static uint32_t next_step_time = 0;
    static uint8_t step_complete = 0;
    static uint8_t substep_counter = 0;

    static float reference_angle = 0.0f;
    static float reference_electrical_angle = 0.0f;
    static float magnitude = 0.3f;
    static uint8_t direction = 0;
    static uint8_t dir_swapped = 0;

    static float current_angle = 0.0f;
    static float current_magnitude = 0.0f;

    switch(step){
        case 0:
            if(!step_complete && (HAL_GetTick() >= next_step_time)){

                snprintf(usart3_tx_buffer, sizeof(usart3_tx_buffer), "Ref: %d, Ang: %d, CAng: %d, CMag: %d\n", (int)(reference_electrical_angle * 1000), (int)(hfoc.encoder_angle_electrical * 1000), (int)(current_angle * 1000), (int)(current_magnitude * 1000));
                HAL_UART_Transmit_DMA(&huart3, (uint8_t*)usart3_tx_buffer, strlen(usart3_tx_buffer));

                AlphaBetaVoltages Vab;
                Vab.alpha = magnitude * cosf(reference_electrical_angle);
                Vab.beta = magnitude * sinf(reference_electrical_angle);
                PhaseVoltages phase_voltages = FOC_InvClarke_transform(Vab);
                FOC_SetPhaseVoltages(&hfoc, phase_voltages);

                FOC_UpdateEncoderAngle(&hfoc);
                FOC_UpdateEncoderSpeed(&hfoc, 1.0f, 2.0f/(50.0f + 1.0f));

                AlphaBetaCurrents Iab = FOC_Clarke_transform(hfoc.phase_current);
                current_angle = atan2f(Iab.beta, Iab.alpha);
                normalize_angle(&current_angle);
                current_magnitude = sqrtf(Iab.alpha * Iab.alpha + Iab.beta * Iab.beta);


                if(direction == 0){//cw
                    reference_angle += 2*M_PIF / 100.0f;
                }else{//ccw
                    reference_angle -= 2*M_PIF / 100.0f;
                }
                reference_electrical_angle = reference_angle * hfoc.motor_pole_pairs;
                normalize_angle(&reference_electrical_angle);
                
                
                substep_counter++;
                if(substep_counter > 100){
                    substep_counter = 0;
                    HAL_GPIO_WritePin(DEBUG_LED0_GPIO_Port, DEBUG_LED0_Pin, 1);

                    if(!direction){

                        if(hfoc.encoder_speed_electrical < 0.0f){
                            dir_swapped = 1;
                        }

                        direction = 1;
                        substep_counter = 0;
                    }else{

                        if(hfoc.encoder_speed_electrical > 0.0f){
                            if(dir_swapped){
                               hfoc.flash_data.motor_direction_swapped = !hfoc.flash_data.motor_direction_swapped;
                            }else{
                                __NOP(); //error
                            }
                        }

                        step_complete = 1;
                    }
                    next_step_time = HAL_GetTick() + 10;
                }else{
                    next_step_time = HAL_GetTick() + 10; 
                }
            }
            break;
        case 1:
            if(!step_complete){

                FOC_SetPhaseVoltages(&hfoc, (PhaseVoltages){0.0f, 0.0f, 0.0f});

                HAL_GPIO_WritePin(DEBUG_LED0_GPIO_Port, DEBUG_LED0_Pin, 0);
                HAL_GPIO_TogglePin(DEBUG_LED1_GPIO_Port, DEBUG_LED1_Pin);
                // step_complete = 1;
                next_step_time = HAL_GetTick() + 100;
            }
            break;
        case 2:
        default:
            step = 0;
            return 1; // complete
            break;
    }

    if(step_complete && (HAL_GetTick() >= next_step_time)){
        step_complete = 0;
        step++;
    }
return 0;
}


// /**
//   * @brief 
//   * @note 
//   * @param None
//   * @retval uint8_t: 0 if the calibration is not complete, 1 if the calibration is complete
//   */
//  static uint8_t General_Test_Loop(){

//     static uint8_t step = 0;
//     static uint32_t next_step_time = 0;
//     static uint8_t step_complete = 0;

//     switch(step){
//         case 0:
//             if(!step_complete){

//                 HAL_GPIO_WritePin(DEBUG_LED0_GPIO_Port, DEBUG_LED0_Pin, 1);
//                 step_complete = 1;
//                 next_step_time = HAL_GetTick() + 1000; //wait 1ms before the next step
//             }
//             break;
//         case 1:
//             if(!step_complete){

//                 HAL_GPIO_WritePin(DEBUG_LED0_GPIO_Port, DEBUG_LED0_Pin, 0);
//                 step_complete = 1;
//                 next_step_time = HAL_GetTick() + 1000; //wait 1ms before the next step
//             }
//             break;
//         case 2:
//         default:
//             step = 0;
//             return 1; // complete
//             break;
//     }

//     if(step_complete && (HAL_GetTick() >= next_step_time)){
//         step_complete = 0;
//         step++;
//     }
// return 0;
// }




























void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) { //when the adc conversion is complete
    if(hadc->Instance == ADC1){ 
        if(!adc1_complete_flag){
            adc1_complete_flag = 1;  
        }else{
            adc1_miss_counter++;
        } 
    }
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) { //when the adc conversion is half complete
    if(hadc->Instance == ADC1){
        if(!adc1_half_complete_flag){
            adc1_half_complete_flag = 1;
        }else{
            adc1_miss_counter++;
        }
    }
    
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if(htim->Instance == TIM6){
        // foc_adc1_measurement_flag = 1;
    } else if(htim->Instance == TIM7){
        // foc_outer_loop_flag = 1;
    } else if(htim->Instance == TIM17){
        // debug_loop_flag = 1;
    }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
    if(htim == &htim4){
        WS2812b_PulseFinishedCallback();
    }
}