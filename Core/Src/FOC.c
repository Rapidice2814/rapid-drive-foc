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
#include "FOC_Loops.h"
#include "Logging.h"
#include "FOC_Config.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;

extern FDCAN_HandleTypeDef hfdcan1;

extern  I2C_HandleTypeDef hi2c1;

extern RNG_HandleTypeDef hrng;

extern  SPI_HandleTypeDef hspi2;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern  TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim7;

extern UART_HandleTypeDef huart3;

extern PCD_HandleTypeDef hpcd_USB_FS;


FOC_HandleTypeDef hfoc = {0};
FOC_State Current_FOC_State = FOC_STATE_INIT;



static uint8_t General_LED_Loop();
static uint8_t Error_LED_Loop();

/* flags, used for the interrupts*/
volatile uint8_t adc1_complete_flag = 0;
volatile uint8_t adc1_half_complete_flag = 0;

volatile uint8_t adc2_complete_flag = 0;
volatile uint8_t adc2_half_complete_flag = 0;

volatile uint8_t foc_adc1_measurement_flag = 0;
volatile uint8_t foc_outer_loop_flag = 0;
volatile uint8_t debug_loop_flag = 0;


/* ADC gains*/
#define CURRENT_SENSE_CONVERSION_FACTOR ((3.3f/4096.0f) / (CURRENT_SENSE_RESISTANCE * CSA_GAIN_VALUE))
#define VOLTAGE_SENSE_CONVERSION_FACTOR ((3.3f/4096.0f) / VBUS_VOLTAGE_DIVIDER_RATIO)

static volatile uint16_t adc1_buffer[ADC1_CHANNELS * CURRENT_LOOP_CLOCK_DIVIDER * 2] = {0};
static volatile uint16_t adc2_buffer[ADC2_CHANNELS * CURRENT_LOOP_CLOCK_DIVIDER * 2] = {0};


void FOC_Setup(){
    FOC_Init(&hfoc); 
    // HAL_GetUIDw0();
    // HAL_GetUIDw1();
    // HAL_GetUIDw2();

    FLASH_DataTypeDef flash_data = {0}; //temporary
    FOC_FLASH_ReadData(&flash_data);
    if(flash_data.contains_data == 1 && flash_data.struct_terminator == FLASH_STRUCT_TERMINATOR){
        memcpy(&hfoc.flash_data, &flash_data, sizeof(FLASH_DataTypeDef)); //only copy the data if it is valid
    }else{
        // hfoc.flash_data.contains_data_flag = 1;
        // FOC_FLASH_WriteData(&hfoc.flash_data);
    }

    WS2812b_Setup(&htim4, TIM_CHANNEL_1);


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
    
    hfoc.voltage_limit = VOLTAGE_LIMIT;

    FOC_SetPWMCCRPointers(&hfoc, &htim1.Instance->CCR3, &htim1.Instance->CCR2, &htim1.Instance->CCR1, PWM_CLOCK_DIVIDER); //set the pwm ccr register pointers

    FOC_SetPhaseVoltages(&hfoc, (PhaseVoltagesTypeDef){0.0f, 0.0f, 0.0f});

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4); //this triggers the adc

    HAL_GPIO_WritePin(INL_ALL_GPIO_Port, INL_ALL_Pin, GPIO_PIN_SET);

    HAL_TIM_Base_Start(&htim1); //pwm timer and also adc trigger
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1_buffer, ADC1_CHANNELS * CURRENT_LOOP_CLOCK_DIVIDER * 2); //start adc in dma mode for the current and vbus
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc2_buffer, ADC2_CHANNELS * CURRENT_LOOP_CLOCK_DIVIDER * 2); //adc for temp sensor

    
    HAL_TIM_Base_Start(&htim2); //timer for the counter
    HAL_TIM_Base_Start_IT(&htim7); //timer for the debug


    /* PID controllers */
    PID_Init(&hfoc.pid_current_d, (1.0f/CURRENT_LOOP_FREQUENCY), 0.01f, &hfoc.flash_data.limits.max_dq_voltage, &hfoc.flash_data.controller.PID_gains_d, 0);
    PID_Init(&hfoc.pid_current_q, (1.0f/CURRENT_LOOP_FREQUENCY), 0.01f, &hfoc.flash_data.limits.max_dq_voltage, &hfoc.flash_data.controller.PID_gains_q, 0);

    PID_Init(&hfoc.pid_speed, (1.0f/(CURRENT_LOOP_FREQUENCY / SPEED_LOOP_CLOCK_DIVIDER)), 0.01f, &hfoc.flash_data.limits.max_dq_current, &hfoc.flash_data.controller.PID_gains_speed, 0);
    PID_Init(&hfoc.pid_position, (1.0f/(CURRENT_LOOP_FREQUENCY / SPEED_LOOP_CLOCK_DIVIDER)), 0.01f, &hfoc.flash_data.limits.max_dq_current, &hfoc.flash_data.controller.PID_gains_position, 1);

    // hfoc.flash_data.controller.PID_gains_speed.Kp = 0.0f;
    // hfoc.flash_data.controller.PID_gains_speed.Ki = 0.0f;
    // hfoc.speed_setpoint = 300.0f;

    /* UART */
    Log_Setup(&huart3);

    GenerateNtcLut(); //generate the NTC lookup table


    uint32_t rand32;
    uint8_t rand8;
    if (HAL_RNG_GenerateRandomNumber(&hrng, &rand32) == HAL_OK)
    {
        rand8 = (uint8_t)(rand32 & 0xFF);
    }

    Log_printf("\nFOC Setup Complete! Here is a random 8-bit number: %d\n", rand8);

}


static uint32_t start_time = 0;
static uint32_t execution_time = 0;
static uint32_t max_execution_time = 0;

static uint32_t adc1_start_time = 0;
static uint32_t adc1_time = 0;
static uint32_t max_adc1_time = 0;

static uint32_t adc2_start_time = 0;
static uint32_t adc2_time = 0;
static uint32_t max_adc2_time = 0;

static uint32_t log_start_time = 0;
static uint32_t log_time = 0;
static uint32_t max_log_time = 0;

uint8_t timeout_flag = 0;
static void FOC_StateLoop();

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
                hfoc.vbus            = hfoc.vbus            * (1 - ADC_LOOP_ALPHA) + ADC_LOOP_ALPHA * VOLTAGE_SENSE_CONVERSION_FACTOR * (float)(adc1_buffer[i + 3]);
                
                //TODO: saturated
                if(adc1_buffer[i + 0] > 4000 || adc1_buffer[i + 1] > 4000 || adc1_buffer[i + 2] > 4000 || adc1_buffer[i + 3] > 4000){
                    __NOP();
                }
            }
            FOC_CalculateBusCurrent(&hfoc); //calculate the bus current based on the phase currents and voltages

            foc_adc1_measurement_flag = 1; //triggered after the adc conversion is complete (either first or second half of the buffer)
            adc1_half_complete_flag = 0;
            adc1_complete_flag = 0;

        adc1_time = __HAL_TIM_GET_COUNTER(&htim2) - adc1_start_time;
        if (adc1_time > max_adc1_time) {
            max_adc1_time = adc1_time;
        }
    }

    if(adc2_half_complete_flag || adc2_complete_flag){
        adc2_start_time = __HAL_TIM_GET_COUNTER(&htim2);

            int start_index = adc2_half_complete_flag ? 0 : CURRENT_LOOP_CLOCK_DIVIDER * ADC2_CHANNELS;
            int end_index = start_index + CURRENT_LOOP_CLOCK_DIVIDER * ADC2_CHANNELS;

            for (int i = start_index; i < end_index; i += ADC2_CHANNELS) {
                if(adc2_buffer[i] > 0){
                    hfoc.NTC_resistance = hfoc.NTC_resistance * (1 - TEMP_LOOP_ALPHA) + TEMP_LOOP_ALPHA * 100e3f * ((4095.0f / (float)adc2_buffer[i]) - 1);
                } else{
                    hfoc.NTC_resistance = 0.0f;
                }
            }

            hfoc.NTC_temp = GetNtcTemperature(hfoc.NTC_resistance);

            adc2_half_complete_flag = 0;
            adc2_complete_flag = 0;

        adc2_time = __HAL_TIM_GET_COUNTER(&htim2) - adc2_start_time;
        if (adc2_time > max_adc2_time) {
            max_adc2_time = adc2_time;
        }
    }


    if(foc_adc1_measurement_flag){ //runs at CURRENT_LOOP_FREQUENCY
        log_start_time = __HAL_TIM_GET_COUNTER(&htim2);
        Log_Loop();
        log_time = __HAL_TIM_GET_COUNTER(&htim2) - log_start_time;
        if (log_time > max_log_time) {
            max_log_time = log_time;
        }

        if(DRV8323_CheckFault(&hfoc.hdrv8323)){ //check for motor driver fault
            DRV8323_Disable(&hfoc.hdrv8323); //disable the driver
            HAL_GPIO_WritePin(INL_ALL_GPIO_Port, INL_ALL_Pin, 0); //disable the inverter
            FOC_SetPhaseVoltages(&hfoc, (PhaseVoltagesTypeDef){0.0f, 0.0f, 0.0f});
            Current_FOC_State = FOC_STATE_ERROR;
        }

        if(hfoc.NTC_temp > MOTOR_MAX_TEMP || hfoc.NTC_temp < 0.0f){ //check for over temperature or disconnected NTC
            DRV8323_Disable(&hfoc.hdrv8323); //disable the driver
            HAL_GPIO_WritePin(INL_ALL_GPIO_Port, INL_ALL_Pin, 0); //disable the inverter
            FOC_SetPhaseVoltages(&hfoc, (PhaseVoltagesTypeDef){0.0f, 0.0f, 0.0f});
            Current_FOC_State = FOC_STATE_ERROR;
        }


        FOC_StateLoop();



        foc_adc1_measurement_flag = 0;
    }
    

    execution_time = __HAL_TIM_GET_COUNTER(&htim2) - start_time;
    if (execution_time > max_execution_time) {
        max_execution_time = execution_time;
    }
    if(execution_time > 100){ //max 125us for 8kHz loop
        Log_printf("Execution time: %dus\n", (int)execution_time);
    }
}


static void FOC_StateLoop(){
    FOC_UpdateEncoderAngle(&hfoc);
    FOC_UpdateEncoderSpeed(&hfoc, CURRENT_LOOP_FREQUENCY, 0.3f);

    FOC_LoopStatusTypeDef ret;
    switch(Current_FOC_State){
    
        case FOC_STATE_INIT:
            if(hfoc.flash_data.contains_data == 1){
                Log_printf("Flash data loaded!: Offset: %d\n", (int)(hfoc.flash_data.encoder.mechanical_offset * 1000));
            }else{
                Log_printf("Flash data is missing!\n");
            }
            Current_FOC_State = FOC_STATE_BOOTUP_SOUND;
            break;
        case FOC_STATE_BOOTUP_SOUND:
            if(FOC_BootupSound(&hfoc, CURRENT_LOOP_FREQUENCY)){
                Current_FOC_State = FOC_STATE_CURRENT_SENSOR_CALIBRATION;
                __NOP();
            }
            break;
        case FOC_STATE_CURRENT_SENSOR_CALIBRATION:
            if(FOC_CurrentSensorCalibration(&hfoc) == FOC_LOOP_COMPLETED){
                hfoc.adc_calibrated = 1;
                Current_FOC_State = FOC_STATE_CHECKLIST;
            }
            break;
        case FOC_STATE_CHECKLIST:
            if(hfoc.flash_data.encoder.offset_valid != 1){
                Current_FOC_State = FOC_STATE_ALIGNMENT;
            }else if(hfoc.flash_data.motor.phase_resistance_valid != 1 || hfoc.flash_data.motor.phase_inductance_valid != 1){
                Current_FOC_State = FOC_STATE_IDENTIFY;
            }else if(hfoc.flash_data.controller.current_PID_gains_valid != 1){
                Current_FOC_State = FOC_STATE_PID_AUTOTUNE;
            }else if(hfoc.flash_data.controller.anticogging_data_valid != 1){
                Current_FOC_State = FOC_STATE_ANTICOGGING;
            }else{
                Current_FOC_State = FOC_STATE_RUN;
            }
            break;
        case FOC_STATE_GENERAL_TEST:
            if(General_LED_Loop()){
                Current_FOC_State = FOC_STATE_RUN;
                __NOP();
            }
            break;
        case FOC_STATE_CALIBRATION:
            break;

        case FOC_STATE_IDENTIFY:
            ret = FOC_MotorIdentification(&hfoc);
            if(ret == FOC_LOOP_COMPLETED){
                hfoc.flash_data.controller.current_PID_gains_valid = 0;
                Current_FOC_State = FOC_STATE_CHECKLIST;
            } else if(ret == FOC_LOOP_ERROR){
                Current_FOC_State = FOC_STATE_ERROR;
            } 
            break;

        case FOC_STATE_PID_AUTOTUNE:
            ret = FOC_PIDAutotune(&hfoc);
            if(ret == FOC_LOOP_COMPLETED){
                hfoc.flash_data.controller.anticogging_data_valid = 0;
                hfoc.flash_data.controller.current_PID_gains_valid = 1;
                hfoc.flash_data.controller.current_PID_FF_enabled = 1;
                Current_FOC_State = FOC_STATE_CHECKLIST;
            } else if(ret == FOC_LOOP_ERROR){
                Current_FOC_State = FOC_STATE_ERROR;
            }
            break;

        case FOC_STATE_ALIGNMENT:
            __NOP();
            ret = FOC_Alignment(&hfoc, 1.0f);
            if(ret == FOC_LOOP_COMPLETED){
                hfoc.flash_data.controller.anticogging_data_valid = 0;
                Current_FOC_State = FOC_STATE_ALIGNMENT_TEST;
            } else if(ret == FOC_LOOP_ERROR){
                Current_FOC_State = FOC_STATE_ERROR;
            }
            break;

        case FOC_STATE_ALIGNMENT_TEST:
            if(Alignment_Test_Loop(&hfoc, 1.0f) == FOC_LOOP_COMPLETED){
                Current_FOC_State = FOC_STATE_CHECKLIST;
            }
            break;
        case FOC_STATE_ANTICOGGING:
            ret = FOC_AntiCoggingMeasurement(&hfoc);
            if(ret == FOC_LOOP_COMPLETED){
                hfoc.flash_data.controller.anticogging_data_valid = 1;
                Current_FOC_State = FOC_STATE_CHECKLIST;
            } else if(ret == FOC_LOOP_ERROR){
                Current_FOC_State = FOC_STATE_ERROR;
            }
            break;
        case FOC_STATE_RUN:
            Current_Loop(&hfoc);
            static uint8_t current_loop_counter = 0;
            if(++current_loop_counter >= SPEED_LOOP_CLOCK_DIVIDER){
                current_loop_counter = 0;
                Speed_Loop(&hfoc); //runs at CURRENT_LOOP_FREQUENCY / CURRENT_LOOP_CLOCK_DIVIDER
            }


            if(debug_loop_flag){
                Log_printf("Vq:%d, Iq:%d, Iq_set:%d, Mspeed:%d, Vbus:%d, Temp:%d, Eang%d, Mang:%d\n",
                (int)(hfoc.dq_voltage.q * 1000), (int)(hfoc.dq_current.q * 1000), 
                (int)(hfoc.dq_current_setpoint.q * 1000), (int)(hfoc.encoder_speed_mechanical * 1000),
                (int)(hfoc.vbus * 10), (int)(hfoc.NTC_temp * 10),
                (int)(hfoc.encoder_angle_electrical * 1000), (int)(hfoc.encoder_angle_mechanical * 1000));

                // Log_printf("Vq:%d,Vd:%d,Id:%d,Iq:%d,Id_set:%d,Iq_set:%d,EAngle:%d,Espeed:%d,Vbus:%d,Temp:%d\n",
                // (int)(hfoc.dq_voltage.q * 1000), (int)(hfoc.dq_voltage.d * 1000),
                // (int)(hfoc.dq_current.d * 1000), (int)(hfoc.dq_current.q * 1000),
                // (int)(hfoc.dq_current_setpoint.d * 1000), (int)(hfoc.dq_current_setpoint.q * 1000),
                // (int)(hfoc.encoder_angle_electrical * 1000), (int)(hfoc.encoder_speed_electrical * 1000),
                // (int)(hfoc.vbus * 10), (int)(hfoc.NTC_temp * 10));

                // Log_printf("Time: %d, ADC1 Time: %d, ADC2 Time: %d, Log Time: %d, Count:%d\n",
                //     (int)max_execution_time, (int)max_adc1_time, (int)max_adc2_time, (int)max_log_time);

                debug_loop_flag = 0;
            }
            break;
        case FOC_STATE_ERROR:
            if(Error_LED_Loop()){
                // Current_FOC_State = FOC_GENERAL_TEST;
            }
            break;
        case FOC_STATE_FLASH_SAVE:

            if(FOC_FLASH_WriteData(&hfoc.flash_data) != FLASH_OK){
                Current_FOC_State = FOC_STATE_ERROR;
            }
            
            Current_FOC_State = FOC_STATE_RUN;

            break;
        case FOC_STATE_OPENLOOP:
            FOC_OpenLoop(&hfoc, hfoc.speed_setpoint, 1.0f, CURRENT_LOOP_FREQUENCY);

            if(debug_loop_flag){
                debug_loop_flag = 0;
            }
            break;
        
    }
}




/**
  * @brief 
  * @note 
  * @param None
  * @retval uint8_t: 0 if the loop is not complete, 1 if the loop is complete
  */
static uint8_t General_LED_Loop(){

    static uint8_t step = 0;
    static uint32_t next_step_time = 0;

    switch(step){
        case 0:
            if(HAL_GetTick() >= next_step_time){

                HAL_GPIO_WritePin(DEBUG_LED0_GPIO_Port, DEBUG_LED0_Pin, 1);
                for(int i = 0; i < WS2812B_NUMBER_OF_LEDS; i++){
                    WS2812b_SetColor(i, 0, 25, 0); //green
                }
                WS2812b_Send();

                step++;
                next_step_time = HAL_GetTick() + 100; //wait 1ms before the next step
            }
            break;
        case 1:
            if(HAL_GetTick() >= next_step_time){

                HAL_GPIO_WritePin(DEBUG_LED0_GPIO_Port, DEBUG_LED0_Pin, 0);
                for(int i = 0; i < WS2812B_NUMBER_OF_LEDS; i++){
                    WS2812b_SetColor(i, 0, 0, 0); 
                }
                WS2812b_Send();
                
                step++;
                next_step_time = HAL_GetTick() + 100; //wait 1ms before the next step
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

/**
  * @brief 
  * @note 
  * @param None
  * @retval uint8_t: 0 if the loop is not complete, 1 if the loop is complete
  */
static uint8_t Error_LED_Loop(){

    static uint8_t step = 0;
    static uint32_t next_step_time = 0;

    switch(step){
        case 0:
            if(HAL_GetTick() >= next_step_time){
                
                HAL_GPIO_WritePin(DEBUG_LED0_GPIO_Port, DEBUG_LED0_Pin, 1);
                for(int i = 0; i < WS2812B_NUMBER_OF_LEDS; i++){
                    WS2812b_SetColor(i, 25, 0, 0); //green
                }
                WS2812b_Send();

                step++;
                next_step_time = HAL_GetTick() + 100; //wait 1ms before the next step
            }
            break;
        case 1:
            if(HAL_GetTick() >= next_step_time){

                HAL_GPIO_WritePin(DEBUG_LED0_GPIO_Port, DEBUG_LED0_Pin, 0);
                for(int i = 0; i < WS2812B_NUMBER_OF_LEDS; i++){
                    WS2812b_SetColor(i, 0, 0, 0); 
                }
                WS2812b_Send();
                
                step++;
                next_step_time = HAL_GetTick() + 100; //wait 1ms before the next step
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






// /**
//   * @brief 
//   * @note 
//   * @param None
//   * @retval uint8_t: 0 if the loop is not complete, 1 if the loop is complete
//   */
//  static uint8_t General_LED_Loop(){

//     static uint8_t step = 0;
//     static uint32_t next_step_time = 0;

//     switch(step){
//         case 0:
//             if(HAL_GetTick() >= next_step_time){
                
//                 step++;
//                 next_step_time = HAL_GetTick() + 100;
//             }
//             break;
//         case 1:
//             if(HAL_GetTick() >= next_step_time){

                
//                 step++;
//                 next_step_time = HAL_GetTick() + 100;
//             }
//             break;
//         default:
//             if(HAL_GetTick() >= next_step_time){
//                 step = 0;
//                 return 1; // complete
//             }
//             break;
//     }

// return 0;
// }




void Log_ProcessRxPacket(const char* packet, uint16_t Length){
    for(int i = 0; i < Length; i++){
        if(packet[i] == 'D'){
            // Current_FOC_State = FOC_STATE_ANTICOGGING;
            if(packet[i+1] == 'a'){
                hfoc.flash_data.controller.anticogging_FF_enabled = !hfoc.flash_data.controller.anticogging_FF_enabled;
            } else if(packet[i+1] == 'f'){
                hfoc.flash_data.controller.current_PID_FF_enabled = !hfoc.flash_data.controller.current_PID_FF_enabled;
            } 
        }
        if(packet[i] == 'A'){
            // Current_FOC_State = FOC_STATE_ALIGNMENT;
        }
        if(packet[i] == 'R'){
            Current_FOC_State = FOC_STATE_RUN;
        }
        if(packet[i] == 'E'){
            // Current_FOC_State = FOC_STATE_IDENTIFY;
        }
        if(packet[i] == 'F'){
            Current_FOC_State = FOC_STATE_FLASH_SAVE;
        }
        if(packet[i] == 'M'){
            if(packet[i+1] == 's'){
                hfoc.flash_data.controller.speed_PID_enabled = 1;
                hfoc.flash_data.controller.position_PID_enabled = 0;
            } else if(packet[i+1] == 'p'){
                hfoc.flash_data.controller.position_PID_enabled = 1;
                hfoc.flash_data.controller.speed_PID_enabled = 0;
            } else if(packet[i+1] == 'o'){
                hfoc.flash_data.controller.speed_PID_enabled = 0;
                hfoc.flash_data.controller.position_PID_enabled = 0;
            }
        }
        if(packet[i] == 'O'){
            Current_FOC_State = FOC_STATE_OPENLOOP;
        }
        if(packet[i] == 'K'){
            hfoc.motor_disable_flag = 1;
        }
        if(packet[i] == 'T'){
            timeout_flag = !timeout_flag;
        }
        if(packet[i] == 'C'){
            hfoc.flash_data.encoder.offset_valid = 0;
            hfoc.flash_data.motor.phase_resistance_valid = 0;
            hfoc.flash_data.motor.phase_inductance_valid = 0;
            hfoc.flash_data.controller.current_PID_gains_valid = 0;
            Current_FOC_State = FOC_STATE_CHECKLIST;
        }
    }

    if(packet[0] == 'P' && packet[1] == 'd'){
        int Pd = 0;
        sscanf(packet, "Pd%d", &Pd);
        hfoc.flash_data.controller.PID_gains_d.Kp = (float)Pd / 1000.0f;
    }
    if(packet[0] == 'P' && packet[1] == 'q'){
        int Pq = 0;
        sscanf(packet, "Pq%d", &Pq);
        hfoc.flash_data.controller.PID_gains_q.Kp = (float)Pq / 1000.0f;
    }
    if(packet[0] == 'P' && packet[1] == 's'){
        int Ps = 0;
        sscanf(packet, "Ps%d", &Ps);
        hfoc.flash_data.controller.PID_gains_speed.Kp = (float)Ps / 1000.0f;
    }
    
    if(packet[0] == 'I' && packet[1] == 'd'){
        int Id = 0;
        sscanf(packet, "Id%d", &Id);
        hfoc.flash_data.controller.PID_gains_d.Ki = (float)Id / 1000.0f;
    }
    if(packet[0] == 'I' && packet[1] == 'q'){
        int Iq = 0;
        sscanf(packet, "Iq%d", &Iq);
        hfoc.flash_data.controller.PID_gains_q.Ki = (float)Iq / 1000.0f;
    }
    if(packet[0] == 'I' && packet[1] == 's'){
        int Is = 0;
        sscanf(packet, "Is%d", &Is);
        hfoc.flash_data.controller.PID_gains_speed.Ki = (float)Is / 1000.0f;
    }

    if(packet[0] == 'S' && packet[1] == 'q'){
        int Sq = 0;
        sscanf(packet, "Sq%d", &Sq);
        hfoc.dq_current_setpoint.q = (float)Sq / 1000.0f;
    }
    if(packet[0] == 'S' && packet[1] == 'd'){
        int Sd = 0;
        sscanf(packet, "Sd%d", &Sd);
        hfoc.dq_current_setpoint.d = (float)Sd / 1000.0f;
    }
    if(packet[0] == 'S' && packet[1] == 's'){
        int Ss = 0;
        sscanf(packet, "Ss%d", &Ss);
        hfoc.speed_setpoint = (float)Ss;
    }
    if(packet[0] == 'S' && packet[1] == 'p'){
        int Sp = 0;
        sscanf(packet, "Sp%d", &Sp);
        hfoc.angle_setpoint = (float)Sp / 1000.0f;
        normalize_angle2(&hfoc.angle_setpoint); //normalize the angle to [-pi, pi]
    }
}












void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    Log_UART_TxCpltCallback(huart);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    Log_UARTEx_RxEventCallback(huart, Size);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) { //when the adc conversion is complete
    if(hadc->Instance == ADC1){ 
        if(!adc1_complete_flag){
            adc1_complete_flag = 1;  
        }
    } else if(hadc->Instance == ADC2){
        if(!adc2_complete_flag){
            adc2_complete_flag = 1;  
        }
    }
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc) { //when the adc conversion is half complete
    if(hadc->Instance == ADC1){
        if(!adc1_half_complete_flag){
            adc1_half_complete_flag = 1;
        }
    } else if(hadc->Instance == ADC2){
        if(!adc2_half_complete_flag){
            adc2_half_complete_flag = 1;
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if(htim->Instance == TIM6){ //not used
        
    } else if(htim->Instance == TIM7){
        debug_loop_flag = 1;
    } else if(htim->Instance == TIM17){ //not used

    }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
    if(htim == &htim4){
        WS2812b_PulseFinishedCallback();
    }
}