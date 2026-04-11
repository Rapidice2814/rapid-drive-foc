#include <math.h>
#include <stdio.h>
#include <string.h>

#include "FOC.h"
#include "FOC_Utils.h"
#include "DRV8323_Driver.h"
#include "Utils.h"
#include "PID.h"
#include "FOC_Flash.h"
#include "WS2812b_Driver.h"
#include "FOC_Loops.h"
#include "Logging.h"
#include "FOC_Config.h"
#include "FOC_CAN.h"
#include "FOC_ADC.h"
#include "FOC_Diagnostics.h"
#include "FOC_Statemachine.h"
#include "Cordic.h"

extern FDCAN_HandleTypeDef hfdcan1;

extern I2C_HandleTypeDef hi2c1;

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


/* flags, used for the interrupts*/

volatile uint8_t foc_outer_loop_flag = 0;
volatile uint8_t debug_loop_flag = 0;


/* ADC gains*/
#define CURRENT_SENSE_CONVERSION_FACTOR ((3.3f/4096.0f) / (CURRENT_SENSE_RESISTANCE * CSA_GAIN_VALUE))
#define VOLTAGE_SENSE_CONVERSION_FACTOR ((3.3f/4096.0f) / VBUS_VOLTAGE_DIVIDER_RATIO)


void FOC_Setup(){
    FOC_Init(&hfoc); 
    // HAL_GetUIDw0();
    // HAL_GetUIDw1();
    // HAL_GetUIDw2();

    if(FOC_FLASH_ReadData(&hfoc.flash_data) != FLASH_OK){
        FOC_FLASH_SetDefault(&hfoc.flash_data);
    }

    WS2812b_Setup(&htim4, TIM_CHANNEL_1);

    if(DRV8323_SetPins(&hfoc.hdrv8323, &hspi2, DRV_NCS_GPIO_Port, DRV_NCS_Pin, DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, INL_ALL_GPIO_Port, INL_ALL_Pin, DRV_NFAULT_GPIO_Port, DRV_NFAULT_Pin) != DRV8323_OK){
        Error_Handler();
    }

    if(DRV8323_Init(&hfoc.hdrv8323) != DRV8323_OK){
        while(1){
            HAL_GPIO_TogglePin(DEBUG_LED0_GPIO_Port, DEBUG_LED0_Pin);
            HAL_Delay(100);
        };
    }

    if(AS5047P_SetPins(&hfoc.has5047p, &hspi2, AS_NCS_GPIO_Port, AS_NCS_Pin) != AS5047P_OK){
        Error_Handler();
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

    DRV8323_ExitHighImpedance(&hfoc.hdrv8323);

    HAL_TIM_Base_Start(&htim1); //pwm timer and also adc trigger
    FOC_ADC_Setup(); //setup the adc for current and voltage measurements

    HAL_TIM_Base_Start(&htim2); //timer for the counter
    HAL_TIM_Base_Start_IT(&htim7); //timer for the debug

    /* PID controllers */
    PID_Init(&hfoc.pid_current_d, (1.0f/CURRENT_LOOP_FREQUENCY), 0.01f, &hfoc.flash_data.limits.max_dq_voltage, &hfoc.flash_data.controller.PID_gains_d, 0);
    PID_Init(&hfoc.pid_current_q, (1.0f/CURRENT_LOOP_FREQUENCY), 0.01f, &hfoc.flash_data.limits.max_dq_voltage, &hfoc.flash_data.controller.PID_gains_q, 0);

    PID_Init(&hfoc.pid_speed, (1.0f/(CURRENT_LOOP_FREQUENCY / SPEED_LOOP_CLOCK_DIVIDER)), 0.01f, &hfoc.flash_data.limits.max_dq_current, &hfoc.flash_data.controller.PID_gains_speed, 0);
    PID_Init(&hfoc.pid_position, (1.0f/(CURRENT_LOOP_FREQUENCY / SPEED_LOOP_CLOCK_DIVIDER)), 0.01f, &hfoc.flash_data.limits.max_dq_current, &hfoc.flash_data.controller.PID_gains_position, 1);

    /* UART */
    Log_Setup(&huart3);

    GenerateNtcLut(); //generate the NTC lookup table


    uint32_t rand32;
    uint8_t rand8;
    if (HAL_RNG_GenerateRandomNumber(&hrng, &rand32) == HAL_OK)
    {
        rand8 = (uint8_t)(rand32 & 0xFF);
    }

    hfoc.phfdcan = &hfdcan1; //set the CAN handle pointer
    FOC_SetNodeId(&hfoc, 1);
    // FOC_TransmitCANMessage(&hfoc, CMD_HEARTBEAT);


    Log_printf("\nFOC Setup Complete! Here is a random 8-bit number: %d\n", rand8);

}


static uint32_t start_time = 0;
static uint32_t execution_time = 0;
static uint32_t max_execution_time = 0;

static uint32_t adc1_start_time = 0;
static uint32_t adc1_time = 0;
static uint32_t max_adc1_time = 0;

static uint32_t log_start_time = 0;
static uint32_t log_time = 0;
static uint32_t max_log_time = 0;

static uint32_t state_start_time = 0;
static uint32_t state_time = 0;
static uint32_t max_state_time = 0;

void FOC_Loop(){
    start_time = __HAL_TIM_GET_COUNTER(&htim2);


    // FOC_TransmitCyclicCANMessage(&hfoc);
    // FOC_ProcessCANMessage(&hfoc);
    
    adc1_start_time = __HAL_TIM_GET_COUNTER(&htim2);

    if(FOC_ADC_Measure(&hfoc.adc_values) == FOC_ADC_MEASUREMENT_COMPLETE){ //runs at CURRENT_LOOP_FREQUENCY
        adc1_time = __HAL_TIM_GET_COUNTER(&htim2) - adc1_start_time;
        if (adc1_time > max_adc1_time) {
            max_adc1_time = adc1_time;
        }

        float ibus = CalculateBusCurrent(hfoc.adc_values.phase_current, hfoc.phase_voltage, hfoc.adc_values.vbus);
        hfoc.ibus = 0.99f * hfoc.ibus + 0.01f * ibus;

        FOC_UpdateEncoderAngle(&hfoc);
        FOC_UpdateEncoderSpeed(&hfoc, CURRENT_LOOP_FREQUENCY);


        log_start_time = __HAL_TIM_GET_COUNTER(&htim2);
        Log_Loop();
        log_time = __HAL_TIM_GET_COUNTER(&htim2) - log_start_time;
        if (log_time > max_log_time) {
            max_log_time = log_time;
        }

        FOC_CheckErrors(&hfoc);

        state_start_time = __HAL_TIM_GET_COUNTER(&htim2);

        FOC_StateLoop(&hfoc);

        state_time = __HAL_TIM_GET_COUNTER(&htim2) - state_start_time;
        if (state_time > max_state_time) {
            max_state_time = state_time;
        }


    }
    

    execution_time = __HAL_TIM_GET_COUNTER(&htim2) - start_time;
    if (execution_time > max_execution_time) {
        max_execution_time = execution_time;
    }

    if(execution_time > 1200){ //max 125us for 8kHz loop
        Log_printf("Execution time: %dus\n", (int)execution_time);
    }
}




void Log_ProcessRxPacket(const char* packet, uint16_t Length){
    for(int i = 0; i < Length; i++){
        if(packet[i] == 'D'){
            if(packet[i+1] == 'a'){
                hfoc.flash_data.controller.anticogging_FF_enabled = !hfoc.flash_data.controller.anticogging_FF_enabled;
            } else if(packet[i+1] == 'f'){
                hfoc.flash_data.controller.current_PID_FF_enabled = !hfoc.flash_data.controller.current_PID_FF_enabled;
            } 
        }
        if(packet[i] == 'A'){
            hfoc.state = FOC_STATE_ANTICOGGING;
        }
        if(packet[i] == 'R'){
            hfoc.state = FOC_STATE_RESET;
        }
        if(packet[i] == 'E'){
            hfoc.state = FOC_STATE_ERROR;
        }
        if(packet[i] == 'F'){
            hfoc.state = FOC_STATE_FLASH_SAVE;
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
            hfoc.state = FOC_STATE_OPENLOOP;
        }
        if(packet[i] == 'K'){
            hfoc.motor_disable_flag = 1;
        }
        if(packet[i] == 'T'){

        }
        if(packet[i] == 'C'){
            hfoc.flash_data.encoder.offset_valid = 0;
            hfoc.flash_data.motor.phase_resistance_valid = 0;
            hfoc.flash_data.motor.phase_inductance_valid = 0;
            hfoc.flash_data.controller.current_PID_gains_valid = 0;
            hfoc.state = FOC_STATE_CHECKLIST;
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
    if(packet[0] == 'P' && packet[1] == 'p'){
        int Pp = 0;
        sscanf(packet, "Pp%d", &Pp);
        hfoc.flash_data.controller.PID_gains_position.Kp = (float)Pp / 1000.0f;
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
    if(packet[0] == 'I' && packet[1] == 'p'){
        int Ip = 0;
        sscanf(packet, "Ip%d", &Ip);
        hfoc.flash_data.controller.PID_gains_position.Ki = (float)Ip / 1000.0f;
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
        normalize_angle_pm_pi(&hfoc.angle_setpoint); //normalize the angle to [-pi, pi]
    }

    if(packet[0] == 'L' && packet[1] == 'i'){
        int Li = 0;
        sscanf(packet, "Li%d", &Li);
        hfoc.flash_data.limits.max_dq_current = (float)Li / 1000.0f;
    }
    if(packet[0] == 'L' && packet[1] == 'v'){
        int Lv = 0;
        sscanf(packet, "Lv%d", &Lv);
        hfoc.flash_data.limits.max_dq_voltage = (float)Lv / 1000.0f;
    }
}












void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    Log_UART_TxCpltCallback(huart);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    Log_UARTEx_RxEventCallback(huart, Size);
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

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
    CAN_RxFifo0Callback(hfdcan, RxFifo0ITs);
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim){
    if(htim->Instance == TIM1){
        // HAL_GPIO_WritePin(PB2_GPIO_Port, PB2_Pin, GPIO_PIN_SET);
        // HAL_GPIO_WritePin(PB2_GPIO_Port, PB2_Pin, GPIO_PIN_RESET);  
    }
}