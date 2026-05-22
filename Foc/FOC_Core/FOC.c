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
#include "FOC_USB_Debug.h"
#include "FOC_Config.h"
#include "FOC_CAN.h"
#include "FOC_ADC.h"
#include "FOC_Diagnostics.h"
#include "FOC_States.h"
#include "Cordic.h"
#include "Timing.h"

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

    /* Setup pins and peripherals */

    WS2812b_Setup(&htim4, TIM_CHANNEL_1);

    if(DRV8323_SetPins(&hfoc.hdrv8323, &hspi2, DRV_NCS_GPIO_Port, DRV_NCS_Pin, DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, INL_ALL_GPIO_Port, INL_ALL_Pin, DRV_NFAULT_GPIO_Port, DRV_NFAULT_Pin) != DRV8323_OK){
        Error_Handler();
    }
    
    if(AS5047P_SetPins(&hfoc.has5047p, &hspi2, AS_NCS_GPIO_Port, AS_NCS_Pin) != AS5047P_OK){
        Error_Handler();
    }

    if(DRV8323_Init(&hfoc.hdrv8323) != DRV8323_OK){
        while(1){
            HAL_GPIO_TogglePin(DEBUG_LED0_GPIO_Port, DEBUG_LED0_Pin);
            HAL_Delay(100);
        };
    }

    if(AS5047P_Init(&hfoc.has5047p) != AS5047P_OK){
        while(1){
            HAL_GPIO_TogglePin(DEBUG_LED0_GPIO_Port, DEBUG_LED0_Pin);
            HAL_Delay(100);
        };
    }

    if(FOC_SetEncoderPointer(&hfoc, &htim3.Instance->CNT) != FOC_OK){
        Error_Handler();
    }

    if(FOC_SetPWMCCRPointers(&hfoc, &htim1.Instance->CCR3, &htim1.Instance->CCR2, &htim1.Instance->CCR1, PWM_CLOCK_DIVIDER) != FOC_OK){
        Error_Handler();
    }



    for(int i = 0; i < WS2812B_NUMBER_OF_LEDS; i++) {
        WS2812b_SetColor(i, 0, 0, 0);
    }
    WS2812b_Send();

    HAL_TIM_Base_Start(&htim3); //start encoder timer

    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4); //this triggers the adc
    HAL_TIM_Base_Start(&htim1); //pwm timer and also adc trigger

    FOC_ADC_Setup(); //setup the ADCs

    FunctionTimer_Init(&htim2); //timer for the debug
    // HAL_TIM_Base_Start_IT(&htim7); //not used

    /* PID controllers */
    PID_Init(&hfoc.pid_current_d, (1.0f/CURRENT_LOOP_FREQUENCY), 0.01f, &hfoc.flash_data.limits.max_dq_voltage, &hfoc.flash_data.controller.PID_gains_d, 0);
    PID_Init(&hfoc.pid_current_q, (1.0f/CURRENT_LOOP_FREQUENCY), 0.01f, &hfoc.flash_data.limits.max_dq_voltage, &hfoc.flash_data.controller.PID_gains_q, 0);

    PID_Init(&hfoc.pid_speed, (1.0f/(CURRENT_LOOP_FREQUENCY / SPEED_LOOP_CLOCK_DIVIDER)), 0.01f, &hfoc.flash_data.limits.max_dq_current, &hfoc.flash_data.controller.PID_gains_speed, 0);
    PID_Init(&hfoc.pid_position, (1.0f/(CURRENT_LOOP_FREQUENCY / SPEED_LOOP_CLOCK_DIVIDER)), 0.01f, &hfoc.flash_data.limits.max_dq_current, &hfoc.flash_data.controller.PID_gains_position, 1);


    /* USB Debug */
    FOC_USB_Setup();
    

    uint32_t rand32 = 0;
    if (HAL_RNG_GenerateRandomNumber(&hrng, &rand32) != HAL_OK)    {
        Error_Handler();
    }
    uint8_t rand8 = (uint8_t)(rand32 & 0xFF);

    hfoc.phfdcan = &hfdcan1; //set the CAN handle pointer
    FOC_SetNodeId(&hfoc, 1);
    // FOC_TransmitCANMessage(&hfoc, CMD_HEARTBEAT);

    FOC_SetPhaseVoltages(&hfoc, (PhaseVoltagesTypeDef){0.0f, 0.0f, 0.0f});
    DRV8323_ExitHighImpedance(&hfoc.hdrv8323);

    USB_printf("\nFOC Setup Complete! Here is a random 8-bit number: %d\n", rand8);

}


void FOC_Loop(){
    uint32_t start_time = get_current_time();


    // FOC_TransmitCyclicCANMessage(&hfoc);
    // FOC_ProcessCANMessage(&hfoc);
    
    uint32_t adc_start_time = get_current_time();

    if(FOC_ADC_Measure(&hfoc.adc_values) == FOC_ADC_MEASUREMENT_COMPLETE){ //runs at CURRENT_LOOP_FREQUENCY
        calculate_execution_time(&hfoc.execution_time.adc_max, adc_start_time);

        float ibus = CalculateBusCurrent(hfoc.adc_values.phase_current, hfoc.phase_voltage, hfoc.adc_values.vbus);
        hfoc.ibus = 0.99f * hfoc.ibus + 0.01f * ibus;

        FOC_UpdateEncoderAngle(&hfoc);
        FOC_UpdateEncoderSpeed(&hfoc, CURRENT_LOOP_FREQUENCY);

        FOC_CheckErrors(&hfoc);
        
        uint32_t state_start_time = get_current_time();
        FOC_StateLoop(&hfoc);
        calculate_execution_time(&hfoc.execution_time.state_max, state_start_time);
        
        
        FOC_USB_Debug_CaptureSamples();
        uint32_t usb_debug_start_time = get_current_time();
        USB_TransmitPacket();
        USB_ReceivePacket();
        hfoc.execution_time.loop_max = 0;
        calculate_execution_time(&hfoc.execution_time.usb_debug_max, usb_debug_start_time);
        
        hfoc.timestamp++;
    }

    calculate_execution_time(&hfoc.execution_time.loop_max, start_time);

    
    if(hfoc.execution_time.loop_max > 1200){ //max 125us for 8kHz loop
        HAL_GPIO_WritePin(DEBUG_LED1_GPIO_Port, DEBUG_LED1_Pin, GPIO_PIN_SET);
        hfoc.execution_time.usb_debug_max = 0;
        USB_printf("Execution Limit Exceeded: %dus\n", (int)hfoc.execution_time.loop_max);
    } else {
        HAL_GPIO_WritePin(DEBUG_LED1_GPIO_Port, DEBUG_LED1_Pin, GPIO_PIN_RESET);
    }
}




void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    UNUSED(huart);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    UNUSED(huart);
    UNUSED(Size);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if(htim->Instance == TIM6){ //not used
    } else if(htim->Instance == TIM7){ //not used
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