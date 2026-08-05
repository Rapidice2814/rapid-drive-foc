#ifndef FOC_HANDLE_H
#define FOC_HANDLE_H

#include "main.h"

#include "FOC_Flash.h"
#include "FOC_States.h"
#include "PID.h"
#include "DRV8323_Driver.h"	
#include "AS5047P_Driver.h"
#include "FOC_Utils.h"
#include "FOC_ADC.h"
#include "Timing.h"

typedef struct FOC_Handle {
    /* FOC State */
    FOC_StateTypeDef state; // Current state of the FOC driver
    FOC_StateTypeDef previous_state;
    FOC_StateTypeDef next_state;

    /* Flags */
    uint8_t adc_calibrated;

    /* More flags */
    uint8_t motor_disable_flag;

    /* Error registers */
    uint32_t current_errors;
    uint32_t latched_errors;

    /* General */
    FLASH_DataTypeDef flash_data; // flash data structure

    /* ADC values */
    FOC_ADC_ValuesTypeDef adc_values; 
    
    /* Encoder */
    AS5047P_HandleTypeDef has5047p;         //encoder handle
    volatile uint32_t *pencoder_count;      //pointer to the encoder counter

    float encoder_angle_mechanical_wrapped;         //angle in radians
    float encoder_angle_mechanical_unwrapped;       //angle in radians
    float encoder_angle_mechanical_wrapped_prev; //previous angle in radians
    float encoder_speed_mechanical;         //mechanical speed in rad/s
    
    float encoder_angle_electrical;           //angle in radians
    float encoder_speed_electrical;           //electrical speed in rad/s


    /* Currents and Voltages*/
    float ibus; //bus current [A]
    
    ABCurrentsTypeDef ab_current;   //alpha and beta currents [A]
    DQCurrentsTypeDef dq_current;          //d and q currents [A]
    DQVoltagesTypeDef dq_voltage;          //d and q voltages [V]
    ABVoltagesTypeDef ab_voltage;   //alpha and beta voltages [V]

    PhaseVoltagesTypeDef phase_voltage; //phase voltages [V]

    /* PID Controllers */
    PIDControllerTypeDef pid_current_d;            //d current controller
    PIDControllerTypeDef pid_current_q;            //q current controller
    PIDControllerTypeDef pid_speed;                //speed controller
    PIDControllerTypeDef pid_position;             //position controller

    DQCurrentsTypeDef dq_current_setpoint; //d and q current setpoints [A]
    float angle_setpoint; //mechanical position setpoint [rad]
    float speed_setpoint; //mechanical speed setpoint [rad/s]

    /* PWM */
    uint32_t max_ccr; //max pwm value
    volatile uint32_t *pCCRa;
    volatile uint32_t *pCCRb;
    volatile uint32_t *pCCRc;

    /* DRV8323 */
    DRV8323_HandleTypeDef hdrv8323;

    /* CAN */
    FDCAN_HandleTypeDef *phfdcan;

    /* Timing */
    uint32_t timestamp; //timestamp increments every FOC loop, used for debugging and logging
    ExecutionTimeTypeDef execution_time;

} FOC_HandleTypeDef;

#endif /* FOC_HANDLE_H */