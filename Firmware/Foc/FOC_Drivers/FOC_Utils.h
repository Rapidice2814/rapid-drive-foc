#ifndef FOC_UTILS_H
#define FOC_UTILS_H

#include "main.h"
#include "PID.h"
#include "DRV8323_Driver.h"	
#include "AS5047P_Driver.h"
#include "FOC_Flash.h"
#include "Timing.h"

typedef enum{
	FOC_OK,
	FOC_ERROR
}FOC_StatusTypeDef;

typedef struct {
    float a;
    float b;
    float c;
} PhaseCurrentsTypeDef;

typedef struct {
    float alpha;
    float beta;
} ABCurrentsTypeDef;

typedef struct {
    float d;
    float q;
} DQCurrentsTypeDef;

typedef struct {
    float a;
    float b;
    float c;
} PhaseVoltagesTypeDef;

typedef struct {
    float alpha;
    float beta;
} ABVoltagesTypeDef;

typedef struct {
    float d;
    float q;
} DQVoltagesTypeDef;

typedef struct {
    PhaseCurrentsTypeDef phase_current;            //measured phase currents [A]
    
    float vbus; //bus voltage [V]

    float motor_temp; //motor temperature [C]
} FOC_ADC_ValuesTypeDef;


typedef enum {
    FOC_STATE_INIT,
    FOC_STATE_RESET,
    FOC_STATE_BOOTUP_SOUND,
    FOC_STATE_CURRENT_SENSOR_CALIBRATION,
    FOC_STATE_IDENTIFY,
    FOC_STATE_ANTICOGGING,
    FOC_STATE_CHECKLIST,
    FOC_STATE_PID_AUTOTUNE,
    FOC_STATE_GENERAL_TEST,
    FOC_STATE_ERROR,
    FOC_STATE_CALIBRATION,
    FOC_STATE_ALIGNMENT,
    FOC_STATE_ALIGNMENT_TEST,
    FOC_STATE_RUN,
    FOC_STATE_FLASH_SAVE,
    FOC_STATE_OPENLOOP
} FOC_StateTypeDef;


typedef struct {
    /* FOC State */
    FOC_StateTypeDef state; // Current state of the FOC driver

    /* Flags */
    uint8_t adc_calibrated;
    uint8_t encoder_aligned;

    /* More flags */
    uint8_t motor_disable_flag;

    /* Error registers */
    uint32_t current_errors;
    uint32_t latched_errors;

    /* General */
    float voltage_limit; // max voltage put out by the inverter
    FLASH_DataTypeDef flash_data; // flash data structure

    /* ADC values */
    FOC_ADC_ValuesTypeDef adc_values; 
    
    /* Encoder */
    AS5047P_HandleTypeDef has5047p;         //encoder handle
    volatile uint32_t *pencoder_count;      //pointer to the encoder counter

    float encoder_angle_mechanical;         //angle in radians
    float previous_encoder_angle_mechanical; //previous angle in radians
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
    float speed_setpoint; //mechanical speed setpoint [rad/s]
    float angle_setpoint; //mechanical position setpoint [rad]

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

/* General */
FOC_StatusTypeDef FOC_Init(FOC_HandleTypeDef *hfoc);

/* Calculations */
ABCurrentsTypeDef Clarke_transform(PhaseCurrentsTypeDef current);
DQCurrentsTypeDef Park_transform(ABCurrentsTypeDef ab_current, float theta);
ABVoltagesTypeDef InvPark_transform(DQVoltagesTypeDef dq_voltage, float theta);
PhaseVoltagesTypeDef InvClarke_transform(ABVoltagesTypeDef ab_voltage);
float CalculateBusCurrent(PhaseCurrentsTypeDef phase_current, PhaseVoltagesTypeDef phase_voltage, float vbus);

FOC_StatusTypeDef FOC_SetPhaseVoltages(FOC_HandleTypeDef *hfoc, PhaseVoltagesTypeDef phase_voltage);

/* Encoder */
FOC_StatusTypeDef FOC_SetEncoderPointer(FOC_HandleTypeDef *hfoc, volatile uint32_t *encoder_count);
FOC_StatusTypeDef FOC_SetEncoderZero(FOC_HandleTypeDef *hfoc);
FOC_StatusTypeDef FOC_UpdateEncoderAngle(FOC_HandleTypeDef *hfoc);
FOC_StatusTypeDef FOC_UpdateEncoderSpeed(FOC_HandleTypeDef *hfoc, float frequency);
/* PWM */
FOC_StatusTypeDef FOC_SetPWMCCRPointers(FOC_HandleTypeDef *hfoc, volatile uint32_t *pCCRa, volatile uint32_t *pCCRb, volatile uint32_t *pCCRc, uint32_t max_ccr);

#endif // FOC_UTILS_H