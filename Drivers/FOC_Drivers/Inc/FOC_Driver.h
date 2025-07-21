#ifndef FOC_DRIVER_H
#define FOC_DRIVER_H

#include "main.h"
#include "PID.h"
#include "DRV8323_Driver.h"	
#include "AS5047P_Driver.h"
#include "FOC_Flash.h"

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
    /* Flags */
    uint8_t adc_calibrated;
    uint8_t encoder_aligned;

    /* More flags */
    uint8_t motor_disable_flag;

    /* General */
    float voltage_limit; // max voltage put out by the inverter
    FLASH_DataTypeDef flash_data; // flash data structure

    /* ADC buffers */
    PhaseCurrentsTypeDef phase_current;            //measured phase currents [A]
    PhaseCurrentsTypeDef phase_current_offset;     //offset for the phase currents [A]
    float vbus; //bus voltage [V]
    float ibus; //bus current [A]

    /* Encoder */
    AS5047P_HandleTypeDef has5047p;         //encoder handle
    volatile uint32_t *pencoder_count;      //pointer to the encoder counter

    float encoder_angle_mechanical;         //angle in radians
    float previous_encoder_angle_mechanical; //previous angle in radians
    float encoder_speed_mechanical;         //mechanical speed in rad/s
    
    float encoder_angle_electrical;           //angle in radians
    float previous_encoder_angle_electrical;  //previous angle in radians
    float encoder_speed_electrical;           //electrical speed in rad/s


    /* Currents and Voltages*/
    ABCurrentsTypeDef ab_current;   //alpha and beta currents [A]
    DQCurrentsTypeDef dq_current;          //d and q currents [A]
    DQVoltagesTypeDef dq_voltage;          //d and q voltages [V]
    ABVoltagesTypeDef ab_voltage;   //alpha and beta voltages [V]

    PhaseVoltagesTypeDef phase_voltage; //phase voltages [V]

    /* Temperature */
    float NTC_resistance; //resistance [ohms]
    float NTC_temp; //temperature [C]


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

} FOC_HandleTypeDef;

/* General */
FOC_StatusTypeDef FOC_Init(FOC_HandleTypeDef *hfoc);


/* Calculations */
ABCurrentsTypeDef FOC_Clarke_transform(PhaseCurrentsTypeDef current);
DQCurrentsTypeDef FOC_Park_transform(ABCurrentsTypeDef ab_current, float theta);
ABVoltagesTypeDef FOC_InvPark_transform(DQVoltagesTypeDef dq_voltage, float theta);
PhaseVoltagesTypeDef FOC_InvClarke_transform(ABVoltagesTypeDef ab_voltage);

FOC_StatusTypeDef FOC_SetPhaseVoltages(FOC_HandleTypeDef *hfoc, PhaseVoltagesTypeDef phase_voltages);

/* Encoder */
FOC_StatusTypeDef FOC_SetEncoderPointer(FOC_HandleTypeDef *hfoc, volatile uint32_t *encoder_count);
FOC_StatusTypeDef FOC_SetEncoderZero(FOC_HandleTypeDef *hfoc);
FOC_StatusTypeDef FOC_UpdateEncoderAngle(FOC_HandleTypeDef *hfoc);
FOC_StatusTypeDef FOC_UpdateEncoderSpeed(FOC_HandleTypeDef *hfoc, float dt, float filter_alpha);
/* PWM */
FOC_StatusTypeDef FOC_SetPWMCCRPointers(FOC_HandleTypeDef *hfoc, volatile uint32_t *pCCRa, volatile uint32_t *pCCRb, volatile uint32_t *pCCRc, uint32_t max_ccr);
FOC_StatusTypeDef FOC_CalculateBusCurrent(FOC_HandleTypeDef *hfoc);

#endif // FOC_DRIVER_H