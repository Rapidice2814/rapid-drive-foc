#ifndef FOC_DRIVER_H
#define FOC_DRIVER_H

#include "main.h"
#include "PID.h"
#include "DRV8323_Driver.h"	

typedef struct {
    float a;
    float b;
    float c;
} PhaseCurrents;

typedef struct {
    float alpha;
    float beta;
} AlphaBetaCurrents;

typedef struct {
    float d;
    float q;
} DQCurrents;

typedef struct {
    float a;
    float b;
    float c;
} PhaseVoltages;

typedef struct {
    float alpha;
    float beta;
} AlphaBetaVoltages;

typedef struct {
    float d;
    float q;
} DQVoltages;

typedef struct {
    /* Flags */
    uint8_t adc_calibrated;
    uint8_t encoder_aligned;

    /* More flags */
    uint8_t motor_disable_flag;

    /* General */
    float vin; // input voltage
    float voltage_limit; // max voltage put out by the inverter

    /* ADC buffers */
    PhaseCurrents phase_current;                    //measured phase currents [A]

    /* Encoder */
    volatile uint32_t *pencoder_count;      //pointer to the encoder counter
    float encoder_angle_electric;           //angle in radians
    float previous_encoder_angle_electric;  //previous angle in radians
    float encoder_speed_electric;           //speed in rad/s

    /* Motor parameters */
    float motor_pole_pairs;                 //number of pole pairs
    float motor_stator_resistance;          //stator resistance [ohms]
    float motor_stator_inductance;          //stator inductance [henries]
    float motor_magnet_flux_linkage;        //magnet flux linkage [webers]

    /* Currents and Voltages*/
    AlphaBetaCurrents ab_current;   //alpha and beta currents [A]
    DQCurrents dq_current;          //d and q currents [A]
    DQVoltages dq_voltage;          //d and q voltages [V]
    AlphaBetaVoltages ab_voltage;   //alpha and beta voltages [V]


    /* PID Controllers */
    PIDController pid_current_d;            //d current controller
    PIDController pid_current_q;            //q current controller
    PIDController pid_speed;                //speed controller

    DQCurrents dq_current_setpoint; //d and q current setpoints [A]
    float speed_setpoint; //speed setpoint [rad/s]

    /* PWM */
    uint32_t max_ccr; //max pwm value
    volatile uint32_t *pCCRa;
    volatile uint32_t *pCCRb;
    volatile uint32_t *pCCRc;

    /* DRV8323 */
    DRV8323_HandleTypeDef hdrv8323;

} FOC_HandleTypeDef;

/* General */
void FOC_SetInputVoltage(FOC_HandleTypeDef *hfoc, float vin);
void FOC_SetVoltageLimit(FOC_HandleTypeDef *hfoc, float voltage_limit);


/* Calculations */
AlphaBetaCurrents FOC_Clarke_transform(PhaseCurrents current);
DQCurrents FOC_Park_transform(AlphaBetaCurrents ab_current, float theta);
AlphaBetaVoltages FOC_InvPark_transform(DQVoltages dq_voltage, float theta);
PhaseVoltages FOC_InvClarke_transform(AlphaBetaVoltages ab_voltage);

void FOC_SetPhaseVoltages(FOC_HandleTypeDef *hfoc, PhaseVoltages phase_voltages);

/* Encoder */
void FOC_SetEncoderPointer(FOC_HandleTypeDef *hfoc, volatile uint32_t *encoder_count);
void FOC_SetEncoderZero(FOC_HandleTypeDef *hfoc);
void FOC_UpdateEncoderAngle(FOC_HandleTypeDef *hfoc);
void FOC_UpdateEncoderSpeed(FOC_HandleTypeDef *hfoc, float dt);

/* PWM */
void FOC_SetPWMCCRPointers(FOC_HandleTypeDef *hfoc, volatile uint32_t *pCCRa, volatile uint32_t *pCCRb, volatile uint32_t *pCCRc, uint32_t max_ccr);



#endif // FOC_DRIVER_H