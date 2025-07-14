#ifndef FOC_FLASH_H
#define FOC_FLASH_H

#include "stm32g4xx_hal.h"
#include "FOC_Utils.h"
#include "PID.h"
#include "FOC_Config.h"

typedef enum {
    FLASH_OK = 0,
    FLASH_ERROR = 1
} FLASH_StatusTypeDef;


typedef struct {
    /* Flash settings*/
    uint8_t contains_data_flag;

    /* Motor settings */
    uint8_t motor_direction_swapped_flag; // 1 if the motor direction is swapped

    /* Motor parameters */
    uint8_t motor_identified_flag; // 1 if the motor parameters are valid
    float motor_pole_pairs;                 //number of pole pairs
    float motor_stator_resistance;          //stator resistance [ohms]
    float motor_stator_inductance;          //stator inductance [henries]
    float motor_torque_constant;            //motor torque constant [Nm/A]

    /* Encoder */
    uint8_t encoder_aligned_flag;  //flag for the encoder alignment
    float encoder_angle_mechanical_offset;  //offset for the encoder angle [radians]

    /* Current PID controler*/
    uint8_t current_PID_set_flag; // 1 if the current PID is set
    uint8_t current_PID_FF_enabled_flag; // 1 if the current PID feedforward is enabled
    float current_control_bandwidth; //current control bandwidth [rad/s]
    PIDValuesTypeDef PID_gains_q;
    PIDValuesTypeDef PID_gains_d;

    /* Speed PID controler */
    PIDValuesTypeDef PID_gains_speed;
    uint8_t speed_PID_enabled_flag;

    /* Position PID controler */
    PIDValuesTypeDef PID_gains_position;
    uint8_t position_PID_enabled_flag;

    /* Anti-cogging */
    uint8_t anticogging_enabled_flag; // 1 if the anti-cogging is enabled in the loop
    uint8_t anticogging_data_valid_flag; // 1 if the anti-cogging data is valid
    float anticogging_measurements[2][NUMBER_OF_ANTICOG_MEASUREMENTS]; // array to store the anti-cogging measurements

    uint8_t data_valid_flag; // 1 if the data is valid, 0 if not

} FLASH_DataTypeDef;

FLASH_StatusTypeDef FOC_FLASH_WriteData(FLASH_DataTypeDef *pdata);
FLASH_StatusTypeDef FOC_FLASH_ReadData(FLASH_DataTypeDef *pdata);














#endif // FOC_FLASH_H