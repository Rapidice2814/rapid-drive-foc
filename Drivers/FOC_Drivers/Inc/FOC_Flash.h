#ifndef FOC_FLASH_H
#define FOC_FLASH_H

#include "stm32g4xx_hal.h"
#include "FOC_Utils.h"
#include "PID.h"

typedef enum {
    FLASH_OK = 0,
    FLASH_ERROR = 1
} FLASH_StatusTypeDef;


typedef struct {
    /* Flash settings*/
    uint8_t contains_data;

    /* Motor settings */
    uint8_t motor_direction_swapped;

    /* Motor parameters */
    float motor_pole_pairs;                 //number of pole pairs
    float motor_stator_resistance;          //stator resistance [ohms]
    float motor_stator_inductance;          //stator inductance [henries]
    float motor_magnet_flux_linkage;        //magnet flux linkage [webers]

    /* Encoder */
    uint8_t encoder_aligned_flag;  //flag for the encoder alignment
    float encoder_angle_mechanical_offset;  //offset for the encoder angle [radians]

    /* PID controler gains*/
    PIDValuesTypeDef PID_gains_q;
    PIDValuesTypeDef PID_gains_d;


} FLASH_DataTypeDef;

FLASH_StatusTypeDef FOC_FLASH_WriteData(FLASH_DataTypeDef *pdata);
FLASH_StatusTypeDef FOC_FLASH_ReadData(FLASH_DataTypeDef *pdata);














#endif // FOC_FLASH_H