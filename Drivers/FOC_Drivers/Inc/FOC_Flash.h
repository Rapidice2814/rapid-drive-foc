#ifndef FOC_FLASH_H
#define FOC_FLASH_H

#include "stm32g4xx_hal.h"
#include "FOC_Utils.h"
#include "PID.h"
#include "FOC_Config.h"


#define DATA_FLASH_PAGE 56 //the last 8 pages are reserved, pages 56-63
#define NUMBER_OF_FLASH_PAGES 8
#define STORAGE_FLASH_BASE (0x08000000 + FLASH_PAGE_SIZE * DATA_FLASH_PAGE) // 0x0801C000 for page 56, 

typedef enum {
    FLASH_OK = 0,
    FLASH_ERROR = 1
} FLASH_StatusTypeDef;

struct FLASH_MotorParameters {
    uint8_t direction; // boolean, 0 for normal, 1 for reversed

    uint8_t pole_pairs; // [-], number of pole pairs
    uint8_t pole_pairs_valid; // boolean

    float phase_resistance; // [ohms]
    uint8_t phase_resistance_valid; // boolean

    float phase_inductance; // [H]
    uint8_t phase_inductance_valid; // boolean

    float torque_constant; // [Nm/A]
    uint8_t torque_constant_valid; // boolean
};

struct FLASH_ControllerParameters {
    PIDValuesTypeDef PID_gains_d; // d-axis current PID gains
    PIDValuesTypeDef PID_gains_q; // q-axis current PID gains
    uint8_t current_PID_gains_valid; // boolean
    uint8_t current_PID_FF_enabled; // boolean

    float current_control_bandwidth; // [rad/s]
    uint8_t current_control_bandwidth_valid; // boolean


    PIDValuesTypeDef PID_gains_speed; // speed PID gains
    PIDValuesTypeDef PID_gains_position; // position PID gains

    uint8_t speed_PID_enabled; //boolean
    uint8_t position_PID_enabled; //boolean

    uint8_t anticogging_FF_enabled; //boolean
    uint8_t anticogging_data_valid; //boolean
    float anticogging_array[2][NUMBER_OF_ANTICOG_MEASUREMENTS]; // array to store the anti-cogging measurements

};

struct FLASH_EncoderParameters {
    float mechanical_offset; // [radians]
    uint8_t offset_valid; // boolean
};

struct FLASH_Limits {
    float vbus_overvoltage; // [V]
    float vbus_undervoltage; // [V]
    float max_phase_current; // [A]
};





typedef struct {
    /* Flash settings*/
    uint8_t contains_data_flag; // boolean, first byte of the flash data. 1 if the data is present, 0 if not

    struct FLASH_MotorParameters motor; // motor parameters

    struct FLASH_EncoderParameters encoder; // encoder parameters

    struct FLASH_ControllerParameters controller; // controller parameters

    uint8_t data_valid_flag; // boolean, last byte of the flash data. 1 if the data is valid, 0 if not. This is there to detect if the data is correctly read from flash

} FLASH_DataTypeDef;

FLASH_StatusTypeDef FOC_FLASH_WriteData(FLASH_DataTypeDef *pdata);
FLASH_StatusTypeDef FOC_FLASH_ReadData(FLASH_DataTypeDef *pdata);














#endif // FOC_FLASH_H