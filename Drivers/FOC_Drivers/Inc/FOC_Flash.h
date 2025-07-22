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
    float vbus_overvoltage_trip_level; // [V]
    float vbus_undervoltage_trip_level; // [V]
    float max_bus_current; // [A]

    float voltage_limit; // [V], the maximum voltage that can be applied to the motor. This should be smaller than vbus

    float max_dq_voltage; // [V], used to limit the output of the PID. should be smaller than voltage_limit/sqrt(3) = 0.577 * voltage_limit
    float max_dq_current; // [A], used to limit the output of the PID.
};

struct FLASH_DriverParameters {
    uint8_t id; // ID of the driver, 4-bit, 1-15, with 0 reserved for unassigned
    uint16_t heartbeat_msg_rate_ms; // [ms], the rate at which the heartbeat message is sent
};



#define FLASH_STRUCT_TERMINATOR 0xDEADBEEF // this is used to detect if the struct is correctly read from flash

typedef struct {
    /* Flash settings*/
    uint8_t contains_data; // boolean, first byte of the flash data. This is used to detect whether the flash has data.

    struct FLASH_MotorParameters motor; // motor parameters
    struct FLASH_EncoderParameters encoder; // encoder parameters
    struct FLASH_ControllerParameters controller; // controller parameters
    struct FLASH_Limits limits; // limits
    struct FLASH_DriverParameters node; // driver parameters

    uint32_t struct_terminator; // this is used to detect if the structure is correctly read from flash. It should be the last member of the structure, and it should be set to FLASH_STRUCT_TERMINATOR when writing to flash
} FLASH_DataTypeDef;

FLASH_StatusTypeDef FOC_FLASH_WriteData(FLASH_DataTypeDef *pdata);
FLASH_StatusTypeDef FOC_FLASH_ReadData(FLASH_DataTypeDef *pdata);














#endif // FOC_FLASH_H