#ifndef FOC_FLASH_H
#define FOC_FLASH_H

#include "main.h"
#include "Utils.h"
#include "PID.h"
#include "FOC_Config.h"

typedef enum {
    FLASH_OK = 0,
    FLASH_SAME,
    FLASH_DIFFERENT,
    FLASH_ERROR,
    FLASH_EMPTY
} FLASH_StatusTypeDef;

struct FLASH_MotorParameters {
    uint8_t direction; // boolean, 0 for normal, 1 for reversed

    uint8_t pole_pairs; // [-], 0 if unknown
    float phase_resistance; // [ohms], 0 if unknown
    float phase_inductance; // [H], 0 if unknown
    float torque_constant; // [Nm/A], 0 if unknown
};

struct FLASH_ControllerParameters {
    PIDValuesTypeDef PID_gains_d; // d-axis current PID gains
    PIDValuesTypeDef PID_gains_q; // q-axis current PID gains    

    uint8_t current_PID_FF_enabled; // boolean

    float current_control_bandwidth; // [rad/s], 0 for unset

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
    float ibus_overcurrent_trip_level; // [A]

    float motor_temp_trip_level; // [C], max temperature of the motor
    float mosfet_temp_trip_level; // [C], max temperature of the mosfet

    float max_dq_voltage; // [V], used to limit the output of the PID. Can be at most Vbus/sqrt(3) to avoid overmodulation.
    float max_dq_current; // [A], used to limit the output of the PID.

    
};

struct FLASH_DriverParameters {
    uint8_t node_id; // ID of the driver, 4-bit, 1-15, with 0 reserved for unassigned
    uint16_t heartbeat_msg_rate_ms; // [ms], the rate at which the heartbeat message is sent
};


#define FLASH_DATA_STRUCT_TERMINATOR 0xDEADBEEF // this is used to detect if the struct is correctly read from flash

typedef struct {
    /* Flash settings*/
    uint8_t contains_data; // boolean, first byte of the flash data. This is used to detect whether the flash has data.

    struct FLASH_MotorParameters motor; // motor parameters
    struct FLASH_EncoderParameters encoder; // encoder parameters
    struct FLASH_ControllerParameters controller; // controller parameters
    struct FLASH_Limits limits; // limits
    struct FLASH_DriverParameters node; // driver parameters

    uint8_t filler[4]; // filler to make the struct size a multiple of 8 bytes. This is used to avoid issues with flash programming, which requires 8-byte alignment.

    uint32_t struct_terminator; // Last part of the struct, should be set to FLASH_DATA_STRUCT_TERMINATOR. This is used to detect whether the struct is correctly read from flash.
} FLASH_DataTypeDef;

FLASH_StatusTypeDef FOC_FLASH_WriteData(FLASH_DataTypeDef *pdata);
FLASH_StatusTypeDef FOC_FLASH_ReadData(FLASH_DataTypeDef *pdata);
FLASH_StatusTypeDef FOC_FLASH_CompareData(const FLASH_DataTypeDef *pdata);
FLASH_StatusTypeDef FOC_FLASH_SetDefault(FLASH_DataTypeDef *pdata);














#endif // FOC_FLASH_H