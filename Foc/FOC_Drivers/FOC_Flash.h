#ifndef FOC_FLASH_H
#define FOC_FLASH_H

#include "main.h"
#include "Utils.h"
#include "PID.h"
#include "FOC_Config.h"
#include "FOC_Loops.h"
#include "FOC_CAN.h"
#include "FOC_Encoder.h"
#include "FOC_FlashFormat.h"

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

    ControlModeTypeDef control_mode;

    uint8_t anticogging_FF_enabled; //boolean
    uint8_t anticogging_data_valid; //boolean
    float anticogging_array[2][NUMBER_OF_ANTICOG_MEASUREMENTS]; // array to store the anti-cogging measurements

};

struct FLASH_EncoderParameters {
    float mechanical_offset; // [radians]
    uint8_t offset_valid; // boolean
    uint8_t source_id;
    uint8_t correction_source;
    uint8_t correction_valid;
    uint8_t correction_enabled;
    float raw_zero;
    float calibration_current;
    float correction[FOC_ENCODER_LUT_SIZE];
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
    uint32_t can_msg_period_ticks[CAN_CYCLIC_COUNT]; // [ticks], how often the FOC driver sends messages over CAN. 0 disables the message. The order of the messages is defined in FOC_CAN.h
};

#define FLASH_DATA_STRUCT_TERMINATOR 0xDEADBEEF // this is used to detect if the struct is correctly read from flash

typedef struct {
    /* Flash settings*/
    uint32_t magic;
    uint16_t format_version;
    uint16_t stored_size;
    uint8_t contains_data; // Boolean record-valid marker following the format header.

    struct FLASH_MotorParameters motor; // motor parameters
    struct FLASH_EncoderParameters encoder; // encoder parameters
    struct FLASH_ControllerParameters controller; // controller parameters
    struct FLASH_Limits limits; // limits
    struct FLASH_DriverParameters node; // driver parameters
    union { uint8_t reserved[12]; PIDValuesTypeDef anticogging_position_gains; }; // Same version-2 layout.

    uint8_t filler[4]; // Keep the CRC/terminator tail and record 8-byte aligned.

    uint32_t crc;

    uint32_t struct_terminator; // Last part of the struct, should be set to FLASH_DATA_STRUCT_TERMINATOR. This is used to detect whether the struct is correctly read from flash.
} FLASH_DataTypeDef;

static inline void FOC_FLASH_InitAntiCogGains(FLASH_DataTypeDef *data){
    if(data->filler[0] != 0xAC || data->filler[1] != 0x51){
        data->anticogging_position_gains = (PIDValuesTypeDef){10.0f, 20.0f, 0.2f};
        data->filler[0] = 0xAC; data->filler[1] = 0x51;
    }
}


/** @brief Validates a settings record before any of its configuration is applied. */
static inline uint8_t FOC_FLASH_RecordValid(const FLASH_DataTypeDef *data){
    return data != NULL && data->magic == FOC_FLASH_MAGIC &&
        data->format_version == FOC_FLASH_FORMAT_VERSION && data->stored_size == sizeof(*data) &&
        data->contains_data == 1 && data->struct_terminator == FLASH_DATA_STRUCT_TERMINATOR &&
        data->crc == FOC_FlashCRC(data, offsetof(FLASH_DataTypeDef, crc));
}



/* Flash API functions */
FLASH_StatusTypeDef FOC_FLASH_WriteData(FLASH_DataTypeDef *pdata);
FLASH_StatusTypeDef FOC_FLASH_ReadData(FLASH_DataTypeDef *pdata);
FLASH_StatusTypeDef FOC_FLASH_ClearData();
FLASH_StatusTypeDef FOC_FLASH_CompareData(const FLASH_DataTypeDef *pdata);
FLASH_StatusTypeDef FOC_FLASH_SetDefault(FLASH_DataTypeDef *pdata);














#endif // FOC_FLASH_H
