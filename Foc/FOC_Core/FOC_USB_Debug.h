#ifndef FOC_USB_DEBUG_H
#define FOC_USB_DEBUG_H

#include "stdint.h"

typedef enum{
	DEBUG_OK,
	DEBUG_STOPPED,
	DEBUG_BUSY,
	DEBUG_ERROR
}Debug_StatusTypeDef;

/**** USB Packet Structure ****/
/*
Binary Packets:
Binary packets are detected based on the SOF bytes at the beginning of the payload. In case the SOF bytes are not found, the packet is decoded as a text packet, based on ASCII values.
All values are represented either as a 4-byte float or a 4-byte integer, signed or unsigned depending on the signal type. 
The PC should interpret the values based on the signal definitions in FOC_USB_DEBUG_SIGNAL_LIST and the corresponding types (f for float, i32 for signed int, u32 for unsigned int).
All the USB packets follow the same structure: 
    SOF (2 bytes) | Msg Type (1 byte) | Payload Length (2 bytes) | Payload (N bytes)
Depending on the Msg Type, the payload can have different formats:
MSG_GET_VERSION: PC -> FOC
    Payload: None
MSG_VERSION_REPLY: FOC -> PC
    Payload: Major Version (1 byte) | Minor Version (1 byte) | Patch Version (1 byte)
    The 
MSG_LOG_DATA: FOC -> PC
    Payload: Timestamp (4 bytes) | Sample Count (2 bytes) | Signal Count (2 bytes) | Data Buffer (Sample Count * Signal Count * 4 bytes)
    The data buffer contains the captured signal values in the order defined by the signal mask. Each value is a 4-byte float or integer depending on the signal type.
MSG_SET_MASK: PC -> FOC
    Payload: Signal Mask (SIGNAL_MASK_BYTES bytes)
    The signal mask is a 32-bit value where each bit corresponds to a specific signal. 
    FOC_USB_DEBUG_SIGNAL_LIST defines the mapping (and type) of bits to signals in the FOC_HandleTypeDef structure.
    At most MAX_LOGDATA_SIGNAL_COUNT bits can be set in the mask, which determines how many signals will be captured and included in the log data packets.
    Mask can only be updated when logging is stopped.
MSG_GET_MASK: PC -> FOC
    Payload: None
    Requests the current signal mask from the FOC firmware.
MSG_START_LOG: PC -> FOC
    Payload: None
    Enables the logging of data based on the current signal mask. The FOC_USB_Debug_CaptureSamples function will start capturing samples and filling the log data payload.
MSG_STOP_LOG: PC -> FOC
    Payload: None
    Disables the logging of data. The FOC_USB_Debug_CaptureSamples function will stop capturing samples and filling the log data payload.
MSG_SET_PID: PC -> FOC
    Payload: Controller ID (1 byte) | PID Gains (12 bytes: 4 bytes for Kp, 4 bytes for Ki, 4 bytes for Kd)
    Sets the PID gains for a specific controller (e.g., position, speed, current).
    FOC_PID_CONTROLLERS_LIST defines the mapping of controller IDs to the actual PID controllers in the FOC_HandleTypeDef structure.
MSG_GET_PID: PC -> FOC
    Payload: Controller ID (1 byte)
    Requests the current PID gains for a specific controller.
MSG_PID_REPLY: FOC -> PC
    Payload: Controller ID (1 byte) | PID Gains (12 bytes: 4 bytes for Kp, 4 bytes for Ki, 4 bytes for Kd)
    Reply to a MSG_GET_PID request, containing the current PID gains for the requested controller.
MSG_SET_VAR: PC -> FOC
    Payload: Variable ID (1 byte) | Variable Value (4 bytes)
    VAR_ID_LIST defines the mapping of variable IDs to specific variables in the FOC_HandleTypeDef structure that can be set or read.
MSG_GET_VAR: PC -> FOC
    Payload: Variable ID (1 byte)
    Requests the current value of a specific variable defined in VAR_ID_LIST.
MSG_VAR_REPLY: FOC -> PC
    Payload: Variable ID (1 byte) | Variable Value (4 bytes)
    Reply to a MSG_GET_VAR request, containing the current value of the requested variable.
MSG_FLASH_SAVE: PC -> FOC
    Payload: None
    Instructs the FOC firmware to save the current configuration (e.g., PID gains, settings) to flash memory.
    Can only be executed, when FOC is in IDLE mode.
MSG_FLASH_LOAD: PC -> FOC
    Payload: None
    Instructs the FOC firmware to load the configuration from flash memory.
    Can only be executed, when FOC is in IDLE mode.
MSG_FLASH_CLEAR: PC -> FOC
    Payload: None
    Instructs the FOC firmware to clear the configuration in flash memory.
    Can only be executed, when FOC is in IDLE mode.
MSG_SET_STATE: PC -> FOC
    Payload: Desired State (1 byte)(FOC_StateTypeDef)
    Sets the desired state of the FOC driver to enum FOC_StateTypeDef.
MSG_GET_STATE: PC -> FOC
    Payload: None
    Requests the current state of the FOC driver.
MSG_STATE_REPLY: FOC -> PC
    Payload: Current State (1 byte)
    Reply to a MSG_GET_STATE request, containing the current state of the FOC driver.
MSG_SET_NODE_ID: PC -> FOC
    Payload: Node ID (1 byte)
    Sets the node ID of the FOC driver for CAN communication. Only values 1-15 are valid, with 0 reserved for unassigned.
MSG_GET_NODE_ID: PC -> FOC
    Payload: None
    Requests the current node ID of the FOC driver.
MSG_NODE_ID_REPLY: FOC -> PC
    Payload: Node ID (1 byte)
    Reply to a MSG_GET_NODE_ID request, containing the current node ID of the FOC driver.
MSG_GET_ACTIVE_ERRORS: PC -> FOC
    Payload: None
    Requests the current active errors of the FOC driver.
MSG_ACTIVE_ERRORS_REPLY: FOC -> PC
    Payload: Active Errors (4 bytes)
    Reply to a MSG_GET_ACTIVE_ERRORS request, containing the current active errors of the FOC driver.
MSG_GET_LATCHED_ERRORS: PC -> FOC
    Payload: None
    Requests the current latched errors of the FOC driver.
MSG_LATCHED_ERRORS_REPLY: FOC -> PC
    Payload: Latched Errors (4 bytes)
    Reply to a MSG_GET_LATCHED_ERRORS request, containing the current latched errors of the FOC driver.
MSG_CLEAR_LATCHED_ERRORS: PC -> FOC
    Payload: None
    Instructs the FOC firmware to clear the latched errors.
MSG_SET_CAN_CYCLIC_RATE: PC -> FOC
    Payload: CAN_CyclicTypeDef (1 byte) | Cyclic Rate (4 bytes)
    Sets the rate at which the FOC firmware sends cyclic messages over CAN. A value of 0 disables the cyclic messages.
MSG_GET_CAN_CYCLIC_RATE: PC -> FOC
    Payload: CAN_CyclicTypeDef (1 byte)
    Requests the current cyclic rate for CAN messages from the FOC firmware.
MSG_CAN_CYCLIC_REPLY: FOC -> PC
    Payload: CAN_CyclicTypeDef (1 byte) | Cyclic Rate (4 bytes)
    Reply to a MSG_GET_CAN_CYCLIC_RATE request, containing the current cyclic rate for CAN messages from the FOC firmware.
MSG_SET_CONTROL_MODE: PC -> FOC
    Payload: Control Mode (1 byte)(ControlModeTypeDef)
    Sets the control mode of the FOC driver to enum ControlModeTypeDef.
MSG_GET_CONTROL_MODE: PC -> FOC
    Payload: None
    Requests the current control mode of the FOC driver.
MSG_CONTROL_MODE_REPLY: FOC -> PC
    Payload: Control Mode (1 byte)
    Reply to a MSG_GET_CONTROL_MODE request, containing the current control mode of the FOC driver.


MSG_UNKNOWN_TYPE: FOC -> PC
    Payload: None
    Sent by the FOC firmware when it receives a message with an unrecognized Msg Type. Can be used for debugging and error handling on the PC side.
MSG_INVALID_PAYLOAD: FOC -> PC
    Payload: None
    Sent by the FOC firmware when it receives a message with a recognized Msg Type but the payload is invalid (e.g., wrong length, invalid values). Can be used for debugging and error handling on the PC side.
MSG_ACK: FOC -> PC
    Payload: None
    Sent by the FOC firmware to acknowledge the successful receipt and processing of a command from the PC (e.g., MSG_SET_MASK, MSG_START_LOG).
MSG_ERROR: FOC -> PC
    Payload: None
    Sent by the FOC firmware to indicate an error in processing a command from the PC (e.g., command not allowed in current state).

*/

typedef enum {
    MSG_GET_VERSION = 0x00, //PC -> FOC
    MSG_VERSION_REPLY = 0x01, //FOC -> PC
    MSG_ENTER_BOOTLOADER = 0x02, // PC -> FOC
    MSG_LOG_DATA = 0x03, //FOC -> PC
    MSG_SET_MASK = 0x04, //PC -> FOC
    MSG_GET_MASK = 0x05, // PC -> FOC
    MSG_MASK_REPLY = 0x06, // FOC -> PC
    MSG_START_LOG = 0x07, //PC -> FOC
    MSG_STOP_LOG = 0x08, //PC -> FOC
    MSG_SET_PID = 0x09, //PC -> FOC
    MSG_GET_PID = 0x0A, //PC -> FOC
    MSG_PID_REPLY = 0x0B, //FOC -> PC
    MSG_SET_VAR = 0x0C, //PC -> FOC
    MSG_GET_VAR = 0x0D, //PC -> FOC
    MSG_VAR_REPLY = 0x0E, //FOC -> PC
    MSG_FLASH_SAVE = 0x0F, //PC -> FOC
    MSG_FLASH_LOAD = 0x10, //PC -> FOC
    MSG_FLASH_CLEAR = 0x11, // PC -> FOC
    MSG_SET_STATE = 0x12, //PC -> FOC
    MSG_GET_STATE = 0x13, //PC -> FOC
    MSG_STATE_REPLY = 0x14, //FOC -> PC
    MSG_TEXT_COMMAND = 0x15, // PC -> FOC
    MSG_TEXT_REPLY = 0x16, // FOC -> PC
    MSG_SET_NODE_ID = 0x17, // PC -> FOC
    MSG_GET_NODE_ID = 0x18, // PC -> FOC
    MSG_NODE_ID_REPLY = 0x19, // FOC -> PC
    MSG_GET_ACTIVE_ERRORS = 0x1A, // PC -> FOC
    MSG_ACTIVE_ERRORS_REPLY = 0x1B, // FOC -> PC
    MSG_GET_LATCHED_ERRORS = 0x1C, // PC -> FOC
    MSG_LATCHED_ERRORS_REPLY = 0x1D, // FOC -> PC
    MSG_CLEAR_LATCHED_ERRORS = 0x1E, // PC -> FOC
    MSG_SET_CAN_CYCLIC_RATE = 0x1F, // PC -> FOC
    MSG_GET_CAN_CYCLIC_RATE = 0x20, // PC -> FOC
    MSG_CAN_CYCLIC_REPLY = 0x21, // FOC -> PC
    MSG_SET_CONTROL_MODE = 0x22, // PC -> FOC
    MSG_GET_CONTROL_MODE = 0x23, // PC -> FOC
    MSG_CONTROL_MODE_REPLY = 0x24, // FOC -> PC

    MSG_UNKNOWN_TYPE = 0xFA, //FOC -> PC
    MSG_INVALID_PAYLOAD = 0xFB, //FOC -> PC
    MSG_UNKNOWN_ID = 0xFC, //FOC -> PC
    MSG_BUFFER_OVERFLOW = 0xFD, //FOC -> PC
    MSG_ACK = 0xFE, //FOC -> PC
    MSG_ERROR = 0xFF //FOC -> PC
} MsgTypeTypeDef;


#define SIGNAL_MASK_BYTES 8 // max 8*8 = 64 signals
#define FOC_USB_DEBUG_SIGNAL_LIST(X)            \
    X(0, u32,   tick)                      		\
    X(1, f,     adc_values.motor_temp)          \
    X(2, f,     adc_values.mosfet_temp)         \
    X(3, f,     adc_values.vbus)                \
    X(4, f,     ibus)                           \
    X(5, f,     adc_values.phase_current.a)     \
    X(6, f,     adc_values.phase_current.b)     \
    X(7, f,     adc_values.phase_current.c)     \
    X(8, f,     ab_current.alpha)               \
    X(9, f,     ab_current.beta)                \
    X(10, f,    dq_current.d)                   \
    X(11, f,    dq_current.q)                   \
    X(12, f,    dq_current_filtered.d)          \
    X(13, f,    dq_current_filtered.q)          \
    X(14, f,    phase_voltage.a)                \
    X(15, f,    phase_voltage.b)                \
    X(16, f,    phase_voltage.c)                \
    X(17, f,    ab_voltage.alpha)               \
    X(18, f,    ab_voltage.beta)                \
    X(19, f,    dq_voltage.d)                   \
    X(20, f,    dq_voltage.q)                   \
    X(21, f,    encoder_angle_mechanical_wrapped) \
    X(22, f,    encoder_angle_mechanical_unwrapped) \
    X(23, f,    encoder_speed_mechanical)       \
    X(24, f,    encoder_angle_electrical)       \
    X(25, f,    encoder_speed_electrical)       \
    X(26, f,    dq_current_setpoint.d)          \
    X(27, f,    dq_current_setpoint.q)          \
    X(28, f,    angle_setpoint)                 \
    X(29, f,    speed_setpoint)                 \
    X(30, u32,  execution_time.loop_max)        \
    X(31, f,  hfi.injection_phase)              \
    X(32, f,  hfi.i_alpha_l_raw)                \
    X(33, f,  hfi.i_beta_l_raw)                 \
    X(34, f,  hfi.i_alpha_l_filtered)           \
    X(35, f,  hfi.i_beta_l_filtered)            \

#define FOC_PID_CONTROLLERS_LIST(X)             \
    X(0, pid_current_d)                         \
    X(1, pid_current_q)                         \
    X(2, pid_speed)                             \
    X(3, pid_position)                          \

#define VAR_ID_LIST(X)                         \
    X(0, f, dq_current_setpoint.d)             \
    X(1, f, dq_current_setpoint.q)             \
    X(2, f, angle_setpoint)                    \
    X(3, f, speed_setpoint)                    \
    X(4, f, flash_data.limits.max_dq_current)  \
    X(5, f, flash_data.limits.max_dq_voltage)  \
    X(6, f, flash_data.limits.vbus_overvoltage_trip_level) \
    X(7, f, flash_data.limits.vbus_undervoltage_trip_level) \
    X(8, f, flash_data.limits.ibus_overcurrent_trip_level) \
    X(9, f, flash_data.limits.motor_temp_trip_level) \
    X(10, f, flash_data.limits.mosfet_temp_trip_level) \
    X(11, u32, flash_data.motor.pole_pairs) \
    X(12, f, flash_data.motor.phase_resistance) \
    X(13, f, flash_data.motor.phase_inductance) \
    X(14, f, flash_data.motor.torque_constant) \
    X(15, u32, flash_data.hfi.hfi_enabled) \
    X(16, f, flash_data.hfi.injection_amplitude) \
    X(17, f, flash_data.hfi.injection_omega) \
    X(18, f, flash_data.controller.current_control_bandwidth) \

Debug_StatusTypeDef FOC_USB_Setup();
Debug_StatusTypeDef FOC_USB_Debug_CaptureSamples();
uint8_t Debug_SendTextResponse(const char* format, ...) __attribute__((format(printf, 1, 2)));


#endif // FOC_USB_DEBUG_H