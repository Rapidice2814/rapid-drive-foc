#ifndef FOC_CAN_H
#define FOC_CAN_H

#include "main.h"
#include <stdint.h>

/**** CAN Packet Structure ****/
/*
CAN ID Structure: 4-bit Node ID | 7-bit Command
Supported Commands:
CAN_ESTOP: PC -> FOC
    Payload: None
    Instructs the FOC firmware to enter a stop state.
CAN_GET_VERSION: PC -> FOC
    Payload: None
    Requests the current firmware version from the FOC firmware.
CAN_VERSION_REPLY: FOC -> PC
    Payload: Major Version (1 byte), Minor Version (1 byte), Patch Version (1 byte)
    Reply to a CAN_GET_VERSION request, containing the current firmware version of the FOC firmware.
CAN_REBOOT: PC -> FOC
    Payload: None
    Instructs the FOC firmware to reboot the microcontroller.
CAN_ENTER_BOOTLOADER: PC -> FOC
    Payload: None
    Instructs the FOC firmware to enter the bootloader mode for firmware updates.
CAN_SET_ADDRESS: PC -> FOC
    UNIMPLEMENTED
CAN_GET_ADDRESS: PC -> FOC
    UNIMPLEMENTED
CAN_ADDRESS_REPLY: FOC -> PC
    UNIMPLEMENTED
CAN_SET_STATE: PC -> FOC
    Payload: Desired State (1 byte)(FOC_StateTypeDef)
    Sets the desired state of the FOC driver to enum FOC_StateTypeDef.
CAN_GET_STATE: PC -> FOC
    Payload: None
    Requests the current state of the FOC driver.
CAN_STATE_REPLY: FOC -> PC
    Payload: Current State (1 byte)(FOC_StateTypeDef)
    Reply to a CAN_GET_STATE request, containing the current state of the FOC driver.
CAN_SET_CONTROL_MODE: PC -> FOC
    Payload: Control Mode (1 byte)(ControlModeTypeDef)
    Sets the control mode of the FOC driver to enum ControlModeTypeDef.
CAN_GET_CONTROL_MODE: PC -> FOC
    Payload: None
    Requests the current control mode of the FOC driver.
CAN_CONTROL_MODE_REPLY: FOC -> PC
    Payload: Current Control Mode (1 byte)(ControlModeTypeDef)
    Reply to a CAN_GET_CONTROL_MODE request, containing the current control mode of the FOC driver.
CAN_GET_HEARTBEAT: PC -> FOC
    Payload: None
    Requests a heartbeat message from the FOC firmware.
CAN_HEARTBEAT_REPLY: FOC -> PC
    Payload: None
    Reply to a CAN_GET_HEARTBEAT request, indicating that the FOC firmware is alive and responsive.
    Can also be configured to send cyclically at a specified rate.
CAN_GET_ENCODER_ESTIMATES: PC -> FOC
    Payload: None
    Requests the current encoder estimates (angle and speed) from the FOC firmware.
CAN_ENCODER_ESTIMATES_REPLY: FOC -> PC
    Payload: Encoder Angle (4 bytes, float), Encoder Speed (4 bytes, float)
    Reply to a CAN_GET_ENCODER_ESTIMATES request, containing the current encoder angle and speed estimates from the FOC firmware.
    Can also be configured to send cyclically at a specified rate.
CAN_GET_BUS_VOLTGE_CURRENT: PC -> FOC
    Payload: None
    Requests the current bus voltage and current measurements from the FOC firmware.
CAN_BUS_VOLTAGE_CURRENT_REPLY: FOC -> PC
    Payload: Bus Voltage (4 bytes, float), Bus Current (4 bytes, float)
    Reply to a CAN_GET_BUS_VOLTGE_CURRENT request, containing the current bus voltage and current measurements from the FOC firmware.
    Can also be configured to send cyclically at a specified rate.
CAN_GET_TEMPERATURES: PC -> FOC
    Payload: None
    Requests the current temperature measurements from the FOC firmware.
CAN_TEMPERATURES_REPLY: FOC -> PC
    Payload: Mosfet Temperature (4 bytes, float), Motor Temperature (4 bytes, float)
    Reply to a CAN_GET_TEMPERATURES request, containing the current temperature measurements from the FOC firmware.
    Can also be configured to send cyclically at a specified rate.
CAN_SET_TORQUE: PC -> FOC
    Payload: Desired Torque (4 bytes, float)
    Sets the desired torque for the FOC driver.
CAN_GET_TORQUE: PC -> FOC
    Payload: None
    Requests the current torque setpoint and actual torque from the FOC firmware.
CAN_TORQUE_REPLY: FOC -> PC
    Payload: Torque Setpoint (4 bytes, float), Actual Torque (4 bytes, float)
    Reply to a CAN_GET_TORQUE request, containing the current torque setpoint and actual torque from the FOC firmware.
    Can also be configured to send cyclically at a specified rate.
CAN_SET_CURRENT: PC -> FOC
    Payload: Desired Q-axis Current (4 bytes, float)
    Sets the desired Q-axis current for the FOC driver.
CAN_GET_CURRENT: PC -> FOC
    Payload: None
    Requests the current Q-axis current setpoint and actual Q-axis current from the FOC firmware.
CAN_CURRENT_REPLY: FOC -> PC
    Payload: Q-axis Current Setpoint (4 bytes, float), Actual Q-axis Current (4 bytes, float)
    Reply to a CAN_GET_CURRENT request, containing the current Q-axis current setpoint and actual Q-axis current from the FOC firmware.
    Can also be configured to send cyclically at a specified rate.
CAN_SET_SPEED: PC -> FOC
    Payload: Desired Mechanical Speed (4 bytes, float)
    Sets the desired mechanical speed for the FOC driver.
CAN_GET_SPEED: PC -> FOC
    Payload: None
    Requests the current mechanical speed setpoint and actual mechanical speed from the FOC firmware.
CAN_SPEED_REPLY: FOC -> PC
    Payload: Speed Setpoint (4 bytes, float), Actual Speed (4 bytes, float)
    Reply to a CAN_GET_SPEED request, containing the current mechanical speed setpoint and actual mechanical speed from the FOC firmware.
    Can also be configured to send cyclically at a specified rate.
CAN_SET_POSITION: PC -> FOC
    Payload: Desired Mechanical Position (4 bytes, float)
    Sets the desired mechanical position for the FOC driver.
CAN_GET_POSITION: PC -> FOC
    Payload: None
    Requests the current mechanical position setpoint and actual mechanical position from the FOC firmware.
CAN_POSITION_REPLY: FOC -> PC
    Payload: Position Setpoint (4 bytes, float), Actual Position (4 bytes, float)
    Reply to a CAN_GET_POSITION request, containing the current mechanical position setpoint and actual mechanical position from the FOC firmware.
    Can also be configured to send cyclically at a specified rate.
CAN_GET_ERRORS: PC -> FOC
    Payload: None
    Requests the current active and latched errors from the FOC firmware.
CAN_ERRORS_REPLY: FOC -> PC
    Payload: Active Errors (4 bytes, uint32_t), Latched Errors (4 bytes, uint32_t)
    Reply to a CAN_GET_ERRORS request, containing the current active and latched errors from the FOC firmware.
    Can also be configured to send cyclically at a specified rate.
CAN_CLEAR_LATCHED_ERRORS: PC -> FOC
    Payload: None
    Instructs the FOC firmware to clear the latched errors.

CAN_ACK: FOC -> PC
    Payload: None
    Sent by the FOC firmware to acknowledge the successful receipt and processing of a command from the PC.
CAN_ERROR: FOC -> PC
    Payload: None
    Sent by the FOC firmware to indicate an error in processing a command from the PC (e.g., command not allowed in current state).

*/

#define CAN_BROADCAST_NODE_ID 0x00

#define ID_MASK 0x0F             // 4-bit ID mask
#define COMMAND_MASK (0x7F << 4) // 7-bit command mask

#define GET_CAN_ID(node_id, command) ((node_id & ID_MASK) | ((command << 4) & COMMAND_MASK)) // Constructs the CAN identifier, 4-bit ID, 7-bit command
#define GET_ID_FROM_CAN_ID(can_id) (can_id & ID_MASK)                                        // Extracts the ID from the CAN identifier
#define GET_COMMAND_FROM_CAN_ID(can_id) ((can_id & COMMAND_MASK) >> 4)                       // Extracts the command from the CAN identifier

typedef enum {
    CAN_ESTOP = 0x00,         // PC -> FOC
    CAN_GET_VERSION = 0x01,   // PC -> FOC
    CAN_VERSION_REPLY = 0x02, // FOC -> PC

    CAN_REBOOT = 0x03,           // PC -> FOC
    CAN_ENTER_BOOTLOADER = 0x04, // PC -> FOC

    CAN_SET_ADDRESS = 0x05,   // PC -> FOC
    CAN_GET_ADDRESS = 0x06,   // PC -> FOC
    CAN_ADDRESS_REPLY = 0x07, // FOC -> PC

    CAN_SET_STATE = 0x08,   // PC -> FOC
    CAN_GET_STATE = 0x09,   // PC -> FOC
    CAN_STATE_REPLY = 0x0A, // FOC -> PC

    CAN_SET_CONTROL_MODE = 0x0B,   // PC -> FOC
    CAN_GET_CONTROL_MODE = 0x0C,   // PC -> FOC
    CAN_CONTROL_MODE_REPLY = 0x0D, // FOC -> PC

    /* CYCLIC */
    CAN_GET_HEARTBEAT = 0x0E,   // PC -> FOC
    CAN_HEARTBEAT_REPLY = 0x0F, // FOC -> PC

    CAN_GET_ENCODER_ESTIMATES = 0x10,   // PC -> FOC
    CAN_ENCODER_ESTIMATES_REPLY = 0x11, // FOC -> PC

    CAN_GET_BUS_VOLTGE_CURRENT = 0x12,    // PC -> FOC
    CAN_BUS_VOLTAGE_CURRENT_REPLY = 0x13, // FOC -> PC

    CAN_GET_TEMPERATURES = 0x14,   // PC -> FOC
    CAN_TEMPERATURES_REPLY = 0x15, // FOC -> PC

    CAN_SET_TORQUE = 0x16,   // PC -> FOC
    CAN_GET_TORQUE = 0x17,   // PC -> FOC
    CAN_TORQUE_REPLY = 0x18, // FOC -> PC

    CAN_SET_CURRENT = 0x19,   // PC -> FOC
    CAN_GET_CURRENT = 0x1A,   // PC -> FOC
    CAN_CURRENT_REPLY = 0x1B, // FOC -> PC

    CAN_SET_SPEED = 0x1C,   // PC -> FOC
    CAN_GET_SPEED = 0x1D,   // PC -> FOC
    CAN_SPEED_REPLY = 0x1E, // FOC -> PC

    CAN_SET_POSITION = 0x1F,   // PC -> FOC
    CAN_GET_POSITION = 0x20,   // PC -> FOC
    CAN_POSITION_REPLY = 0x21, // FOC -> PC

    CAN_GET_ERRORS = 0x22,              // PC -> FOC
    CAN_ERRORS_REPLY = 0x23,            // FOC -> PC
    CAN_CLEAR_LATCHED_ERRORS = 0x24,    // PC -> FOC

    CAN_ACK = 0x3E,  // FOC -> PC
    CAN_ERROR = 0x3F // FOC -> PC
} CAN_CommandTypeDef;

typedef enum {
    CAN_CYCLIC_HEARTBEAT = 0,
    CAN_CYCLIC_ENCODER_ESTIMATES = 0x01,
    CAN_CYCLIC_BUS_VOLTAGE_CURRENT = 0x02,
    CAN_CYCLIC_TEMPERATURES = 0x03,
    CAN_CYCLIC_TORQUE = 0x04,
    CAN_CYCLIC_CURRENT = 0x05,
    CAN_CYCLIC_SPEED = 0x06,
    CAN_CYCLIC_POSITION = 0x07,
    CAN_CYCLIC_ERRORS = 0x08,

    CAN_CYCLIC_COUNT
} CAN_CyclicIndexTypeDef;

typedef struct FOC_Handle FOC_HandleTypeDef;

void FOC_SetNodeId(FOC_HandleTypeDef *hfoc, uint8_t node_id);
uint8_t FOC_GetNodeId(FOC_HandleTypeDef *hfoc);
void FOC_SetCyclicRate(FOC_HandleTypeDef *hfoc, CAN_CyclicIndexTypeDef index, uint32_t rate);
uint32_t FOC_GetCyclicRate(FOC_HandleTypeDef *hfoc, CAN_CyclicIndexTypeDef index);
void FOC_ProcessCANMessage(FOC_HandleTypeDef *hfoc);
void FOC_TransmitCyclicCANMessage(FOC_HandleTypeDef *hfoc);
void CAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
#endif /* FOC_CAN_H */