#ifndef FOC_CAN_H
#define FOC_CAN_H

#include "main.h"
#include <stdint.h>

/**
 * @file
 * @brief CAN command and cyclic-message declarations.
 * @see docs/can-protocol.md for identifiers, payloads, and command behavior.
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
void FOC_CAN_Loop(FOC_HandleTypeDef *hfoc);
void CAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
#endif /* FOC_CAN_H */
