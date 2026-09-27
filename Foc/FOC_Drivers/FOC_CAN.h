#ifndef FOC_CAN_H
#define FOC_CAN_H

#include "main.h"
#include <stdint.h>

#define ID_MASK 0x0F                // 4-bit ID mask
#define DIRECTION_MASK (0x01 << 4)  // 1-bit direction mask, 0 when the command is received by the node, 1 when the command is sent from the node
#define COMMAND_MASK (0x3F << 5)    // 6-bit command mask

#define GET_CAN_ID(node_id, direction, command) ((node_id & ID_MASK) | ((direction << 4) & DIRECTION_MASK) | ((command << 5) & COMMAND_MASK)) // Constructs the CAN identifier, 4-bit ID, 1-bit direction, 6-bit command
#define GET_ID_FROM_CAN_ID(can_id) (can_id & ID_MASK) // Extracts the ID from the CAN identifier
#define GET_DIRECTION_FROM_CAN_ID(can_id) ((can_id & DIRECTION_MASK) >> 4) // Extracts the direction from the CAN identifier
#define GET_COMMAND_FROM_CAN_ID(can_id) ((can_id & COMMAND_MASK) >> 5) // Extracts the command from the CAN identifier


typedef enum {
    CMD_ESTOP = 0x00,
    CMD_VERSION = 0x01,
    CMD_HEARTBEAT = 0x02,
    CMD_SET_TORQUE = 0x03,
    CMD_SET_CURRENT = 0x04,
    CMD_SET_SPEED = 0x05,
    CMD_SET_POSITION = 0x06,
    CMD_ENCODER = 0x07,
} CommandTypeDef;

typedef struct FOC_Handle FOC_HandleTypeDef;

void FOC_SetNodeId(FOC_HandleTypeDef *hfoc, uint8_t node_id);
uint8_t FOC_GetNodeId(FOC_HandleTypeDef *hfoc);
void FOC_SetHeartbeatRate(FOC_HandleTypeDef *hfoc, uint16_t rate);
uint16_t FOC_GetHeartbeatRate(FOC_HandleTypeDef *hfoc);
void FOC_SetEncoderRate(FOC_HandleTypeDef *hfoc, uint16_t rate);
uint16_t FOC_GetEncoderRate(FOC_HandleTypeDef *hfoc);
void FOC_ProcessCANMessage(FOC_HandleTypeDef *hfoc);
void FOC_TransmitCANMessage(FOC_HandleTypeDef *hfoc, CommandTypeDef command);
void FOC_TransmitCyclicCANMessage(FOC_HandleTypeDef *hfoc);
void CAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
#endif /* FOC_CAN_H */