#ifndef FOC_CAN_H
#define FOC_CAN_H

#include "main.h"
#include "FOC_Driver.h"
#include <stdint.h>

#define ID_MASK 0x0F                // 4-bit ID mask
#define COMMAND_MASK (0x7F << 4)    // 7-bit command mask


typedef enum {
    CMD_SET_ESTOP,
    CMD_SET_ADDRESS,
    CMD_SET_STATE,
    CMD_SET_INPUT_TORQUE,
    CMD_SET_INPUT_SPEED,
    CMD_SET_INPUT_POSITION,
    CMD_SET_LIMITS,
    CMD_SET_REQUEST,
    CMD_GET_PING,
    CMD_GET_HEARTBEAT,
    CMD_GET_ERROR,
    CMD_GET_STATUS
} CommandTypeDef;

void FOC_ProcessCANMessage(FOC_HandleTypeDef *hfoc);
void FOC_TransmitCANMessage(FOC_HandleTypeDef *hfoc, CommandTypeDef command);
void CAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
#endif /* FOC_CAN_H */