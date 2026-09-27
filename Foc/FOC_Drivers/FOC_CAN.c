#include "FOC_CAN.h"
#include "FOC_Handle.h"
#include "Utils.h"
#include <math.h>
#include <string.h>

void FOC_SetNodeId(FOC_HandleTypeDef *hfoc, uint8_t node_id){
    if (hfoc->phfdcan == NULL) return;
    
    
    if (hfoc->phfdcan->State == HAL_FDCAN_STATE_BUSY){
        HAL_FDCAN_Stop(hfoc->phfdcan); // Stop the FDCAN peripheral if it is busy
    }
    
    FDCAN_FilterTypeDef sFilterConfig = {0};
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterID1 = 0x000; // Accept id 0, broadcast
    sFilterConfig.FilterID2 = ID_MASK;
    if (HAL_FDCAN_ConfigFilter(hfoc->phfdcan, &sFilterConfig) != HAL_OK){
        Error_Handler();
    }
    
    sFilterConfig.FilterIndex = 1;
    sFilterConfig.FilterID1 = node_id & ID_MASK; // Accept node id
    sFilterConfig.FilterID2 = ID_MASK;
    if (HAL_FDCAN_ConfigFilter(hfoc->phfdcan, &sFilterConfig) != HAL_OK){
        Error_Handler();
    }
    
    if (HAL_FDCAN_Start(hfoc->phfdcan) != HAL_OK){
        Error_Handler();
    }
    
    if (HAL_FDCAN_ActivateNotification(hfoc->phfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK){
        Error_Handler();
    }

    hfoc->flash_data.node.node_id = node_id;
    
}

uint8_t FOC_GetNodeId(FOC_HandleTypeDef *hfoc){
    return hfoc->flash_data.node.node_id;
}

void FOC_SetHeartbeatRate(FOC_HandleTypeDef *hfoc, uint16_t rate){
    hfoc->flash_data.node.heartbeat_msg_rate = rate;
}

void FOC_SetEncoderRate(FOC_HandleTypeDef *hfoc, uint16_t rate){
    hfoc->flash_data.node.encoder_msg_rate = rate;
}

uint16_t FOC_GetEncoderRate(FOC_HandleTypeDef *hfoc){
    return hfoc->flash_data.node.encoder_msg_rate;
}

uint16_t FOC_GetHeartbeatRate(FOC_HandleTypeDef *hfoc){
    return hfoc->flash_data.node.heartbeat_msg_rate;
}


void FOC_TransmitCANMessage(FOC_HandleTypeDef *hfoc, CommandTypeDef command){
    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_ON;
    TxHeader.FDFormat = FDCAN_FD_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    TxHeader.Identifier = GET_CAN_ID(hfoc->flash_data.node.node_id, 1, command);

    uint8_t TxData[64];

    switch (command) { //sent commands
        case CMD_ESTOP:
            break;
        case CMD_VERSION:
            TxData[0] = (uint8_t)(FOC_VERSION_MAJOR);
            TxData[1] = (uint8_t)(FOC_VERSION_MINOR);
            TxData[2] = (uint8_t)(FOC_VERSION_PATCH);
            TxHeader.DataLength = FDCAN_DLC_BYTES_3;
            break;
        case CMD_HEARTBEAT:
            TxData[0] = 0xFF & hfoc->state; // current state of the FOC driver
            TxData[1] = (uint8_t)fminf(fmaxf(hfoc->adc_values.motor_temp, 0.0f), 255.0f); // Send the temperature as the second byte, in C
            uint16_t timestamp = (uint16_t)(FOC_GetTick(hfoc) & 0xFFFF); // Get the current timestamp
            memcpy(&TxData[2], &timestamp, sizeof(uint16_t)); //byte 2-3
            TxHeader.DataLength = FDCAN_DLC_BYTES_4;
            break;
        case CMD_ENCODER:
            write_float_le(&TxData[0], hfoc->encoder_angle_mechanical_unwrapped); //byte 0-3
            write_float_le(&TxData[4], hfoc->encoder_speed_mechanical); //byte 4-7
            TxHeader.DataLength = FDCAN_DLC_BYTES_8;
            break;
        default:
            return;
    }

    if(HAL_FDCAN_GetTxFifoFreeLevel(hfoc->phfdcan) == 0){
        return;
    }
    
    if (HAL_FDCAN_AddMessageToTxFifoQ(hfoc->phfdcan, &TxHeader, TxData) != HAL_OK) {
        Error_Handler();
    }
}



void FOC_ProcessCANMessage(FOC_HandleTypeDef *hfoc){
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[64];
    if(HAL_FDCAN_GetRxFifoFillLevel(hfoc->phfdcan, FDCAN_RX_FIFO0) > 0){
        HAL_GPIO_TogglePin(DEBUG_LED0_GPIO_Port, DEBUG_LED0_Pin); //toggle debug led to indicate a message was received
        if (HAL_FDCAN_GetRxMessage(hfoc->phfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK){
            Error_Handler();
        }
        if (RxHeader.IdType != FDCAN_STANDARD_ID) return; // Only process standard ID messages

        uint8_t command = GET_COMMAND_FROM_CAN_ID(RxHeader.Identifier);
        uint8_t node_id = GET_ID_FROM_CAN_ID(RxHeader.Identifier);

        if (node_id == 0) {
            return;
        }

        switch (command) { //received commands
            case CMD_ESTOP:
                break;
            case CMD_VERSION:
                FOC_TransmitCANMessage(hfoc, CMD_VERSION);
                break;
            case CMD_SET_TORQUE: //0-3byte float, torque setpoint
                if (RxHeader.DataLength != FDCAN_DLC_BYTES_4) return;
                float torque_setpoint = read_float_le(RxData);
                hfoc->dq_current_setpoint.q = torque_setpoint * hfoc->flash_data.motor.torque_constant;
                break;
            case CMD_SET_CURRENT: //0-3byte float, current setpoint
                if (RxHeader.DataLength != FDCAN_DLC_BYTES_4) return;
                hfoc->dq_current_setpoint.q = read_float_le(RxData);
                break;
            case CMD_SET_SPEED: //0-3byte float, speed setpoint
                if (RxHeader.DataLength != FDCAN_DLC_BYTES_4) return;
                hfoc->speed_setpoint = read_float_le(RxData);
                break;
            case CMD_SET_POSITION: //0-3byte float, position setpoint
            if (RxHeader.DataLength != FDCAN_DLC_BYTES_4) return;
                hfoc->angle_setpoint = read_float_le(RxData);
                break;
            default:
                return;
        }

    }
}

void FOC_TransmitCyclicCANMessage(FOC_HandleTypeDef *hfoc){
    if(hfoc->flash_data.node.node_id == 0) return;

    static uint32_t last_heartbeat_tick = 0;
    static uint32_t last_encoder_tick = 0;

    if ((hfoc->flash_data.node.heartbeat_msg_rate != 0) && (FOC_GetTick(hfoc) - last_heartbeat_tick >= hfoc->flash_data.node.heartbeat_msg_rate)) {
        last_heartbeat_tick = FOC_GetTick(hfoc);
        FOC_TransmitCANMessage(hfoc, CMD_HEARTBEAT);
    }

    if ((hfoc->flash_data.node.encoder_msg_rate != 0) && (FOC_GetTick(hfoc) - last_encoder_tick >= hfoc->flash_data.node.encoder_msg_rate)) {
        last_encoder_tick = FOC_GetTick(hfoc);
        FOC_TransmitCANMessage(hfoc, CMD_ENCODER);
    }
}



void CAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
    (void)hfdcan; //unused
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET){

    }
}