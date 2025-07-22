#include "FOC_CAN.h"
#include <math.h>
#include <string.h>

union {
    uint32_t u;
    float f;
} conv; //used to convert between float and uint32_t

void FOC_SetNodeId(FOC_HandleTypeDef *hfoc, uint8_t node_id){
    if (hfoc->phfdcan == NULL) return;
    
    hfoc->flash_data.node.id = node_id;
    
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
    sFilterConfig.FilterID1 = hfoc->flash_data.node.id & ID_MASK; // Accept node id
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

    TxHeader.Identifier = GET_CAN_ID(hfoc->flash_data.node.id, 1, command);

    uint8_t TxData[64];

    switch (command) { //sent commands
        case CMD_ESTOP:
            break;
        case CMD_VERSION:
            break;
        case CMD_ADDRESS:
            break;
        case CMD_STATE:
            break;
        case CMD_LIMITS:
            break;
        case CMD_REQUEST:
            break;
        case CMD_PING:
            break;
        case CMD_HEARTBEAT:
            TxData[0] = 0xFF & hfoc->state; // current state of the FOC driver
            TxData[1] = (uint8_t)fminf(fmaxf(hfoc->NTC_temp, 0.0f), 255.0f); // Send the temperature as the second byte, in C
            uint16_t timestamp = (uint16_t)(HAL_GetTick() & 0xFFFF); // Get the current timestamp, wraps around every 65536 ms
            memcpy(&TxData[2], &timestamp, sizeof(uint16_t)); //byte 2-3
            TxHeader.DataLength = FDCAN_DLC_BYTES_4;
            break;
        case CMD_ERROR:
            break;
        case CMD_STATUS:
        default:
            return; // Invalid command
    }
    
    if (HAL_FDCAN_AddMessageToTxFifoQ(hfoc->phfdcan, &TxHeader, TxData) != HAL_OK) {
        // Error_Handler();
        hfoc->state = FOC_STATE_ERROR;
    }
}




static volatile uint8_t can_rx_counter = 0;

void FOC_ProcessCANMessage(FOC_HandleTypeDef *hfoc){
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[64];
    if(can_rx_counter > 0){
        // HAL_GPIO_TogglePin(PB2_GPIO_Port, PB2_Pin);
        HAL_GPIO_WritePin(PB2_GPIO_Port, PB2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(PB2_GPIO_Port, PB2_Pin, GPIO_PIN_RESET);
        can_rx_counter--;
        if (HAL_FDCAN_GetRxMessage(hfoc->phfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK){
            Error_Handler();
        }
        if (RxHeader.IdType != FDCAN_STANDARD_ID) return; // Only process standard ID messages

        uint8_t command = GET_COMMAND_FROM_CAN_ID(RxHeader.Identifier);
        // uint8_t node_id = GET_ID_FROM_CAN_ID(RxHeader.Identifier);

        switch (command) { //received commands
            case CMD_ESTOP:
                break;
            case CMD_VERSION:
                break;
            case CMD_ADDRESS:
                break;
            case CMD_STATE:
                break;
            case CMD_SET_TORQUE: //0-3byte float, torque setpoint
                if (RxHeader.DataLength != FDCAN_DLC_BYTES_4) return;
                memcpy(&conv.u, RxData, sizeof(uint32_t));
                hfoc->dq_current_setpoint.q = conv.f;
                break;
            case CMD_SET_SPEED: //0-3byte float, speed setpoint
                if (RxHeader.DataLength != FDCAN_DLC_BYTES_4) return;
                memcpy(&conv.u, RxData, sizeof(uint32_t));
                hfoc->speed_setpoint = conv.f;
                break;
            case CMD_SET_POSITION: //0-3byte float, position setpoint
            if (RxHeader.DataLength != FDCAN_DLC_BYTES_4) return;
                memcpy(&conv.u, RxData, sizeof(uint32_t));
                hfoc->angle_setpoint = conv.f;
                break;
            case CMD_LIMITS:
                break;
            case CMD_REQUEST:
                break;
            case CMD_PING:
                break;
            case CMD_HEARTBEAT:
                break;
            case CMD_ERROR:
                break;
            case CMD_STATUS:
            default:
                return; // Invalid command
        }



    }
}


static uint32_t last_heartbeat_time_ms = 0;

void FOC_TransmitCyclicCANMessage(FOC_HandleTypeDef *hfoc){
    if(hfoc->flash_data.node.id == 0) return;

    if ((hfoc->flash_data.node.heartbeat_msg_rate_ms != 0) && (HAL_GetTick() - last_heartbeat_time_ms >= hfoc->flash_data.node.heartbeat_msg_rate_ms)) {
        last_heartbeat_time_ms = HAL_GetTick();
        FOC_TransmitCANMessage(hfoc, CMD_HEARTBEAT);
    }
}



void CAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
    (void)hfdcan; //unused
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET){
        if(can_rx_counter < 3){
            can_rx_counter++;
        }else{
            Error_Handler();
        }
    }
}