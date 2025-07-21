#include "FOC_CAN.h"


void FOC_SetNodeId(FOC_HandleTypeDef *hfoc, uint8_t node_id) {
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

static uint8_t can_data[8] = {0x00, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};

static uint32_t GetFDCanDLC(uint8_t len);
void FOC_TransmitCANMessage(FOC_HandleTypeDef *hfoc, CommandTypeDef command){
    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_ON;
    TxHeader.FDFormat = FDCAN_FD_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    TxHeader.Identifier = (hfoc->flash_data.node.id & ID_MASK) | ((command << 4) & COMMAND_MASK);

    TxHeader.DataLength = GetFDCanDLC(sizeof(can_data));
    if (HAL_FDCAN_AddMessageToTxFifoQ(hfoc->phfdcan, &TxHeader, can_data) != HAL_OK) {
        Error_Handler();
    }

    can_data[0]++;
}





static volatile uint8_t can_rx_counter = 0;

void FOC_ProcessCANMessage(FOC_HandleTypeDef *hfoc){
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[64];
    if(can_rx_counter > 0){
        can_rx_counter--;
        if (HAL_FDCAN_GetRxMessage(hfoc->phfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK){
            Error_Handler();
        }
    }
}


void CAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
    if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET){
        if(can_rx_counter < 3){
            can_rx_counter++;
        }else{
            Error_Handler();
        }
    }
}



static uint32_t GetFDCanDLC(uint8_t len) {
    if (len <= 8)
        return len;  // DLC == length directly
    else if (len <= 12)
        return FDCAN_DLC_BYTES_12;
    else if (len <= 16)
        return FDCAN_DLC_BYTES_16;
    else if (len <= 20)
        return FDCAN_DLC_BYTES_20;
    else if (len <= 24)
        return FDCAN_DLC_BYTES_24;
    else if (len <= 32)
        return FDCAN_DLC_BYTES_32;
    else if (len <= 48)
        return FDCAN_DLC_BYTES_48;
    else
        return FDCAN_DLC_BYTES_64;
}
