#include "CAN_Driver.h"
#include "FOC.h"


void CAN_SetupFilter(FDCAN_HandleTypeDef *hfdcan, uint8_t driver_id) {
    FDCAN_FilterTypeDef sFilterConfig0 = {0};

    sFilterConfig0.IdType = FDCAN_STANDARD_ID;
    sFilterConfig0.FilterIndex = 0;
    sFilterConfig0.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig0.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig0.FilterID1 = 0x000; // Accept id 0, broadcast
    sFilterConfig0.FilterID2 = ID_MASK;

    if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig0) != HAL_OK) {
        Error_Handler();
    }

    FDCAN_FilterTypeDef sFilterConfig1 = {0};

    sFilterConfig1.IdType = FDCAN_STANDARD_ID;
    sFilterConfig1.FilterIndex = 1;
    sFilterConfig1.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig1.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig1.FilterID1 = driver_id & ID_MASK; // Accept driver id
    sFilterConfig1.FilterID2 = ID_MASK;

    if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig1) != HAL_OK) {
        Error_Handler();
    }
}
