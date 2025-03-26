#ifndef FOC_FLASH_H
#define FOC_FLASH_H

#include "stm32g4xx_hal.h"

typedef enum {
    FLASH_OK = 0,
    FLASH_ERROR = 1
} FLASH_StatusTypeDef;


typedef struct {
    uint32_t banana;
    float apple;
    uint8_t orange;
} FLASH_DataTypeDef;

FLASH_StatusTypeDef FOC_FLASH_WriteData(FLASH_DataTypeDef *data);
FLASH_StatusTypeDef FOC_FLASH_ReadData(FLASH_DataTypeDef *data);














#endif // FOC_FLASH_H