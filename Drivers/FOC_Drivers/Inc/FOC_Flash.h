#ifndef FOC_FLASH_H
#define FOC_FLASH_H

#include "stm32g4xx_hal.h"

typedef enum {
    FLASH_OK = 0,
    FLASH_ERROR = 1
} FLASH_StatusTypeDef;


typedef struct {
    uint8_t contains_data;
    uint8_t motor_direction_swapped;

    /* Encoder */
    uint8_t encoder_aligned_flag;  //flag for the encoder alignment
    float encoder_angle_mechanical_offset;  //offset for the encoder angle [radians]

    

} FLASH_DataTypeDef;

FLASH_StatusTypeDef FOC_FLASH_WriteData(FLASH_DataTypeDef *pdata);
FLASH_StatusTypeDef FOC_FLASH_ReadData(FLASH_DataTypeDef *pdata);














#endif // FOC_FLASH_H