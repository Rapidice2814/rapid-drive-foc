#ifndef AS5047P_DRIVER_H
#define AS5047P_DRIVER_H

#include "main.h"
#include <stdlib.h>
#include <stdbool.h>

typedef enum{
	AS5047P_OK,
	AS5047P_NORESPONSE,
	AS5047P_ERROR
}AS5047P_StatusTypeDef;

typedef struct {
	/* SPI */
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *slave_select_port;
	uint16_t slave_select_pin;

	/* Flags */
	uint8_t pins_set;
	uint8_t setup_complete;

}AS5047P_HandleTypeDef;

AS5047P_StatusTypeDef AS5047P_SetPins(AS5047P_HandleTypeDef *has5047p, SPI_HandleTypeDef *hspi, GPIO_TypeDef *slave_select_port, uint16_t slave_select_pin);
AS5047P_StatusTypeDef AS5047P_Init(AS5047P_HandleTypeDef *has5047p);
AS5047P_StatusTypeDef AS5047P_GetAngle(AS5047P_HandleTypeDef *has5047p, float *angle);
AS5047P_StatusTypeDef AS5047P_GetAngle_Raw(AS5047P_HandleTypeDef *has5047p, float *angle);
AS5047P_StatusTypeDef AS5047P_GetCMAG(AS5047P_HandleTypeDef *has5047p, uint16_t *cmag);
AS5047P_StatusTypeDef AS5047P_GetDIAAGC(AS5047P_HandleTypeDef *has5047p, uint16_t *value);



#endif // AS5047P_DRIVER_H