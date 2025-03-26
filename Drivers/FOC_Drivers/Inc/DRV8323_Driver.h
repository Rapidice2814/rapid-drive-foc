/*
 * DRV8323_Driver.h
 *
 *  Created on: Nov 1, 2024
 *      Author: Csongor
 */

#ifndef INC_DRV8323_DRIVER_H_
#define INC_DRV8323_DRIVER_H_

#include "main.h"
#include <stdlib.h>
#include <stdbool.h>

typedef enum{
	DRV8323_OK,
	DRV8323_NORESPONSE,
	DRV8323_ERROR
}DRV8323_StatusTypeDef;

typedef struct {
	/* SPI */
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *slave_select_port;
	uint16_t slave_select_pin;

	/* GPIO outputs */
	GPIO_TypeDef *enable_port;
	uint16_t enable_pin;

	/* GPIO input */
	GPIO_TypeDef *nfault_port;
	uint16_t nfault_pin;

	/* Flags */
	uint8_t pins_set;
	uint8_t driver_enabled;
	uint8_t setup_complete;

}DRV8323_HandleTypeDef;

DRV8323_StatusTypeDef DRV8323_SetPins(DRV8323_HandleTypeDef *hdrv8323, SPI_HandleTypeDef *hspi, GPIO_TypeDef *slave_select_port, uint16_t slave_select_pin, GPIO_TypeDef *enable_port, uint16_t enable_pin, GPIO_TypeDef *nfault_port, uint16_t nfault_pin);
DRV8323_StatusTypeDef DRV8323_Init(DRV8323_HandleTypeDef *hdrv8323);

uint8_t DRV8323_CheckFault(DRV8323_HandleTypeDef *hdrv8323);
DRV8323_StatusTypeDef DRV8323_ReadFaultStatusRegister1(DRV8323_HandleTypeDef *hdrv8323, uint16_t *data);
DRV8323_StatusTypeDef DRV8323_ReadFaultStatusRegister2(DRV8323_HandleTypeDef *hdrv8323, uint16_t *data);

DRV8323_StatusTypeDef DRV8323_ClearFaults(DRV8323_HandleTypeDef *hdrv8323);
DRV8323_StatusTypeDef DRV8323_CSACALStart(DRV8323_HandleTypeDef *hdrv8323);
DRV8323_StatusTypeDef DRV8323_CSACALStop(DRV8323_HandleTypeDef *hdrv8323);

DRV8323_StatusTypeDef DRV8323_Enable(DRV8323_HandleTypeDef *hdrv8323);
DRV8323_StatusTypeDef DRV8323_Disable(DRV8323_HandleTypeDef *hdrv8323);
#endif /* INC_DRV8323_DRIVER_H_ */
