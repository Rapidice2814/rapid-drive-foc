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

    /* DMA buffers outlive both SPI frames; ready remains valid until restart. */
    uint16_t dma_tx;
    volatile uint16_t dma_rx;
    volatile uint16_t dma_angle;
    volatile uint8_t dma_stage, dma_ready, dma_error;

}AS5047P_HandleTypeDef;

AS5047P_StatusTypeDef AS5047P_SetPins(AS5047P_HandleTypeDef *has5047p, SPI_HandleTypeDef *hspi, GPIO_TypeDef *slave_select_port, uint16_t slave_select_pin);
AS5047P_StatusTypeDef AS5047P_Init(AS5047P_HandleTypeDef *has5047p);
AS5047P_StatusTypeDef AS5047P_GetAngle(AS5047P_HandleTypeDef *has5047p, uint16_t *angle);
AS5047P_StatusTypeDef AS5047P_RequestAngleDMA(AS5047P_HandleTypeDef *encoder);
AS5047P_StatusTypeDef AS5047P_GetAngleDMA(AS5047P_HandleTypeDef *encoder, uint16_t *angle);
void AS5047P_StopAngleDMA(AS5047P_HandleTypeDef *encoder);
AS5047P_StatusTypeDef AS5047P_GetAngle_Raw(AS5047P_HandleTypeDef *has5047p, uint16_t *angle);
AS5047P_StatusTypeDef AS5047P_GetCMAG(AS5047P_HandleTypeDef *has5047p, uint16_t *cmag);
AS5047P_StatusTypeDef AS5047P_GetDIAAGC(AS5047P_HandleTypeDef *has5047p, uint16_t *value);
AS5047P_StatusTypeDef AS5047P_SetZeroAngle(AS5047P_HandleTypeDef *has5047p);



#endif // AS5047P_DRIVER_H
