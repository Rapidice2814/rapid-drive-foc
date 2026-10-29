#ifndef TEST_MAIN_H
#define TEST_MAIN_H
#include <stdint.h>
#include <stddef.h>
#define UNUSED(x) ((void)(x))
#define __NOP() __asm__ volatile("nop")
#define CURRENT_LOOP_FREQUENCY 8000
#define SPEED_LOOP_CLOCK_DIVIDER 8
#define ENCODER_PULSES_PER_ROTATION 4096
typedef struct {int unused;} FDCAN_HandleTypeDef;
typedef enum {HAL_OK, HAL_ERROR, HAL_BUSY} HAL_StatusTypeDef;
typedef enum {HAL_SPI_STATE_READY, HAL_SPI_STATE_BUSY_TX_RX} HAL_SPI_StateTypeDef;
typedef enum {GPIO_PIN_RESET, GPIO_PIN_SET} GPIO_PinState;
typedef struct {unsigned value;} GPIO_TypeDef;
typedef struct {void *hdmarx, *hdmatx; HAL_SPI_StateTypeDef state;} SPI_HandleTypeDef;
HAL_SPI_StateTypeDef HAL_SPI_GetState(SPI_HandleTypeDef *);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t, uint32_t);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t);
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *);
void HAL_GPIO_WritePin(GPIO_TypeDef *, uint16_t, GPIO_PinState);
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *);
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *);
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t);
#endif
