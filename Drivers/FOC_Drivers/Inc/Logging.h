#ifndef LOGGING_H
#define LOGGING_H

#include <stdio.h>
#include <stdarg.h>
#include "main.h"

void Log_Setup(UART_HandleTypeDef *huart);
void Log_Queue(const char* format, ...) __attribute__((format(printf, 1, 2)));
void Log_Loop();
void Log_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void Log_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);

void Log_ProcessRxPacket(const char* packet, uint16_t Length);


#endif // LOGGING_H