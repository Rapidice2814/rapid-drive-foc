#ifndef LOGGING_H
#define LOGGING_H

#include <stdio.h>
#include <stdarg.h>
#include "main.h"

void Log_Setup(UART_HandleTypeDef *huart);
void Log_Queue(const char* format, ...);
void Log_Loop();



#endif // LOGGING_H