#ifndef DEBUG_H
#define DEBUG_H

#include "main.h"
#include "FOC_Driver.h"

void Debug_Setup();
void Debug_Queue(FOC_HandleTypeDef *hfoc);
void Debug_Loop();



#endif // DEBUG_H