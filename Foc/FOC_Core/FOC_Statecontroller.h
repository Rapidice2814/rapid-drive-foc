#ifndef FOC_STATECONTROLLER_H
#define FOC_STATECONTROLLER_H

#include "FOC_Utils.h"


void FOC_StateLoop(FOC_HandleTypeDef *hfoc);
FOC_StatusTypeDef FOC_SetState(FOC_HandleTypeDef *hfoc, FOC_StateTypeDef new_state);

#endif // FOC_STATECONTROLLER_H