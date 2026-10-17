#ifndef FOC_Diagnostics_H
#define FOC_Diagnostics_H

#include "FOC_Utils.h"

typedef struct FOC_Handle FOC_HandleTypeDef;

#define FOC_ERROR_ESTOP (1u << 0)
#define FOC_ERROR_SYSTEM_FAULT (1u << 1)
#define FOC_ERROR_TIMING_VIOLATION (1u << 2)
#define FOC_ERROR_INVALID_CONFIGURATION (1u << 3)

#define FOC_ERROR_DRIVER_FAULT (1u << 4)

#define FOC_ERROR_MOTOR_OT (1u << 5)
#define FOC_ERROR_MOTOR_UT (1u << 6)
#define FOC_ERROR_MOTOR_DISCONNECTED (1u << 7)
#define FOC_ERROR_MOTOR_STALL (1u << 8)

#define FOC_ERROR_MOSFET_OT (1u << 9)
#define FOC_ERROR_MOSFET_UT (1u << 10)

#define FOC_ERROR_VBUS_OV (1u << 11)
#define FOC_ERROR_VBUS_UV (1u << 12)

#define FOC_ERROR_IBUS_OC (1u << 13)

#define FOC_ERROR_ENCODER_FAULT (1u << 14)

void FOC_CheckErrors(FOC_HandleTypeDef *hfoc);
void FOC_ClearLatchedErrors(FOC_HandleTypeDef *hfoc);

void FOC_SetError(FOC_HandleTypeDef *hfoc, uint32_t error);
uint32_t FOC_GetActiveErrors(FOC_HandleTypeDef *hfoc);
uint32_t FOC_GetLatchedErrors(FOC_HandleTypeDef *hfoc);

#endif // FOC_Diagnostics_H