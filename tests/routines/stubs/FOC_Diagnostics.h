#include "FOC_Handle.h"
#define FOC_ERROR_OPERATION_FAILED (1u << 15)
#define FOC_ERROR_ENCODER_FAULT (1u << 14)
#define FOC_ERROR_HFI_FAULT (1u << 16)
void FOC_SetError(FOC_HandleTypeDef *, uint32_t);
void FOC_CheckErrors(FOC_HandleTypeDef *);
void FOC_ClearLatchedErrors(FOC_HandleTypeDef *);
