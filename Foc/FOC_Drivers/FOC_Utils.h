#ifndef FOC_UTILS_H
#define FOC_UTILS_H

#include "main.h"

typedef enum{
	FOC_OK,
	FOC_ERROR
}FOC_StatusTypeDef;

typedef struct {
    float a;
    float b;
    float c;
} PhaseCurrentsTypeDef;

typedef struct {
    float alpha;
    float beta;
} ABCurrentsTypeDef;

typedef struct {
    float d;
    float q;
} DQCurrentsTypeDef;

typedef struct {
    float a;
    float b;
    float c;
} PhaseVoltagesTypeDef;

typedef struct {
    float alpha;
    float beta;
} ABVoltagesTypeDef;

typedef struct {
    float d;
    float q;
} DQVoltagesTypeDef;


typedef struct FOC_Handle FOC_HandleTypeDef;


/* General */
FOC_StatusTypeDef FOC_Init(FOC_HandleTypeDef *hfoc);

/* Calculations */
ABCurrentsTypeDef Clarke_transform(PhaseCurrentsTypeDef current);
DQCurrentsTypeDef Park_transform(ABCurrentsTypeDef ab_current, float theta);
ABVoltagesTypeDef InvPark_transform(DQVoltagesTypeDef dq_voltage, float theta);
PhaseVoltagesTypeDef InvClarke_transform(ABVoltagesTypeDef ab_voltage);
float CalculateBusCurrent(PhaseCurrentsTypeDef phase_current, PhaseVoltagesTypeDef phase_voltage, float vbus);

FOC_StatusTypeDef FOC_SetPhaseVoltages(FOC_HandleTypeDef *hfoc, PhaseVoltagesTypeDef phase_voltage);

/* Encoder */
FOC_StatusTypeDef FOC_SetEncoderPointer(FOC_HandleTypeDef *hfoc, volatile uint32_t *encoder_count);
FOC_StatusTypeDef FOC_SetEncoderZero(FOC_HandleTypeDef *hfoc);
FOC_StatusTypeDef FOC_UpdateEncoder(FOC_HandleTypeDef *hfoc, float frequency);
/* PWM */
FOC_StatusTypeDef FOC_SetPWMCCRPointers(FOC_HandleTypeDef *hfoc, volatile uint32_t *pCCRa, volatile uint32_t *pCCRb, volatile uint32_t *pCCRc, uint32_t max_ccr);

#endif // FOC_UTILS_H