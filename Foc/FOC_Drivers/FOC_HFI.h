#ifndef FOC_HFI_H
#define FOC_HFI_H

#include "Utils.h"
#include "Filters.h"

typedef struct FOC_Handle FOC_HandleTypeDef;

typedef struct {
    float injection_phase;
    float i_alpha_l_raw;
    float i_beta_l_raw;
    float i_alpha_l_filtered;
    float i_beta_l_filtered;
    BiquadFilter i_alpha_l_filter;
    BiquadFilter i_beta_l_filter;
} HFITypeDef;

void FOC_HFI_Init(FOC_HandleTypeDef *hfoc);
float FOC_HFI_GetInjectedVoltage(FOC_HandleTypeDef *hfoc);
float FOC_HFI_GetAngleEstimate(FOC_HandleTypeDef *hfoc);




#endif /* FOC_HFI_H */