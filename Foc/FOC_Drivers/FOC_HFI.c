#include "FOC_HFI.h"
#include "FOC_Handle.h"
#include "main.h"

#include "math.h"

void FOC_HFI_Init(FOC_HandleTypeDef *hfoc){
    biquad_filter_init(&hfoc->hfi.i_alpha_l_filter, BIQUAD_LOW_PASS, 200.0, CURRENT_LOOP_FREQUENCY, 0.707f);
    biquad_filter_init(&hfoc->hfi.i_beta_l_filter, BIQUAD_LOW_PASS, 200.0, CURRENT_LOOP_FREQUENCY, 0.707f);

    // biquad_filter_init(&hfoc->dq_current_d_filter, BIQUAD_LOW_PASS, HFI_CURRENT_LPF_CUTOFF_FREQ, CURRENT_LOOP_FREQUENCY, 0.707f);
    // biquad_filter_init(&hfoc->dq_current_q_filter, BIQUAD_LOW_PASS, HFI_CURRENT_LPF_CUTOFF_FREQ, CURRENT_LOOP_FREQUENCY, 0.707f);

    hfoc->hfi.injection_phase = 0.0f;
}


float FOC_HFI_GetInjectedVoltage(FOC_HandleTypeDef *hfoc){
    float voltage = hfoc->flash_data.hfi.injection_amplitude * cosf(hfoc->hfi.injection_phase);
    hfoc->hfi.injection_phase += hfoc->flash_data.hfi.injection_omega / CURRENT_LOOP_FREQUENCY;
    normalize_angle_0_2pi(&hfoc->hfi.injection_phase);

    FOC_HFI_GetAngleEstimate(hfoc);

    return voltage;
}

float FOC_HFI_GetAngleEstimate(FOC_HandleTypeDef *hfoc){
    float  two_sin_omega = 2.0f * sinf(hfoc->hfi.injection_phase);

    hfoc->hfi.i_alpha_l_raw = hfoc->ab_current.alpha * two_sin_omega;
    hfoc->hfi.i_beta_l_raw = hfoc->ab_current.beta * two_sin_omega;

    hfoc->hfi.i_alpha_l_filtered = biquad_filter_update(&hfoc->hfi.i_alpha_l_filter, hfoc->hfi.i_alpha_l_raw);
    hfoc->hfi.i_beta_l_filtered = biquad_filter_update(&hfoc->hfi.i_beta_l_filter, hfoc->hfi.i_beta_l_raw);


    return 0.0f;
}