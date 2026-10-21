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
    BiquadFilter positive_alpha_filter, positive_beta_filter;
    float positive_alpha, positive_beta, injection_alpha, injection_beta;
    float angle_axis_mod_pi, angle_physical, angle_electrical;
    float angle_mechanical_unwrapped, speed_electrical, speed_mechanical;
    float signal_amplitude, saliency_ratio, error_electrical, error_mechanical;
    uint32_t samples, good_samples, status;
    uint8_t active, seeded, locked, valid, feedback_source;
} HFITypeDef;

void FOC_HFI_Init(FOC_HandleTypeDef *hfoc);
#define FOC_FEEDBACK_ENCODER 0u
#define FOC_FEEDBACK_HFI 1u
#define HFI_STATUS_ENABLED 1u
#define HFI_STATUS_ACTIVE 2u
#define HFI_STATUS_SEEDED 4u
#define HFI_STATUS_LOCKED 8u
#define HFI_STATUS_VALID 16u
#define HFI_STATUS_SELECTED 32u
uint8_t FOC_HFI_Update(FOC_HandleTypeDef *hfoc, float i_alpha, float i_beta);
uint8_t FOC_HFI_SelectFeedback(FOC_HandleTypeDef *hfoc, uint8_t source);
float FOC_FeedbackElectricalAngle(const FOC_HandleTypeDef *hfoc);
float FOC_FeedbackMechanicalAngle(const FOC_HandleTypeDef *hfoc);
float FOC_FeedbackMechanicalSpeed(const FOC_HandleTypeDef *hfoc);




#endif /* FOC_HFI_H */
