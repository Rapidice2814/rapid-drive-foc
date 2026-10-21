#include "FOC_HFI.h"
#include "FOC_Handle.h"
#include "FOC_Config.h"
#include <math.h>
#include <string.h>

/** @brief Rebuilds diagnostic flags without changing estimator state. */
static void status(FOC_HandleTypeDef *h){
    HFITypeDef *s = &h->hfi;
    s->status = (h->flash_data.hfi.hfi_enabled ? HFI_STATUS_ENABLED : 0) |
        (s->active ? HFI_STATUS_ACTIVE : 0) | (s->seeded ? HFI_STATUS_SEEDED : 0) |
        (s->locked ? HFI_STATUS_LOCKED : 0) | (s->valid ? HFI_STATUS_VALID : 0) |
        (s->feedback_source == FOC_FEEDBACK_HFI ? HFI_STATUS_SELECTED : 0);
}

/** @brief Resets HFI to encoder feedback without enabling excitation.
 * @note Saliency does not reveal magnetic polarity or absolute mechanical origin.
 */
void FOC_HFI_Init(FOC_HandleTypeDef *h){
    memset(&h->hfi, 0, sizeof(h->hfi));
    BiquadFilter *filters[] = {&h->hfi.i_alpha_l_filter, &h->hfi.i_beta_l_filter,
        &h->hfi.positive_alpha_filter, &h->hfi.positive_beta_filter};
    for(unsigned i = 0; i < 4; i++)
        biquad_filter_init(filters[i], BIQUAD_LOW_PASS, HFI_DEMOD_LPF_HZ, CURRENT_LOOP_FREQUENCY, 0.70710678f);
    /* The ordinary current regulator must not try to cancel the HF carrier. */
    biquad_filter_init(&h->dq_current_d_filter, BIQUAD_LOW_PASS, HFI_CURRENT_LPF_CUTOFF_FREQ, CURRENT_LOOP_FREQUENCY, 0.70710678f);
    biquad_filter_init(&h->dq_current_q_filter, BIQUAD_LOW_PASS, HFI_CURRENT_LPF_CUTOFF_FREQ, CURRENT_LOOP_FREQUENCY, 0.70710678f);
    status(h);
}

/** @brief Wraps a pi-periodic axial error to [-pi/2, pi/2]. */
static float axial_error(float error){
    error *= 2; normalize_angle_pm_pi(&error); return error * 0.5f;
}

/**
 * @brief Demodulates one current sample and generates the next rotating voltage.
 * @param h Controller with the independently sampled, aligned encoder reference.
 * @param i_alpha Stator alpha current before Park conversion.
 * @param i_beta Stator beta current before Park conversion.
 * @return 0 if selected HFI feedback lost validity, otherwise 1.
 * @note Midpoint phase matches the previous zero-order-held voltage. Encoder
 *       seeds branch/origin once and remains a safety check, but never corrects
 *       the running observer. This is not encoder-free absolute position sensing.
 */
uint8_t FOC_HFI_Update(FOC_HandleTypeDef *h, float i_alpha, float i_beta){
    HFITypeDef *s = &h->hfi;
    const float dt = 1.0f / CURRENT_LOOP_FREQUENCY;
    float omega = h->flash_data.hfi.injection_omega, amplitude = h->flash_data.hfi.injection_amplitude;
    if(h->state != FOC_STATE_RUN || !h->flash_data.hfi.hfi_enabled){
        uint8_t lost_selected = h->state == FOC_STATE_RUN && s->feedback_source == FOC_FEEDBACK_HFI;
        if(s->active || s->seeded || s->feedback_source) FOC_HFI_Init(h);
        status(h); return !lost_selected;
    }
    if(h->active_errors || h->latched_errors || h->hdrv8323.high_impedance || !h->adc_calibrated ||
       !isfinite(i_alpha) || !isfinite(i_beta) || !isfinite(amplitude) || amplitude <= 0 ||
       !isfinite(h->flash_data.limits.max_dq_current) || h->flash_data.limits.max_dq_current <= 0 ||
       !isfinite(h->flash_data.limits.max_dq_voltage) || h->flash_data.limits.max_dq_voltage <= 0 ||
       !isfinite(omega) || omega < CONFIG_ABS_HFI_OMEGA_MIN || omega > CONFIG_ABS_HFI_OMEGA_MAX ||
       omega > CURRENT_LOOP_FREQUENCY * M_PIF * 0.5f || h->flash_data.motor.pole_pairs == 0 ||
       !h->encoder_runtime.healthy || !h->flash_data.encoder.offset_valid ||
       !isfinite(h->encoder_angle_electrical) || !isfinite(h->encoder_angle_mechanical_unwrapped) ||
       !isfinite(h->encoder_speed_electrical) || !isfinite(h->adc_values.vbus) ||
       amplitude >= fminf(h->flash_data.limits.max_dq_voltage, h->adc_values.vbus * M_1_SQRT3F)){
        uint8_t selected = s->feedback_source == FOC_FEEDBACK_HFI;
        if(s->active || s->seeded || selected) FOC_HFI_Init(h);
        status(h); return !selected;
    }
    if(hypotf(i_alpha, i_beta) > h->flash_data.limits.max_dq_current * HFI_CURRENT_TRIP_FACTOR){
        s->injection_alpha = s->injection_beta = 0; s->valid = s->locked = 0;
        status(h); return 0;
    }
    const float step = omega * dt;
    if(!s->active){
        s->active = 1; s->injection_phase = -step;
    }else{
        float phase = s->injection_phase + step * 0.5f + HFI_DEMOD_PHASE_CORRECTION;
        float c = cosf(phase), sn = sinf(phase);
        s->i_alpha_l_raw = i_alpha * c - i_beta * sn;
        s->i_beta_l_raw = i_alpha * sn + i_beta * c;
        s->i_alpha_l_filtered = biquad_filter_update(&s->i_alpha_l_filter, s->i_alpha_l_raw);
        s->i_beta_l_filtered = biquad_filter_update(&s->i_beta_l_filter, s->i_beta_l_raw);
        s->positive_alpha = biquad_filter_update(&s->positive_alpha_filter, i_alpha * c + i_beta * sn);
        s->positive_beta = biquad_filter_update(&s->positive_beta_filter, i_beta * c - i_alpha * sn);
        s->signal_amplitude = hypotf(s->i_alpha_l_filtered, s->i_beta_l_filtered);
        float positive = hypotf(s->positive_alpha, s->positive_beta);
        s->saliency_ratio = positive > HFI_MIN_SIGNAL_AMPS ? s->signal_amplitude / positive : 0;
        float axis = 0.5f * atan2f(-HFI_SALIENCY_POLARITY * s->i_alpha_l_filtered,
                                    HFI_SALIENCY_POLARITY * s->i_beta_l_filtered);
        if(axis < 0) axis += M_PIF;
        s->angle_axis_mod_pi = axis; s->samples++;
        uint8_t observable = isfinite(axis) && isfinite(s->saliency_ratio) &&
            s->signal_amplitude >= HFI_MIN_SIGNAL_AMPS && s->saliency_ratio >= HFI_MIN_SALIENCY_RATIO &&
            s->saliency_ratio <= HFI_MAX_SALIENCY_RATIO &&
            fabsf(h->encoder_speed_electrical) <= HFI_MAX_ELECTRICAL_SPEED &&
            s->samples >= (uint32_t)(HFI_SETTLE_SECONDS * CURRENT_LOOP_FREQUENCY);
        const float sign = h->flash_data.motor.direction ? -1.0f : 1.0f;
        const float poles = h->flash_data.motor.pole_pairs;
        if(observable && !s->seeded){
            float reference = sign * h->encoder_angle_electrical;
            s->angle_physical = reference + axial_error(axis - reference);
            normalize_angle_0_2pi(&s->angle_physical);
            float difference = sign * s->angle_physical - h->encoder_angle_electrical;
            normalize_angle_pm_pi(&difference);
            s->angle_mechanical_unwrapped = h->encoder_angle_mechanical_unwrapped + difference / poles;
            s->seeded = 1;
        }
        if(s->seeded){
            float error = axial_error(axis - s->angle_physical);
            float delta = (s->speed_electrical + HFI_PLL_KP * error) * dt;
            s->speed_electrical = fminf(HFI_MAX_ELECTRICAL_SPEED, fmaxf(-HFI_MAX_ELECTRICAL_SPEED,
                                        s->speed_electrical + HFI_PLL_KI * error * dt));
            s->angle_physical += delta; normalize_angle_0_2pi(&s->angle_physical);
            s->angle_electrical = sign * s->angle_physical;
            normalize_angle_0_2pi(&s->angle_electrical);
            s->angle_mechanical_unwrapped += sign * delta / poles;
            s->speed_mechanical = sign * s->speed_electrical / poles;
            s->error_electrical = s->angle_electrical - h->encoder_angle_electrical;
            normalize_angle_pm_pi(&s->error_electrical);
            s->error_mechanical = s->angle_mechanical_unwrapped - h->encoder_angle_mechanical_unwrapped;
            observable = observable && fabsf(error) <= HFI_MAX_TRACKING_ERROR &&
                fabsf(s->error_electrical) <= HFI_MAX_REFERENCE_ERROR &&
                fabsf(s->error_mechanical) * poles <= HFI_MAX_REFERENCE_ERROR;
        }else observable = 0;
        s->good_samples = observable ? s->good_samples + (s->good_samples < CURRENT_LOOP_FREQUENCY) : 0;
        s->locked = s->good_samples >= (uint32_t)(HFI_LOCK_SECONDS * CURRENT_LOOP_FREQUENCY);
        s->valid = observable && s->locked;
        if(s->feedback_source == FOC_FEEDBACK_HFI && !s->valid){
            s->injection_alpha = s->injection_beta = 0; status(h); return 0;
        }
    }
    s->injection_phase += step; normalize_angle_0_2pi(&s->injection_phase);
    s->injection_alpha = amplitude * cosf(s->injection_phase);
    s->injection_beta = amplitude * sinf(s->injection_phase);
    status(h); return 1;
}

/** @brief Selects feedback only in stationary zero-target open-loop RUN.
 * @note Runtime only: STOP/ERROR restores encoder; the encoder remains required.
 */
uint8_t FOC_HFI_SelectFeedback(FOC_HandleTypeDef *h, uint8_t source){
    if(source > FOC_FEEDBACK_HFI || h->state != FOC_STATE_RUN || h->active_errors || h->latched_errors ||
       h->calibration_upload.active || h->flash_data.controller.control_mode != CONTROL_MODE_OPENLOOP ||
       !isfinite(h->encoder_speed_mechanical) || fabsf(h->encoder_speed_mechanical) > 0.1f ||
       !isfinite(h->dq_current_setpoint.d) || !isfinite(h->dq_current_setpoint.q) ||
       fabsf(h->dq_current_setpoint.d) > 0.001f || fabsf(h->dq_current_setpoint.q) > 0.001f) return 0;
    if(source == FOC_FEEDBACK_HFI && (!h->hfi.valid || !h->hfi.locked || !h->hfi.active ||
       !h->flash_data.hfi.hfi_enabled || !h->encoder_runtime.healthy)) return 0;
    h->hfi.feedback_source = source;
    h->flash_data.controller.anticogging_FF_enabled = 0;
    PID_Reset(&h->pid_current_d); PID_Reset(&h->pid_current_q);
    PID_Reset(&h->pid_speed); PID_Reset(&h->pid_position);
    h->angle_setpoint = FOC_FeedbackMechanicalAngle(h); h->speed_setpoint = 0;
    status(h); return 1;
}

/** @brief Returns logical electrical feedback without overwriting encoder data. */
float FOC_FeedbackElectricalAngle(const FOC_HandleTypeDef *h){return h->hfi.feedback_source ? h->hfi.angle_electrical : h->encoder_angle_electrical;}
/** @brief Returns encoder-origin unwrapped mechanical position. */
float FOC_FeedbackMechanicalAngle(const FOC_HandleTypeDef *h){return h->hfi.feedback_source ? h->hfi.angle_mechanical_unwrapped : h->encoder_angle_mechanical_unwrapped;}
/** @brief Returns selected mechanical feedback speed. */
float FOC_FeedbackMechanicalSpeed(const FOC_HandleTypeDef *h){return h->hfi.feedback_source ? h->hfi.speed_mechanical : h->encoder_speed_mechanical;}
