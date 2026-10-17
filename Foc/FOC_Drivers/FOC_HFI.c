#include "FOC_HFI.h"
#include "FOC_Handle.h"
#include "FOC_Config.h"
#include "Cordic.h"
#include "main.h"

/*
 * High Frequency Injection (pulsating carrier in the estimated d-axis)
 *
 * A carrier voltage V*cos(wt) is injected along the estimated d-axis. In the estimated frame the high frequency currents are
 *      id_hf ~ ((Ld+Lq)/2 - (Ld-Lq)/2 * cos(2*err)) * g(t)
 *      iq_hf ~ (          - (Ld-Lq)/2 * sin(2*err)) * g(t)
 * with err = true angle - estimated angle and g(t) the (delayed) integral of the carrier. Both share the same waveform, so
 * LPF(id_hf*iq_hf) / LPF(id_hf*id_hf) gives a signal ~ (Lq-Ld)/Lq * err for small errors, independent of the carrier
 * amplitude, frequency, phase delay and average inductance. A PI-PLL drives this signal to zero.
 *
 * The saliency is pi periodic, so the PLL can lock on the south pole. After converging, two d-axis current pulses are applied
 * along the estimate. The pulse along the magnet flux saturates the iron, lowers Ld and gives the larger carrier current.
 * If the negative pulse wins, the estimate is flipped by pi.
 *
 * The carrier current is demodulated after the band-pass filter and the PWM/ADC delay. While rotating, the current then belongs
 * to an older estimate, so it is demodulated at angle - speed * delay. Without this, the angle lags by ~speed*delay/saliency.
 */

#define HFI_CONVERGE_CYCLES ((uint32_t)(HFI_CONVERGE_TIME * CURRENT_LOOP_FREQUENCY))
#define HFI_POLARITY_PULSE_CYCLES ((uint32_t)(HFI_POLARITY_PULSE_TIME * CURRENT_LOOP_FREQUENCY))
#define HFI_POLARITY_MEASUREMENT_CYCLES (HFI_POLARITY_PULSE_CYCLES - HFI_POLARITY_PULSE_CYCLES / 2)

static void FOC_HFI_SetState(HFITypeDef *hfi, HFIStateTypeDef state){
    hfi->state = state;
    hfi->state_counter = 0;
}

/**
 * @brief Initializes the HFI observer. Needs to be called after the flash data is loaded.
 * @param hfoc Handle to the FOC structure
 * @retval None
 */
void FOC_HFI_Init(FOC_HandleTypeDef *hfoc){
    hfoc->hfi.max_speed = HFI_MAX_ELECTRICAL_SPEED;
    PID_Init(&hfoc->hfi.pll, (1.0f/CURRENT_LOOP_FREQUENCY), 0.01f, &hfoc->hfi.max_speed, &hfoc->flash_data.hfi.PLL_gains, 0);

    FOC_HFI_Reset(hfoc);
}

/**
 * @brief Resets the HFI observer and sets up the filters for the configured injection frequency. The angle estimate starts at 0.
 * @param hfoc Handle to the FOC structure
 * @retval None
 */
void FOC_HFI_Reset(FOC_HandleTypeDef *hfoc){
    HFITypeDef *hfi = &hfoc->hfi;
    const float injection_frequency = hfoc->flash_data.hfi.injection_omega / M_2PIF;

    hfi->configured_mode = hfoc->flash_data.hfi.mode;
    hfi->configured_omega = hfoc->flash_data.hfi.injection_omega;

    hfi->injection_phase = 0.0f;
    hfi->injection_voltage = 0.0f;
    hfi->ab_current_hf = (ABCurrentsTypeDef){0.0f, 0.0f};
    hfi->id_hf = 0.0f;
    hfi->iq_hf = 0.0f;
    hfi->id_iq_filtered = 0.0f;
    hfi->id_id_filtered = 0.0f;
    hfi->angle_error = 0.0f;
    hfi->angle = 0.0f;
    hfi->speed = 0.0f;
    hfi->encoder_angle_error = 0.0f;
    hfi->bias_current = 0.0f;
    hfi->polarity_power_positive = 0.0f;
    hfi->polarity_power_negative = 0.0f;
    PID_Reset(&hfi->pll);

    // the carrier needs at least 4 samples per period, and the 2x carrier ripple after demodulation has to be well above the LPF
    if(!(injection_frequency <= 0.25f * CURRENT_LOOP_FREQUENCY && injection_frequency >= 4.0f * HFI_CURRENT_LPF_CUTOFF_FREQ)){
        FOC_HFI_SetState(hfi, HFI_STATE_INVALID_CONFIG);
        return;
    }

    hfi->demodulation_delay = 2.0f * HFI_BANDPASS_Q / hfoc->flash_data.hfi.injection_omega + 1.0f / CURRENT_LOOP_FREQUENCY; // band-pass group delay + one PWM period

    biquad_filter_init(&hfi->i_alpha_bpf, BIQUAD_BAND_PASS, injection_frequency, CURRENT_LOOP_FREQUENCY, HFI_BANDPASS_Q);
    biquad_filter_init(&hfi->i_beta_bpf, BIQUAD_BAND_PASS, injection_frequency, CURRENT_LOOP_FREQUENCY, HFI_BANDPASS_Q);
    biquad_filter_init(&hfi->id_iq_lpf, BIQUAD_LOW_PASS, HFI_CURRENT_LPF_CUTOFF_FREQ, CURRENT_LOOP_FREQUENCY, 0.707f);
    biquad_filter_init(&hfi->id_id_lpf, BIQUAD_LOW_PASS, HFI_CURRENT_LPF_CUTOFF_FREQ, CURRENT_LOOP_FREQUENCY, 0.707f);

    FOC_HFI_SetState(hfi, HFI_STATE_CONVERGING);
}

/**
 * @brief Returns 1 if HFI is enabled in the configuration
 * @param hfoc Handle to the FOC structure
 */
uint8_t FOC_HFI_IsActive(FOC_HandleTypeDef *hfoc){
    return hfoc->flash_data.hfi.mode == HFI_MODE_OBSERVER || hfoc->flash_data.hfi.mode == HFI_MODE_SENSORLESS;
}

/**
 * @brief Returns 1 if the HFI angle estimate is valid, including the magnet polarity
 * @param hfoc Handle to the FOC structure
 */
uint8_t FOC_HFI_IsLocked(FOC_HandleTypeDef *hfoc){
    return hfoc->hfi.state == HFI_STATE_LOCKED;
}

/**
 * @brief Runs the HFI observer. Needs to be called every current loop cycle while HFI is active.
 * @param hfoc Handle to the FOC structure
 * @param ab_current Measured alpha/beta currents
 * @param encoder_angle_electrical Encoder electrical angle in the phase frame, only used for validation
 * @retval ABCurrentsTypeDef The alpha/beta currents without the carrier, to be used by the current controller
 */
ABCurrentsTypeDef FOC_HFI_Update(FOC_HandleTypeDef *hfoc, ABCurrentsTypeDef ab_current, float encoder_angle_electrical){
    HFITypeDef *hfi = &hfoc->hfi;

    if(hfi->state == HFI_STATE_RESET ||
        hfi->configured_mode != hfoc->flash_data.hfi.mode ||
        hfi->configured_omega != hfoc->flash_data.hfi.injection_omega){
        FOC_HFI_Reset(hfoc);
    }

    if(hfi->state == HFI_STATE_INVALID_CONFIG){
        hfi->injection_voltage = 0.0f;
        hfi->bias_current = 0.0f;
        return ab_current;
    }

    /* Split the measured current into the carrier response and the fundamental current */
    hfi->ab_current_hf.alpha = biquad_filter_update(&hfi->i_alpha_bpf, ab_current.alpha);
    hfi->ab_current_hf.beta = biquad_filter_update(&hfi->i_beta_bpf, ab_current.beta);

    ABCurrentsTypeDef ab_current_fundamental = {
        .alpha = ab_current.alpha - hfi->ab_current_hf.alpha,
        .beta = ab_current.beta - hfi->ab_current_hf.beta
    };

    /* Demodulate in the estimated frame the carrier was injected in, using the d-axis carrier current as the reference */
    DQCurrentsTypeDef dq_current_hf = Park_transform(hfi->ab_current_hf, hfi->angle - hfi->speed * hfi->demodulation_delay);
    hfi->id_hf = dq_current_hf.d;
    hfi->iq_hf = dq_current_hf.q;

    hfi->id_iq_filtered = biquad_filter_update(&hfi->id_iq_lpf, hfi->id_hf * hfi->iq_hf);
    hfi->id_id_filtered = biquad_filter_update(&hfi->id_id_lpf, hfi->id_hf * hfi->id_hf);

    if(hfi->id_id_filtered > HFI_MIN_CARRIER_POWER){
        hfi->angle_error = constrainf(hfi->id_iq_filtered / hfi->id_id_filtered, -1.0f, 1.0f);
    } else {
        hfi->angle_error = 0.0f;
    }

    /* PLL: the proportional part corrects the angle, the integrator tracks the speed */
    float pll_output = PID_Update(&hfi->pll, hfi->angle_error, 0.0f);
    hfi->speed = hfi->pll.integrator;
    hfi->angle += pll_output / CURRENT_LOOP_FREQUENCY;
    normalize_angle_0_2pi(&hfi->angle);

    hfi->encoder_angle_error = hfi->angle - encoder_angle_electrical;
    normalize_angle_pm_pi(&hfi->encoder_angle_error);

    /* Convergence and polarity detection */
    hfi->state_counter++;
    switch(hfi->state){
        case HFI_STATE_CONVERGING:
            if(hfi->state_counter >= HFI_CONVERGE_CYCLES){
                if(hfoc->flash_data.hfi.polarity_current > 0.0f){
                    hfi->polarity_power_positive = 0.0f;
                    hfi->polarity_power_negative = 0.0f;
                    FOC_HFI_SetState(hfi, HFI_STATE_POLARITY_POSITIVE);
                } else {
                    FOC_HFI_SetState(hfi, HFI_STATE_LOCKED);
                }
            }
            break;

        case HFI_STATE_POLARITY_POSITIVE:
            hfi->bias_current = hfoc->flash_data.hfi.polarity_current;
            if(hfi->state_counter > HFI_POLARITY_PULSE_CYCLES / 2){ //skip the current step and the filter settling
                hfi->polarity_power_positive += hfi->id_id_filtered / (float)HFI_POLARITY_MEASUREMENT_CYCLES;
            }
            if(hfi->state_counter >= HFI_POLARITY_PULSE_CYCLES){
                FOC_HFI_SetState(hfi, HFI_STATE_POLARITY_NEGATIVE);
            }
            break;

        case HFI_STATE_POLARITY_NEGATIVE:
            hfi->bias_current = -hfoc->flash_data.hfi.polarity_current;
            if(hfi->state_counter > HFI_POLARITY_PULSE_CYCLES / 2){
                hfi->polarity_power_negative += hfi->id_id_filtered / (float)HFI_POLARITY_MEASUREMENT_CYCLES;
            }
            if(hfi->state_counter >= HFI_POLARITY_PULSE_CYCLES){
                hfi->bias_current = 0.0f;
                if(hfi->polarity_power_negative > hfi->polarity_power_positive){ //the estimate points to the south pole
                    hfi->angle += M_PIF;
                    normalize_angle_0_2pi(&hfi->angle);
                }
                FOC_HFI_SetState(hfi, HFI_STATE_LOCKED);
            }
            break;

        default:
            break;
    }

    /* Next carrier sample */
    hfi->injection_phase += hfoc->flash_data.hfi.injection_omega / CURRENT_LOOP_FREQUENCY;
    normalize_angle_0_2pi(&hfi->injection_phase);
    float cos_phase, sin_phase;
    Cordic_CalculateSinCos(hfi->injection_phase, &cos_phase, &sin_phase);
    hfi->injection_voltage = hfoc->flash_data.hfi.injection_amplitude * cos_phase;

    return ab_current_fundamental;
}

/**
 * @brief Adds the polarity detection current to the current setpoints. The current is applied along the estimated angle.
 * @param hfoc Handle to the FOC structure
 * @param control_angle Electrical angle used by the current controller, in the phase frame
 * @param setpoint_d Pointer to the d-axis current setpoint, in the phase frame
 * @param setpoint_q Pointer to the q-axis current setpoint, in the phase frame
 * @retval None
 */
void FOC_HFI_ApplyBiasCurrent(FOC_HandleTypeDef *hfoc, float control_angle, float *setpoint_d, float *setpoint_q){
    if(hfoc->hfi.bias_current == 0.0f) return;

    float cos_delta, sin_delta;
    Cordic_CalculateSinCos(hfoc->hfi.angle - control_angle, &cos_delta, &sin_delta);

    *setpoint_d += hfoc->hfi.bias_current * cos_delta;
    *setpoint_q += hfoc->hfi.bias_current * sin_delta;
}

/**
 * @brief Adds the carrier voltage along the estimated d-axis to the alpha/beta voltages
 * @param hfoc Handle to the FOC structure
 * @param ab_voltage Pointer to the alpha/beta voltages
 * @retval None
 */
void FOC_HFI_AddInjectionVoltage(FOC_HandleTypeDef *hfoc, ABVoltagesTypeDef *ab_voltage){
    if(hfoc->hfi.injection_voltage == 0.0f) return;

    float cos_angle, sin_angle;
    Cordic_CalculateSinCos(hfoc->hfi.angle, &cos_angle, &sin_angle);

    ab_voltage->alpha += hfoc->hfi.injection_voltage * cos_angle;
    ab_voltage->beta += hfoc->hfi.injection_voltage * sin_angle;
}
