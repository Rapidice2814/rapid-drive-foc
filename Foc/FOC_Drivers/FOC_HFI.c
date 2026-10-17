#include "FOC_HFI.h"
#include "FOC_Handle.h"
#include "FOC_Config.h"
#include "Cordic.h"
#include "main.h"

#include <math.h>

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
 * The saliency is pi periodic, so the PLL can lock on the south pole. After converging, short +/- d-axis voltage pulses (one
 * current loop cycle each) are applied along the estimate, with the current controller bypassed. The pulse along the magnet flux
 * saturates the iron, lowers Ld and gives the faster current rise. If the negative pulses win, the estimate is flipped by pi.
 * Current pulses long enough for the current controller would pull a light rotor towards the current before the measurement.
 *
 * With ~0A fundamental current, every phase current crosses zero each carrier period and the inverter dead time distorts the
 * carrier depending on the current direction. On low saliency (surface magnet) motors this fake, stationary saliency is as large
 * as the real one. A DC d-axis bias current along the estimate keeps the currents away from zero and fixes this.
 * A d-axis current against the magnet is an unstable equilibrium: the rotor flips towards the current, and if the estimate follows
 * the rotor the motor runs away. So while the polarity is unknown the bias alternates its sign. After the polarity detection the
 * PLL gets a short settling time with the alternating bias, and only then the bias is applied steadily, along the magnet.
 *
 * The carrier current is demodulated after the band-pass filter and the PWM/ADC delay. While rotating, the current then belongs
 * to an older estimate, so it is demodulated at angle - speed * delay. Without this, the angle lags by ~speed*delay/saliency.
 */

#define HFI_CONVERGE_CYCLES ((uint32_t)(HFI_CONVERGE_TIME * CURRENT_LOOP_FREQUENCY))
#define HFI_SETTLE_CYCLES ((uint32_t)(HFI_SETTLE_TIME * CURRENT_LOOP_FREQUENCY))
#define HFI_BIAS_ALTERNATE_CYCLES ((uint32_t)(HFI_BIAS_ALTERNATE_TIME * CURRENT_LOOP_FREQUENCY))

static void FOC_HFI_SetState(HFITypeDef *hfi, HFIStateTypeDef state){
    hfi->state = state;
    hfi->state_counter = 0;
}

/**
 * @brief Voltage of the polarity pulses, so the current averaged over one pulse (as the ADC sees it) reaches polarity_current
 * @param hfoc Handle to the FOC structure
 * @retval float [V]
 */
static float FOC_HFI_PolarityPulseVoltage(FOC_HandleTypeDef *hfoc){
    const float R = hfoc->flash_data.motor.phase_resistance;
    const float L = hfoc->flash_data.motor.phase_inductance;
    const float T = 1.0f / CURRENT_LOOP_FREQUENCY;
    float voltage = HFI_POLARITY_DEFAULT_VOLTAGE;

    if(R > 0.0f && L > 0.0f){
        const float tau = L / R;
        const float average_fraction = 1.0f - tau / T * (1.0f - expf(-T / tau)); // average of the RL step response over one period, relative to V/R
        voltage = hfoc->flash_data.hfi.polarity_current * R / average_fraction;
    }
    return fminf(voltage, hfoc->flash_data.limits.max_dq_voltage);
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
    hfi->polarity_pulse_voltage = 0.0f;
    hfi->polarity_response_positive = 0.0f;
    hfi->polarity_response_negative = 0.0f;
    hfi->polarity_pulse_pairs = 0;
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
 * @brief Returns 1 while the polarity voltage pulses run, the current controller output has to be zero then
 * @param hfoc Handle to the FOC structure
 */
uint8_t FOC_HFI_OverridesCurrentControl(FOC_HandleTypeDef *hfoc){
    return hfoc->hfi.state == HFI_STATE_POLARITY_POSITIVE || hfoc->hfi.state == HFI_STATE_POLARITY_NEGATIVE;
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

    /* PLL: the proportional part corrects the angle, the integrator tracks the speed. Frozen during the polarity pulses,
       the carrier is off then */
    if(hfi->state != HFI_STATE_POLARITY_POSITIVE && hfi->state != HFI_STATE_POLARITY_NEGATIVE){
        float pll_output = PID_Update(&hfi->pll, hfi->angle_error, 0.0f);
        hfi->speed = hfi->pll.integrator;
        hfi->angle += pll_output / CURRENT_LOOP_FREQUENCY;
        normalize_angle_0_2pi(&hfi->angle);
    }

    hfi->encoder_angle_error = hfi->angle - encoder_angle_electrical;
    normalize_angle_pm_pi(&hfi->encoder_angle_error);

    /* Convergence and polarity detection */
    hfi->state_counter++;
    hfi->bias_current = hfoc->flash_data.hfi.bias_current;
    float pulse_voltage = 0.0f;
    switch(hfi->state){
        case HFI_STATE_CONVERGING:
            if((hfi->state_counter / HFI_BIAS_ALTERNATE_CYCLES) & 1U){ //polarity unknown, so no net torque from the bias
                hfi->bias_current = -hfi->bias_current;
            }
            if(hfi->state_counter >= HFI_CONVERGE_CYCLES){
                if(hfoc->flash_data.hfi.polarity_current > 0.0f){
                    hfi->polarity_pulse_voltage = FOC_HFI_PolarityPulseVoltage(hfoc);
                    hfi->polarity_response_positive = 0.0f;
                    hfi->polarity_response_negative = 0.0f;
                    hfi->polarity_pulse_pairs = 0;
                    FOC_HFI_SetState(hfi, HFI_STATE_POLARITY_POSITIVE);
                } else {
                    FOC_HFI_SetState(hfi, HFI_STATE_LOCKED);
                }
            }
            break;

        case HFI_STATE_POLARITY_POSITIVE:
        case HFI_STATE_POLARITY_NEGATIVE: {
            const uint8_t positive = (hfi->state == HFI_STATE_POLARITY_POSITIVE);
            hfi->bias_current = 0.0f; // the current controller is bypassed
            if(hfi->state_counter == 1){
                pulse_voltage = positive ? hfi->polarity_pulse_voltage : -hfi->polarity_pulse_voltage;
            } else if(hfi->state_counter <= 4){ // the response shows up in the next cycles (PWM update and ADC averaging)
                const float id = Park_transform(ab_current, hfi->angle).d;
                if(positive){
                    hfi->polarity_response_positive += id;
                } else {
                    hfi->polarity_response_negative -= id;
                }
            }
            if(hfi->state_counter >= HFI_POLARITY_PULSE_SPACING){
                if(positive){
                    FOC_HFI_SetState(hfi, HFI_STATE_POLARITY_NEGATIVE);
                } else if(++hfi->polarity_pulse_pairs < HFI_POLARITY_PULSE_PAIRS){
                    FOC_HFI_SetState(hfi, HFI_STATE_POLARITY_POSITIVE);
                } else {
                    if(hfi->polarity_response_negative > hfi->polarity_response_positive){ //the estimate points to the south pole
                        hfi->angle += M_PIF;
                        normalize_angle_0_2pi(&hfi->angle);
                    }
                    FOC_HFI_SetState(hfi, HFI_STATE_SETTLING);
                }
            }
            break;
        }

        case HFI_STATE_SETTLING:
            if((hfi->state_counter / HFI_BIAS_ALTERNATE_CYCLES) & 1U){
                hfi->bias_current = -hfi->bias_current;
            }
            if(hfi->state_counter >= HFI_SETTLE_CYCLES){
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
    if(FOC_HFI_OverridesCurrentControl(hfoc)){ // no carrier during the polarity pulses
        hfi->injection_voltage = pulse_voltage;
    }

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
