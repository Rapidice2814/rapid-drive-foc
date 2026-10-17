#ifndef FOC_HFI_H
#define FOC_HFI_H

#include "Utils.h"
#include "Filters.h"
#include "PID.h"
#include "FOC_Utils.h"

typedef struct FOC_Handle FOC_HandleTypeDef;

typedef enum {
    HFI_MODE_DISABLED = 0,  // no injection
    HFI_MODE_OBSERVER,      // inject and estimate the angle, but control with the encoder angle. Used to validate and tune HFI
    HFI_MODE_SENSORLESS     // control with the HFI angle and speed
} HFIModeTypeDef;

typedef enum {
    HFI_STATE_RESET = 0,        // filters and PLL need to be (re)initialized
    HFI_STATE_CONVERGING,       // PLL is locking onto the saliency, the angle is still ambiguous by pi
    HFI_STATE_POLARITY_POSITIVE,// positive d-axis current pulse along the estimated angle
    HFI_STATE_POLARITY_NEGATIVE,// negative d-axis current pulse along the estimated angle
    HFI_STATE_LOCKED,           // angle and polarity are known
    HFI_STATE_INVALID_CONFIG    // injection frequency or amplitude is not usable, nothing is injected
} HFIStateTypeDef;

typedef struct {
    HFIStateTypeDef state;
    uint32_t state_counter;         // number of current loop cycles spent in the current state

    uint8_t configured_mode;        // mode the filters were set up for, a change triggers a reset
    float configured_omega;         // injection frequency the filters were set up for, a change triggers a reset
    float demodulation_delay;       // [s], delay between injecting the carrier and demodulating its current (band-pass + PWM/ADC)

    /* Injection */
    float injection_phase;          // [rad], phase of the carrier
    float injection_voltage;        // [V], carrier voltage applied along the estimated d-axis this cycle

    /* Demodulation */
    ABCurrentsTypeDef ab_current_hf;// [A], carrier component of the alpha/beta currents
    float id_hf;                    // [A], carrier current along the estimated d-axis
    float iq_hf;                    // [A], carrier current along the estimated q-axis
    float id_iq_filtered;           // [A^2], low-passed id_hf * iq_hf
    float id_id_filtered;           // [A^2], low-passed id_hf * id_hf, carrier power along the estimated d-axis
    float angle_error;              // [-], normalized saliency error, ~(Lq-Ld)/Lq * (true angle - estimated angle)

    /* Observer output */
    float angle;                    // [rad], estimated electrical angle in the phase frame, 0..2pi
    float speed;                    // [rad/s], estimated electrical speed in the phase frame
    float encoder_angle_error;      // [rad], estimated angle - encoder angle, for validation

    /* Polarity detection */
    float bias_current;             // [A], d-axis current added along the estimated angle during polarity detection
    float polarity_power_positive;  // [A^2], average carrier power during the positive pulse
    float polarity_power_negative;  // [A^2], average carrier power during the negative pulse

    float max_speed;                // [rad/s], PLL output limit
    PIDControllerTypeDef pll;

    BiquadFilter i_alpha_bpf;
    BiquadFilter i_beta_bpf;
    BiquadFilter id_iq_lpf;
    BiquadFilter id_id_lpf;
} HFITypeDef;

void FOC_HFI_Init(FOC_HandleTypeDef *hfoc);
void FOC_HFI_Reset(FOC_HandleTypeDef *hfoc);
uint8_t FOC_HFI_IsActive(FOC_HandleTypeDef *hfoc);
uint8_t FOC_HFI_IsLocked(FOC_HandleTypeDef *hfoc);
ABCurrentsTypeDef FOC_HFI_Update(FOC_HandleTypeDef *hfoc, ABCurrentsTypeDef ab_current, float encoder_angle_electrical);
void FOC_HFI_ApplyBiasCurrent(FOC_HandleTypeDef *hfoc, float control_angle, float *setpoint_d, float *setpoint_q);
void FOC_HFI_AddInjectionVoltage(FOC_HandleTypeDef *hfoc, ABVoltagesTypeDef *ab_voltage);

#endif /* FOC_HFI_H */
