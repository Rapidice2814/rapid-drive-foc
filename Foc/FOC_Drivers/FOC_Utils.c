#include <math.h>
#include <stdio.h>

#include "FOC_Utils.h"
#include "FOC_Handle.h"
#include "Utils.h"
#include "FOC_Config.h"
#include "Cordic.h"


//Sets default values for the FOC structure
FOC_StatusTypeDef FOC_Init(FOC_HandleTypeDef *hfoc){

    FOC_SetState(hfoc, FOC_STATE_INIT, FOC_STATE_NONE);

    hfoc->speed_setpoint = 0.0f;
    hfoc->angle_setpoint = 0.0f;
    hfoc->dq_current_setpoint.q = 0.0f;
    hfoc->dq_current_setpoint.d = 0.0f;

    return FOC_OK;
}

/**
 * @brief Clarke transform
 * @param current Phase currents
 * @retval ABCurrentsTypeDef Alpha and Beta currents
 */
ABCurrentsTypeDef Clarke_transform(PhaseCurrentsTypeDef current){
    ABCurrentsTypeDef result;
    
	float mid = (1.f/3) * (current.a + current.b + current.c);
	float a = current.a - mid;
	float b = current.b - mid;
	result.alpha = a;
	result.beta = M_1_SQRT3F * a + M_2_SQRT3F * b;
    return result;
}

/**
 * @brief Park transform
 * @param ab_current Alpha and Beta currents
 * @param theta Electrical angle in radians
 * @retval DQCurrentsTypeDef D and Q currents
 */
DQCurrentsTypeDef Park_transform(ABCurrentsTypeDef ab_current, float theta){
    DQCurrentsTypeDef result;

    float cos_theta, sin_theta;
    Cordic_CalculateSinCos(theta, &cos_theta, &sin_theta);

	result.d = ab_current.alpha * cos_theta + ab_current.beta * sin_theta;
	result.q = ab_current.beta * cos_theta - ab_current.alpha * sin_theta;

    return result;
}

/**
 * @brief Inverse Park transform
 * @param dq_voltage D and Q voltages
 * @param theta Electrical angle in radians
 * @retval ABVoltagesTypeDef Alpha and Beta voltages
 */
ABVoltagesTypeDef InvPark_transform(DQVoltagesTypeDef dq_voltage, float theta){
    ABVoltagesTypeDef result;

    float cos_theta, sin_theta;
    Cordic_CalculateSinCos(theta, &cos_theta, &sin_theta);

    result.alpha = dq_voltage.d * cos_theta - dq_voltage.q * sin_theta;
    result.beta = dq_voltage.d * sin_theta + dq_voltage.q * cos_theta;

    return result;
}

/**
 * @brief Inverse Clarke transform
 * @param ab_voltage Alpha-beta voltages
 * @retval PhaseVoltagesTypeDef Phase voltages
 */
PhaseVoltagesTypeDef InvClarke_transform(ABVoltagesTypeDef ab_voltage){
    PhaseVoltagesTypeDef result;
    result.a = ab_voltage.alpha;
    result.b = -0.5f * ab_voltage.alpha + M_SQRT3_2F * ab_voltage.beta;
    result.c = -0.5f * ab_voltage.alpha - M_SQRT3_2F * ab_voltage.beta;

    return result;
}

/**
 * @brief Calculates the bus current based on the phase currents and voltages.
 * @param phase_current Phase currents
 * @param phase_voltage Phase voltages
 * @param vbus Bus voltage
 * @retval float Bus current
 */
float CalculateBusCurrent(PhaseCurrentsTypeDef phase_current, PhaseVoltagesTypeDef phase_voltage, float vbus){
    float power = phase_voltage.a * phase_current.a +
                  phase_voltage.b * phase_current.b + 
                  phase_voltage.c * phase_current.c;
    
    float Ibus = power / vbus;
    return Ibus;
}

/**
  * @brief Sets the phase voltages to the inverter.
  * @param hfoc Handle to the FOC structure
  * @param phase_voltage Phase voltages to set
  * @retval FOC_StatusTypeDef
  * @note The maximum phase voltages should not exceed the vbus*sqrt(3), otherwise the output will be clipped.
  */
FOC_StatusTypeDef FOC_SetPhaseVoltages(FOC_HandleTypeDef *hfoc, PhaseVoltagesTypeDef phase_voltage){

    hfoc->phase_voltage = phase_voltage;
    
	float Umin = fminf(phase_voltage.a, fminf(phase_voltage.b, phase_voltage.c));
    float Umax = fmaxf(phase_voltage.a, fmaxf(phase_voltage.b, phase_voltage.c));

    float center = hfoc->adc_values.vbus / 2.0f;
    center -= (Umax+Umin) / 2.0f;

    float duty_a = constrainf((phase_voltage.a + center) / hfoc->adc_values.vbus, 0.0f, 1.0f);
    float duty_b = constrainf((phase_voltage.b + center) / hfoc->adc_values.vbus, 0.0f, 1.0f);
    float duty_c = constrainf((phase_voltage.c + center) / hfoc->adc_values.vbus, 0.0f, 1.0f);

    *(hfoc->pCCRa) = (uint32_t)(duty_a * (float)hfoc->max_ccr);
    *(hfoc->pCCRb) = (uint32_t)(duty_b * (float)hfoc->max_ccr);
    *(hfoc->pCCRc) = (uint32_t)(duty_c * (float)hfoc->max_ccr);

    return FOC_OK;
}


/**
  * @brief Sets the PWM CCR pointers and max CCR value
  * @param hfoc Handle to the FOC structure
  * @param pCCRa Pointer to the CCR register for phase A
  * @param pCCRb Pointer to the CCR register for phase B
  * @param pCCRc Pointer to the CCR register for phase C
  * @param max_ccr Maximum CCR value
  * @retval FOC_StatusTypeDef
  */
FOC_StatusTypeDef FOC_SetPWMCCRPointers(FOC_HandleTypeDef *hfoc, volatile uint32_t *pCCRa, volatile uint32_t *pCCRb, volatile uint32_t *pCCRc, uint32_t max_ccr){
	hfoc->max_ccr = max_ccr;
	hfoc->pCCRa = pCCRa;
	hfoc->pCCRb = pCCRb;
	hfoc->pCCRc = pCCRc;

	// Set the PWM duty cycles to 0
	*(hfoc->pCCRa) = 0; 
	*(hfoc->pCCRb) = 0;
	*(hfoc->pCCRc) = 0;

    return FOC_OK;
}

/**
 * @brief Sets the encoder pointer to the FOC structure.
 * @param hfoc Handle to the FOC structure
 * @param pencoder_count Pointer to the encoder counter
 * @retval FOC_StatusTypeDef
 * @note This function also initializes the encoder angle based on the current encoder position.
 */
FOC_StatusTypeDef FOC_SetEncoderPointer(FOC_HandleTypeDef *hfoc, volatile uint32_t *pencoder_count){
    hfoc->pencoder_count = pencoder_count;

    if(hfoc->has5047p.setup_complete != 1) return FOC_ERROR;
    uint16_t ap5047p_angle = 0;
    AS5047P_GetAngle(&(hfoc->has5047p), &ap5047p_angle);

    *(hfoc->pencoder_count) = (int32_t)(((float)ap5047p_angle / 16384.0f)  * ENCODER_PULSES_PER_ROTATION); //set absolute angle

    return FOC_OK;
}

/**
  * @brief Sets the mechanical encoder zero position to the current position.
  * @param Handle to the FOC structure
  * @retval FOC_StatusTypeDef
  */
FOC_StatusTypeDef FOC_SetEncoderZero(FOC_HandleTypeDef *hfoc){

    hfoc->flash_data.encoder.offset_valid = 0;

    if(hfoc->has5047p.setup_complete != 1) return FOC_ERROR;

    uint16_t diaagc = 0;
    AS5047P_GetDIAAGC(&(hfoc->has5047p), &diaagc);
    uint16_t errors = 0xe00 & diaagc;
    if(errors != 0) return FOC_ERROR; //check for errors
    // uint8_t agc = (0xff & diaagc);


    uint16_t ap5047p_angle = 0;
    AS5047P_GetAngle(&(hfoc->has5047p), &ap5047p_angle);

    hfoc->flash_data.encoder.mechanical_offset = ((float)ap5047p_angle / 16384.0f) * 2.0f * M_PIF;
    hfoc->flash_data.encoder.offset_valid = 1;
    // *(hfoc->pencoder_count) = (int32_t)(hfoc->flash_data.encoder.mechanical_offset * ENCODER_PULSES_PER_ROTATION / (2 * M_PIF));

    return FOC_OK;
}

/**
  * @brief Updates the encoder angle and speed in the FOC structure. This function should be called at a fixed frequency.
  * @param Handle to the FOC structure, delta time, filter alpha
  * @param frequency: frequency at which this function is called, used to calculate the speed from the change in angle. Should be in Hz.
  * @retval FOC_StatusTypeDef
  */
FOC_StatusTypeDef FOC_UpdateEncoder(FOC_HandleTypeDef *hfoc, float frequency){
    float raw_angle_mechanical = (((float)(*(hfoc->pencoder_count)) / ENCODER_PULSES_PER_ROTATION) * 2 * M_PIF) - hfoc->flash_data.encoder.mechanical_offset;
    normalize_angle_0_2pi(&raw_angle_mechanical);

    hfoc->encoder_angle_mechanical_wrapped = raw_angle_mechanical;
    float delta = raw_angle_mechanical - hfoc->encoder_angle_mechanical_wrapped_prev;
    normalize_angle_pm_pi(&delta);

    hfoc->encoder_angle_mechanical_unwrapped += delta;
    hfoc->encoder_angle_mechanical_wrapped_prev = raw_angle_mechanical;

    // uint16_t as5047p_angle = 0;
    // AS5047P_GetAngle(&hfoc->has5047p, &as5047p_angle);
    // hfoc->encoder_angle_mechanical_wrapped = (((float)as5047p_angle / 16384.0f) * 2.0f * M_PIF) - hfoc->flash_data.encoder.mechanical_offset;

    
    float raw_angle_electrical = hfoc->encoder_angle_mechanical_wrapped * (float)(hfoc->flash_data.motor.pole_pairs);
    normalize_angle_0_2pi(&raw_angle_electrical);

    hfoc->encoder_angle_electrical = raw_angle_electrical;
    
    hfoc->encoder_speed_mechanical = hfoc->encoder_speed_mechanical * (1 - SPEED_MEASUREMENT_ALPHA) + SPEED_MEASUREMENT_ALPHA * delta * frequency;
    hfoc->encoder_speed_electrical = hfoc->encoder_speed_mechanical * (float)(hfoc->flash_data.motor.pole_pairs);

    return FOC_OK;
}







