#include <math.h>
#include <stdio.h>

#include "FOC_Driver.h"
#include "FOC_Utils.h"

/* uncomment these to use the cos and sin optimizations */
#define cosf _cosf
#define sinf _sinf

/* General */
FOC_StatusTypeDef FOC_SetInputVoltage(FOC_HandleTypeDef *hfoc, float vbus){
	hfoc->vbus = vbus;
    return FOC_OK;
}
FOC_StatusTypeDef FOC_SetVoltageLimit(FOC_HandleTypeDef *hfoc, float voltage_limit){
    hfoc->voltage_limit = voltage_limit;
    return FOC_OK;
}

/* Calculations */
ABCurrentsTypeDef FOC_Clarke_transform(PhaseCurrentsTypeDef current){
    ABCurrentsTypeDef result;
    
	float mid = (1.f/3) * (current.a + current.b + current.c);
	float a = current.a - mid;
	float b = current.b - mid;
	result.alpha = a;
	result.beta = M_1_SQRT3F * a + M_2_SQRT3F * b;
    return result;
}

DQCurrentsTypeDef FOC_Park_transform(ABCurrentsTypeDef ab_current, float theta){
    DQCurrentsTypeDef result;

	float cos_theta = cosf(theta);
	float sin_theta = sinf(theta);
	result.d = ab_current.alpha * cos_theta + ab_current.beta * sin_theta;
	result.q = ab_current.beta * cos_theta - ab_current.alpha * sin_theta;

    return result;
}

ABVoltagesTypeDef FOC_InvPark_transform(DQVoltagesTypeDef dq_voltage, float theta){
    ABVoltagesTypeDef result;

	float cos_theta = cosf(theta);
	float sin_theta = sinf(theta);
    result.alpha = dq_voltage.d * cos_theta - dq_voltage.q * sin_theta;
    result.beta = dq_voltage.d * sin_theta + dq_voltage.q * cos_theta;

    return result;
}

PhaseVoltagesTypeDef FOC_InvClarke_transform(ABVoltagesTypeDef ab_voltage){
    PhaseVoltagesTypeDef result;
    result.a = ab_voltage.alpha;
    result.b = -0.5f * ab_voltage.alpha + M_SQRT3_2F * ab_voltage.beta;
    result.c = -0.5f * ab_voltage.alpha - M_SQRT3_2F * ab_voltage.beta;

    return result;
}

FOC_StatusTypeDef FOC_SetPhaseVoltages(FOC_HandleTypeDef *hfoc, PhaseVoltagesTypeDef phase_voltages){
    
	float Umin = fminf(phase_voltages.a, fminf(phase_voltages.b, phase_voltages.c));
    float Umax = fmaxf(phase_voltages.a, fmaxf(phase_voltages.b, phase_voltages.c));

    float center = hfoc->voltage_limit / 2.0f;
    center -= (Umax+Umin) / 2;

    float PWMa = (((phase_voltages.a + center) / hfoc->vbus) * hfoc->max_ccr);
    float PWMb = (((phase_voltages.b + center) / hfoc->vbus) * hfoc->max_ccr);
    float PWMc = (((phase_voltages.c + center) / hfoc->vbus) * hfoc->max_ccr);

    *(hfoc->pCCRa) = (uint32_t)PWMa;
    *(hfoc->pCCRb) = (uint32_t)PWMb;
    *(hfoc->pCCRc) = (uint32_t)PWMc;

    return FOC_OK;
}


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



/* Encoder */
FOC_StatusTypeDef FOC_SetEncoderPointer(FOC_HandleTypeDef *hfoc, volatile uint32_t *pencoder_count){
    hfoc->pencoder_count = pencoder_count;

    if(hfoc->has5047p.setup_complete != 1) return FOC_ERROR;
    uint16_t ap5047p_angle = 0;
    AS5047P_GetAngle(&(hfoc->has5047p), &ap5047p_angle);

    *(hfoc->pencoder_count) = (int32_t)(((float)ap5047p_angle / 16384.0f)  * ENCODER_PULSES_PER_ROTATION); //set absolute angle

    return FOC_OK;
}


/**
  * @brief Sets the encoder zero position to the current position.
  * @param Handle to the FOC structure
  * @retval FOC_StatusTypeDef
  */
FOC_StatusTypeDef FOC_SetEncoderZero(FOC_HandleTypeDef *hfoc){

    hfoc->flash_data.encoder_aligned_flag = 0;

    if(hfoc->has5047p.setup_complete != 1) return FOC_ERROR;

    uint16_t diaagc = 0;
    AS5047P_GetDIAAGC(&(hfoc->has5047p), &diaagc);
    uint16_t errors = 0xe00 & diaagc;
    if(errors != 0) return FOC_ERROR; //check for errors
    // uint8_t agc = (0xff & diaagc);


    uint16_t ap5047p_angle = 0;
    AS5047P_GetAngle(&(hfoc->has5047p), &ap5047p_angle);

    hfoc->flash_data.encoder_angle_mechanical_offset = ((float)ap5047p_angle / 16384.0f) * 2.0f * M_PIF;
    hfoc->flash_data.encoder_aligned_flag = 1; //set the encoder aligned flag
    // *(hfoc->pencoder_count) = (int32_t)(hfoc->flash_data.encoder_angle_mechanical_offset * ENCODER_PULSES_PER_ROTATION / (2 * M_PIF));

    return FOC_OK;
}


/**
  * @brief Updates the encoder angle
  * @param Handle to the FOC structure
  * @retval FOC_StatusTypeDef
  */
FOC_StatusTypeDef FOC_UpdateEncoderAngle(FOC_HandleTypeDef *hfoc){
    hfoc->encoder_angle_mechanical = (((float)(*(hfoc->pencoder_count)) / ENCODER_PULSES_PER_ROTATION) * 2 * M_PIF) - hfoc->flash_data.encoder_angle_mechanical_offset;

    // uint16_t as5047p_angle = 0;
    // AS5047P_GetAngle(&hfoc->has5047p, &as5047p_angle);
    // hfoc->encoder_angle_mechanical = (((float)as5047p_angle / 16384.0f) * 2.0f * M_PIF) - hfoc->encoder_angle_mechanical_offset;

    normalize_angle(&hfoc->encoder_angle_mechanical);
    hfoc->encoder_angle_electrical = hfoc->encoder_angle_mechanical * hfoc->flash_data.motor_pole_pairs;
    normalize_angle(&hfoc->encoder_angle_electrical);

    return FOC_OK;
}


/**
  * @brief Updates the encoder angle
  * @param Handle to the FOC structure, delta time, filter alpha
  * @note filter alpha is used by the exponential filter to smooth the speed
  * @retval FOC_StatusTypeDef
  */
FOC_StatusTypeDef FOC_UpdateEncoderSpeed(FOC_HandleTypeDef *hfoc, float dt, float filter_alpha){
    float delta_angle = hfoc->encoder_angle_electrical - hfoc->previous_encoder_angle_electrical;
    if (delta_angle > M_PIF) {
        delta_angle -= 2 * M_PIF;
    } else if (delta_angle < -M_PIF) {
        delta_angle += 2 * M_PIF;
    }
    hfoc->encoder_speed_electrical = hfoc->encoder_speed_electrical * (1 - filter_alpha) + filter_alpha * delta_angle * dt;
    hfoc->previous_encoder_angle_electrical = hfoc->encoder_angle_electrical;

    return FOC_OK;
}

/**
  * @brief Sets the Current controller PI gains based on the motor parameters
  * @param Handle to the FOC structure
  * @note 
  * @retval FOC_StatusTypeDef
  */
FOC_StatusTypeDef FOC_TuneCurrentPID(FOC_HandleTypeDef *hfoc){
    

    if(hfoc->flash_data.current_control_bandwidth <= 0.0f || hfoc->flash_data.current_control_bandwidth > 5000) return FOC_ERROR; // check if the bandwidth is in range
    if(hfoc->flash_data.motor_identified_flag != 1) return FOC_ERROR; // check if the motor parameters are valid

    hfoc->flash_data.PID_gains_d.Kp = hfoc->flash_data.motor_stator_inductance * hfoc->flash_data.current_control_bandwidth;
    hfoc->flash_data.PID_gains_q.Kp = hfoc->flash_data.motor_stator_inductance * hfoc->flash_data.current_control_bandwidth;

    hfoc->flash_data.PID_gains_d.Ki = hfoc->flash_data.motor_stator_resistance * hfoc->flash_data.current_control_bandwidth;
    hfoc->flash_data.PID_gains_q.Ki = hfoc->flash_data.motor_stator_resistance * hfoc->flash_data.current_control_bandwidth;

    return FOC_OK;
}



