#include <math.h>
#include <stdio.h>

#include "FOC_Driver.h"
#include "FOC_Utils.h"

/* uncomment these to use the cos and sin optimizations */
#define cosf _cosf
#define sinf _sinf

/* General */
void FOC_SetInputVoltage(FOC_HandleTypeDef *hfoc, float vbus){
	hfoc->vbus = vbus;
}
void FOC_SetVoltageLimit(FOC_HandleTypeDef *hfoc, float voltage_limit){
    hfoc->voltage_limit = voltage_limit;
}

/* Calculations */
AlphaBetaCurrents FOC_Clarke_transform(PhaseCurrents current){
    AlphaBetaCurrents result;
    
	float mid = (1.f/3) * (current.a + current.b + current.c);
	float a = current.a - mid;
	float b = current.b - mid;
	result.alpha = a;
	result.beta = M_1_SQRT3F * a + M_2_SQRT3F * b;
    return result;
}

DQCurrents FOC_Park_transform(AlphaBetaCurrents ab_current, float theta){
    DQCurrents result;

	float cos_theta = cosf(theta);
	float sin_theta = sinf(theta);
	result.d = ab_current.alpha * cos_theta + ab_current.beta * sin_theta;
	result.q = ab_current.beta * cos_theta - ab_current.alpha * sin_theta;

    return result;
}

AlphaBetaVoltages FOC_InvPark_transform(DQVoltages dq_voltage, float theta){
    AlphaBetaVoltages result;

	float cos_theta = cosf(theta);
	float sin_theta = sinf(theta);
    result.alpha = dq_voltage.d * cos_theta - dq_voltage.q * sin_theta;
    result.beta = dq_voltage.d * sin_theta + dq_voltage.q * cos_theta;

    return result;
}

PhaseVoltages FOC_InvClarke_transform(AlphaBetaVoltages ab_voltage){
    PhaseVoltages result;
    result.a = ab_voltage.alpha;
    result.b = -0.5f * ab_voltage.alpha + M_SQRT3_2F * ab_voltage.beta;
    result.c = -0.5f * ab_voltage.alpha - M_SQRT3_2F * ab_voltage.beta;

    return result;
}

void FOC_SetPhaseVoltages(FOC_HandleTypeDef *hfoc, PhaseVoltages phase_voltages){
    
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
}


void FOC_SetPWMCCRPointers(FOC_HandleTypeDef *hfoc, volatile uint32_t *pCCRa, volatile uint32_t *pCCRb, volatile uint32_t *pCCRc, uint32_t max_ccr){
	hfoc->max_ccr = max_ccr;
	hfoc->pCCRa = pCCRa;
	hfoc->pCCRb = pCCRb;
	hfoc->pCCRc = pCCRc;

	// Set the PWM duty cycles to 0
	*(hfoc->pCCRa) = 0; 
	*(hfoc->pCCRb) = 0;
	*(hfoc->pCCRc) = 0;
}



/* Encoder */
void FOC_SetEncoderPointer(FOC_HandleTypeDef *hfoc, volatile uint32_t *pencoder_count){
    hfoc->pencoder_count = pencoder_count;
}

void FOC_SetEncoderZero(FOC_HandleTypeDef *hfoc){
    float ap5047p_angle = 0.0f;
    AS5047P_GetAngle(&(hfoc->has5047p), &ap5047p_angle);
    hfoc->encoder_angle_mechanical_offset = ap5047p_angle;
    *(hfoc->pencoder_count) = (int32_t)(ap5047p_angle * ENCODER_PULSES_PER_ROTATION / (2 * M_PIF));
}

void FOC_UpdateEncoderAngle(FOC_HandleTypeDef *hfoc){
    hfoc->encoder_angle_mechanical = ((float)(*(hfoc->pencoder_count)) / ENCODER_PULSES_PER_ROTATION) * 2 * M_PIF - hfoc->encoder_angle_mechanical_offset;
    if (hfoc->encoder_angle_mechanical < 0) {
        hfoc->encoder_angle_mechanical += 2 * M_PIF;
    } else if (hfoc->encoder_angle_mechanical > 2 * M_PIF) {
        hfoc->encoder_angle_mechanical -= 2 * M_PIF;
    }
    hfoc->encoder_angle_electrical = hfoc->encoder_angle_mechanical * hfoc->motor_pole_pairs;
    while (hfoc->encoder_angle_electrical > 2*M_PIF){
        hfoc->encoder_angle_electrical -= 2*M_PIF;
    }
}

// #define ENCODER_SPEED_LOOP_ALPHA 0.05f
void FOC_UpdateEncoderSpeed(FOC_HandleTypeDef *hfoc, float dt, float filter_alpha){
    float delta_angle = hfoc->encoder_angle_electrical - hfoc->previous_encoder_angle_electrical;
    if (delta_angle > M_PIF) {
        delta_angle -= 2 * M_PIF;
    } else if (delta_angle < -M_PIF) {
        delta_angle += 2 * M_PIF;
    }
    hfoc->encoder_speed_electrical = hfoc->encoder_speed_electrical * (1 - filter_alpha) + filter_alpha * delta_angle * dt;
    hfoc->previous_encoder_angle_electrical = hfoc->encoder_angle_electrical;
}


