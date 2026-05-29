#include "FOC_Loops.h"
#include "FOC_Handle.h"
#include <math.h>
#include "Cordic.h"

uint8_t FOC_OpenLoop(FOC_HandleTypeDef *hfoc, float espeed, float magnitude, float loop_frequency){
    static float reference_electrical_angle = 0.0f;
    float step_size = espeed / loop_frequency;
    reference_electrical_angle += step_size;
    normalize_angle_0_2pi(&reference_electrical_angle);

    ABVoltagesTypeDef Vab;

    float cos_theta, sin_theta;
    Cordic_CalculateSinCos(reference_electrical_angle, &cos_theta, &sin_theta);

    Vab.alpha = magnitude * cos_theta;
    Vab.beta = magnitude * sin_theta;
    PhaseVoltagesTypeDef phase_voltage = InvClarke_transform(Vab);
    if(espeed > 0.0f){
        FOC_SetPhaseVoltages(hfoc, phase_voltage);
    } else{
        FOC_SetPhaseVoltages(hfoc, (PhaseVoltagesTypeDef){0.0f, 0.0f, 0.0f});
    }

    return 0;
}
