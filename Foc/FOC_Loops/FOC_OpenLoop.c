#include "FOC_Loops.h"
#include <math.h>

uint8_t FOC_OpenLoop(FOC_HandleTypeDef *hfoc, float espeed, float magnitude, float loop_frequency){
    static float reference_electrical_angle = 0.0f;
    float step_size = espeed / loop_frequency;
    reference_electrical_angle += step_size;
    normalize_angle_0_2pi(&reference_electrical_angle);

    ABVoltagesTypeDef Vab;
    Vab.alpha = magnitude * cosf(reference_electrical_angle);
    Vab.beta = magnitude * sinf(reference_electrical_angle);
    PhaseVoltagesTypeDef phase_voltage = FOC_InvClarke_transform(Vab);
    if(espeed > 0.0f){
        FOC_SetPhaseVoltages(hfoc, phase_voltage);
    } else{
        FOC_SetPhaseVoltages(hfoc, (PhaseVoltagesTypeDef){0.0f, 0.0f, 0.0f});
    }

    return 0;
}
