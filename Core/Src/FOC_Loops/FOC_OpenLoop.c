#include "FOC_Loops.h"
#include <math.h>

uint8_t FOC_OpenLoop(FOC_HandleTypeDef *hfoc, float espeed, float magnitude, float loop_frequency){
    static float reference_electrical_angle = 0.0f;
    float step_size = espeed / loop_frequency;
    reference_electrical_angle += step_size;
    normalize_angle(&reference_electrical_angle);

    ABVoltagesTypeDef Vab;
    Vab.alpha = magnitude * cosf(reference_electrical_angle);
    Vab.beta = magnitude * sinf(reference_electrical_angle);
    PhaseVoltagesTypeDef phase_voltages = FOC_InvClarke_transform(Vab);
    if(espeed > 0.0f){
        FOC_SetPhaseVoltages(hfoc, phase_voltages);
    } else{
        FOC_SetPhaseVoltages(hfoc, (PhaseVoltagesTypeDef){0.0f, 0.0f, 0.0f});
    }
    
    // hfoc->dq_voltage.d = 0.0f;
    // hfoc->dq_voltage.q = 1.0f;

    // hfoc->ab_voltage = FOC_InvPark_transform(hfoc->dq_voltage, hfoc->encoder_angle_electrical);
    // PhaseVoltagesTypeDef phase_voltages = FOC_InvClarke_transform(hfoc->ab_voltage);
    // FOC_SetPhaseVoltages(hfoc, phase_voltages);

    return 0;
}
