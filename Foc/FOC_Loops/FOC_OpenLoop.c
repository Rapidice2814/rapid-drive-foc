#include "FOC_Loops.h"
#include "FOC_Handle.h"
#include <math.h>
#include "Cordic.h"

/**
 * @brief Clears this handle's open-loop phase on explicit state entry/exit.
 * @param hfoc FOC handle owning the phase accumulator.
 */
void FOC_OpenLoop_Reset(FOC_HandleTypeDef *hfoc){
    hfoc->open_loop.electrical_angle = 0;
}

/**
 * @brief Advances this handle's phase and applies one open-loop voltage sample.
 * @return Zero on success, one for nonfinite inputs or an invalid sample rate.
 */
uint8_t FOC_OpenLoop(FOC_HandleTypeDef *hfoc, float espeed, float magnitude, float loop_frequency){
    if(!isfinite(espeed) || !isfinite(magnitude) || !isfinite(loop_frequency) || loop_frequency <= 0){
        FOC_SetPhaseVoltages(hfoc, (PhaseVoltagesTypeDef){0});
        return 1;
    }
    float step_size = espeed / loop_frequency;
    float angle = hfoc->open_loop.electrical_angle + step_size;
    if(!isfinite(angle)){
        FOC_SetPhaseVoltages(hfoc, (PhaseVoltagesTypeDef){0});
        return 1;
    }
    hfoc->open_loop.electrical_angle = angle;
    normalize_angle_0_2pi(&hfoc->open_loop.electrical_angle);

    ABVoltagesTypeDef Vab;

    float cos_theta, sin_theta;
    Cordic_CalculateSinCos(hfoc->open_loop.electrical_angle, &cos_theta, &sin_theta);

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
