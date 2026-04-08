#include "FOC_HFI.h"
#include "main.h"
#include "math.h"


float FOC_HFI_GetInjectedVoltage(){
    static float phase = 0.0f;
    float voltage = HFI_INJECTION_AMPLITUDE * sinf(phase);
    phase += HFI_INJECTION_OMEGA / CURRENT_LOOP_FREQUENCY;
    normalize_angle_0_2pi(&phase);
    return voltage;
}