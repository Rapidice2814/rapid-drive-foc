#include "FOC_States.h"
#include "FOC_Handle.h"
#include "FOC_Utils.h"
#include "ErrorLights.h"

/* FOC_STATE_ERROR */

FOC_StateTransitionTypeDef FOC_StateError_Transition(FOC_HandleTypeDef* hfoc){
    UNUSED(hfoc);
    return FOC_STATETRANSITION_OK;
}

void FOC_StateError(FOC_HandleTypeDef* hfoc){
    DRV8323_SetHighImpedance(&hfoc->hdrv8323);
    FOC_SetPhaseVoltages(hfoc, (PhaseVoltagesTypeDef){0.0f, 0.0f, 0.0f});

    if(LightEngine_PlaySequence(error_light_array, sizeof(error_light_array)/sizeof(error_light_array[0])) != LIGHT_IN_PROGRESS){
        __NOP();
    }
}