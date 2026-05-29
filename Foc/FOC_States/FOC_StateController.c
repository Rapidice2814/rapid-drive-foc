#include "FOC_States.h"
#include "FOC_Handle.h"
#include "FOC_USB.h"


/* ENUM, transition function, run function */
#define FOC_STATE_TABLE(X) \
    X(FOC_STATE_INIT,                       FOC_StateInit_Transition,                       FOC_StateInit) \
    X(FOC_STATE_RESET,                      FOC_StateReset_Transition,                      FOC_StateReset) \
    X(FOC_STATE_BOOTUP_SOUND,               FOC_StateBootupSound_Transition,                FOC_StateBootupSound) \
    X(FOC_STATE_CURRENT_SENSOR_CALIBRATION, FOC_StateCurrentSensorCalibration_Transition,   FOC_StateCurrentSensorCalibration) \
    X(FOC_STATE_IDENTIFY,                   FOC_StateIdentify_Transition,                   FOC_StateIdentify) \
    X(FOC_STATE_ANTICOGGING,                FOC_StateAntiCogging_Transition,                FOC_StateAntiCogging) \
    X(FOC_STATE_CHECKLIST,                  FOC_StateTransition_AlwaysOk,                   FOC_StateChecklist) \
    X(FOC_STATE_PID_AUTOTUNE,               FOC_StatePIDAutotune_Transition,                FOC_StatePIDAutotune) \
    X(FOC_STATE_ERROR,                      FOC_StateError_Transition,                      FOC_StateError) \
    X(FOC_STATE_ALIGNMENT,                  FOC_StateAlignment_Transition,                  FOC_StateAlignment) \
    X(FOC_STATE_ALIGNMENT_TEST,             FOC_StateAlignmentTest_Transition,              FOC_StateAlignmentTest) \
    X(FOC_STATE_RUN,                        FOC_StateRun_Transition,                        FOC_StateRun) \
    X(FOC_STATE_STOP,                       FOC_StateStop_Transition,                       FOC_StateStop) \
    X(FOC_STATE_IDLE,                       FOC_StateIdle_Transition,                       FOC_StateIdle) \
    X(FOC_STATE_FLASH_SAVE,                 FOC_StateFlashSave_Transition,                  FOC_StateFlashSave) \
    X(FOC_STATE_FLASH_LOAD,                 FOC_StateFlashLoad_Transition,                  FOC_StateFlashLoad) \
    X(FOC_STATE_OPENLOOP,                   FOC_StateOpenLoop_Transition,                   FOC_StateOpenLoop)



static inline FOC_StateTransitionTypeDef FOC_StateTransition_AlwaysOk(FOC_HandleTypeDef *hfoc){
    UNUSED(hfoc);
    return FOC_STATETRANSITION_OK;
}



static void FOC_StateChecklist(FOC_HandleTypeDef* hfoc){
    if(hfoc->adc_calibrated != 1){
        FOC_SetState(hfoc, FOC_STATE_CURRENT_SENSOR_CALIBRATION, FOC_STATE_CHECKLIST);
    }else if(hfoc->flash_data.encoder.offset_valid != 1){
        FOC_SetState(hfoc, FOC_STATE_ALIGNMENT, FOC_STATE_CHECKLIST);
    }else if(hfoc->flash_data.motor.phase_resistance_valid != 1 || hfoc->flash_data.motor.phase_inductance_valid != 1){
        FOC_SetState(hfoc, FOC_STATE_IDENTIFY, FOC_STATE_CHECKLIST);
    }else if(hfoc->flash_data.controller.current_PID_gains_valid != 1){
        FOC_SetState(hfoc, FOC_STATE_PID_AUTOTUNE, FOC_STATE_CHECKLIST);
    } else {
        FOC_SetState(hfoc, FOC_STATE_RUN, FOC_STATE_NONE);
    }
}





FOC_StateTransitionTypeDef FOC_SetState(FOC_HandleTypeDef *hfoc, FOC_StateTypeDef state, FOC_StateTypeDef next_state){
    if(next_state != FOC_STATE_NONE){
        hfoc->next_state = next_state;
    }

    if(state == hfoc->state){
        return FOC_STATETRANSITION_OK;
    }
    
    FOC_StateTransitionTypeDef transition_result = FOC_STATETRANSITION_OK;

    switch (state) {
        case FOC_STATE_NONE:
            return FOC_STATETRANSITION_INVALID;

        #define FOC_GENERATE_SETSTATE_CASE(state_enum, transition_fn, loop_fn) \
            case state_enum: \
                transition_result = transition_fn(hfoc); \
                break;

        FOC_STATE_TABLE(FOC_GENERATE_SETSTATE_CASE)

        #undef FOC_GENERATE_SETSTATE_CASE

        default:
            return FOC_STATETRANSITION_DENIED;
    }

    if(transition_result != FOC_STATETRANSITION_OK){
        return transition_result;
    }
    USB_printf("Transitioning from state %d to state %d\n", hfoc->state, state);
    
    hfoc->previous_state = hfoc->state;
    hfoc->state = state;
    return FOC_STATETRANSITION_OK;
}

void FOC_NextState(FOC_HandleTypeDef *hfoc){
    FOC_SetState(hfoc, hfoc->next_state, FOC_STATE_NONE);
}

void FOC_StateLoop(FOC_HandleTypeDef *hfoc){
    switch (hfoc->state) {
        case FOC_STATE_NONE:
            return;

        #define FOC_GENERATE_STATELOOP_CASE(state_enum, transition_fn, loop_fn) \
            case state_enum: \
                loop_fn(hfoc); \
                break;
        
        FOC_STATE_TABLE(FOC_GENERATE_STATELOOP_CASE)

        #undef FOC_GENERATE_STATELOOP_CASE

    }
}