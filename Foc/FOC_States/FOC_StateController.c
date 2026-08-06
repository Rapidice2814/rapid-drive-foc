#include "FOC_States.h"
#include "FOC_Handle.h"
#include "FOC_USB_Debug.h"

static inline FOC_StateTransitionTypeDef FOC_StateTransition_AlwaysOk(FOC_HandleTypeDef *hfoc){
    UNUSED(hfoc);
    return FOC_STATETRANSITION_OK;
}

const char *FOC_StateToString(FOC_StateTypeDef state)
{
    if ((unsigned)state < FOC_STATE_COUNT &&
        FOC_StateNames[state] != NULL)
    {
        return FOC_StateNames[state];
    }

    return "FOC_STATE_UNKNOWN";
}


static void FOC_StateChecklist(FOC_HandleTypeDef* hfoc){
    static uint8_t checklist_step = 0;
    switch(checklist_step){
        case 0:
            if(hfoc->adc_calibrated != 1)
                FOC_SetState(hfoc, FOC_STATE_CURRENT_SENSOR_CALIBRATION, FOC_STATE_CHECKLIST);
            break;
        case 1:
            if(hfoc->flash_data.encoder.offset_valid != 1)
                FOC_SetState(hfoc, FOC_STATE_ALIGNMENT, FOC_STATE_CHECKLIST);
            break;
        case 2:
            if(hfoc->flash_data.motor.phase_resistance == 0 || hfoc->flash_data.motor.phase_inductance == 0)
                FOC_SetState(hfoc, FOC_STATE_IDENTIFY, FOC_STATE_CHECKLIST);
            break;
        case 3:
            FOC_SetState(hfoc, FOC_STATE_PID_AUTOTUNE, FOC_STATE_CHECKLIST);
            break;
        case 4:
            FOC_SetState(hfoc, FOC_STATE_RUN, FOC_STATE_NONE);
            checklist_step = 0;
            return;
        default:
            break;
    }
    checklist_step++;
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
    Debug_SendTextResponse("Transitioning from state %s to state %s\n", FOC_StateToString(hfoc->state), FOC_StateToString(state));
    
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
        case FOC_STATE_COUNT:
            return;

        #define FOC_GENERATE_STATELOOP_CASE(state_enum, transition_fn, loop_fn) \
            case state_enum: \
                loop_fn(hfoc); \
                break;
        
        FOC_STATE_TABLE(FOC_GENERATE_STATELOOP_CASE)

        #undef FOC_GENERATE_STATELOOP_CASE

    }
}