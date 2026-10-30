#include "FOC_States.h"
#include "FOC_Handle.h"
#include "FOC_USB_Debug.h"
#include "FOC_Loops.h"
#include "FOC_Diagnostics.h"
#include "WS2812b_Driver.h"

/**
 * @brief Cleans up migrated routines on every accepted state exit.
 * @note Terminal results are retained; only active routines are cancelled.
 */
static FOC_LoopStatusTypeDef FOC_StateFinishRoutine(FOC_HandleTypeDef *hfoc){
    switch(hfoc->state){
        case FOC_STATE_ALIGNMENT:
            return FOC_Alignment_Finish(hfoc, FOC_LOOP_CANCELLED);
        case FOC_STATE_ENCODER_CALIBRATION:
            return FOC_EncoderCalibration_Finish(hfoc, FOC_LOOP_CANCELLED);
        case FOC_STATE_CURRENT_SENSOR_CALIBRATION:
            return FOC_CurrentSensorCalibration_Finish(hfoc, FOC_LOOP_CANCELLED);
        case FOC_STATE_IDENTIFY:
            return FOC_MotorIdentification_Finish(hfoc, FOC_LOOP_CANCELLED);
        case FOC_STATE_PID_AUTOTUNE:
            return FOC_PIDAutotune_Finish(hfoc, FOC_LOOP_CANCELLED);
        case FOC_STATE_ALIGNMENT_TEST:
            return FOC_AlignmentTest_Finish(hfoc, FOC_LOOP_CANCELLED);
        case FOC_STATE_ANTICOGGING:
            return FOC_AntiCogging_Finish(hfoc, FOC_LOOP_CANCELLED);
        case FOC_STATE_OPENLOOP:
            FOC_OpenLoop_Reset(hfoc);
            FOC_SetPhaseVoltages(hfoc, (PhaseVoltagesTypeDef){0});
            return FOC_LOOP_COMPLETED;
        default:
            return FOC_LOOP_COMPLETED;
    }
}

/**
 * @brief Initializes migrated routines only after state entry and exit cleanup.
 * @return ERROR if the motor output could not be enabled, otherwise OK.
 * @note Motor-driving setup starts with zero phase voltage before leaving high
 *       impedance. Failure latches an operation fault; it never starts a routine.
 */
static FOC_StateTransitionTypeDef FOC_StateStartRoutine(FOC_HandleTypeDef *hfoc){
    if(hfoc->state == FOC_STATE_ALIGNMENT || hfoc->state == FOC_STATE_IDENTIFY ||
       hfoc->state == FOC_STATE_ALIGNMENT_TEST){
        FOC_SetPhaseVoltages(hfoc, (PhaseVoltagesTypeDef){0});
        if(DRV8323_ExitHighImpedance(&hfoc->hdrv8323) != DRV8323_OK){
            Debug_SendTextResponse("Motor setup failed: could not enable inverter output.\n");
            FOC_RoutineRuntimeTypeDef failure = {.status = FOC_LOOP_ERROR,
                .error = FOC_ROUTINE_ERROR_INVALID_INPUT, .started_ms = HAL_GetTick(),
                .detail = "Gate driver refused inverter output enable; check driver communication/faults"};
            FOC_ReportRoutineResult(hfoc, "Motor setup", &failure, (unsigned)hfoc->state);
            FOC_SetError(hfoc, FOC_ERROR_OPERATION_FAILED);
            return FOC_STATETRANSITION_ERROR;
        }
    }
    if(hfoc->state == FOC_STATE_ALIGNMENT || hfoc->state == FOC_STATE_IDENTIFY ||
       hfoc->state == FOC_STATE_ALIGNMENT_TEST || hfoc->state == FOC_STATE_ANTICOGGING){
        WS2812b_SetAllColor(20, 15, 0);
        WS2812b_Send();
    }
    switch(hfoc->state){
        case FOC_STATE_ALIGNMENT:
            FOC_Alignment_Start(hfoc, 1.0f);
            break;
        case FOC_STATE_ENCODER_CALIBRATION:
            FOC_EncoderCalibration_Start(hfoc);
            break;
        case FOC_STATE_CURRENT_SENSOR_CALIBRATION:
            FOC_CurrentSensorCalibration_Start(hfoc);
            break;
        case FOC_STATE_IDENTIFY:
            FOC_MotorIdentification_Start(hfoc);
            break;
        case FOC_STATE_PID_AUTOTUNE:
            FOC_PIDAutotune_Start(hfoc);
            break;
        case FOC_STATE_ALIGNMENT_TEST:
            FOC_AlignmentTest_Start(hfoc, 1.0f);
            break;
        case FOC_STATE_ANTICOGGING:
            FOC_AntiCogging_Start(hfoc);
            break;
        case FOC_STATE_OPENLOOP:
            FOC_OpenLoop_Reset(hfoc);
            break;
        case FOC_STATE_RUN:
            hfoc->run_loop_divider = 0;
            break;
        default:
            break;
    }
    return FOC_STATETRANSITION_OK;
}

static inline FOC_StateTransitionTypeDef FOC_StateTransition_AlwaysOk(FOC_HandleTypeDef *hfoc){
    UNUSED(hfoc);
    return FOC_STATETRANSITION_OK;
}

const char *FOC_StateToString(FOC_StateTypeDef state){
    if((unsigned)state < FOC_STATE_COUNT &&
        FOC_StateNames[state] != NULL)
    {
        return FOC_StateNames[state];
    }

    return "FOC_STATE_UNKNOWN";
}

FOC_StateTransitionTypeDef FOC_StateChecklist_Transition(FOC_HandleTypeDef* hfoc){
    UNUSED(hfoc);
    return FOC_STATETRANSITION_OK;
}

static void FOC_StateChecklist(FOC_HandleTypeDef* hfoc){
    FOC_StateTypeDef requested = FOC_STATE_NONE;
    switch(hfoc->checklist_step){
        case 0:
            if(hfoc->adc_calibrated != 1)
                requested = FOC_STATE_CURRENT_SENSOR_CALIBRATION;
            break;
        case 1:
            if(!hfoc->encoder_runtime.healthy){
                /* Missing encoder is not a request for automatic sensorless motion. */
                FOC_SetState(hfoc, FOC_STATE_STOP, FOC_STATE_NONE);
                return;
            }
            if(hfoc->flash_data.encoder.offset_valid != 1)
                requested = FOC_STATE_ALIGNMENT;
            break;
        case 2:
            if(hfoc->flash_data.motor.phase_resistance == 0 || hfoc->flash_data.motor.phase_inductance == 0)
                requested = FOC_STATE_IDENTIFY;
            break;
        case 3:
            requested = FOC_STATE_PID_AUTOTUNE;
            break;
        case 4:
            FOC_SetState(hfoc, FOC_STATE_RUN, FOC_STATE_NONE);
            return;
        default:
            hfoc->checklist_step = 0;
            FOC_SetError(hfoc, FOC_ERROR_OPERATION_FAILED);
            return;
    }
    if(requested == FOC_STATE_NONE ||
       FOC_SetState(hfoc, requested, FOC_STATE_CHECKLIST) == FOC_STATETRANSITION_OK){
        hfoc->checklist_step++;
    }
}

FOC_StateTypeDef FOC_GetState(FOC_HandleTypeDef *hfoc){
    return hfoc->state;
}

/**
 * @brief Requests a state transition and returns the result.
 * @param hfoc FOC handle to transition.
 * @param state Requested state; ERROR requires at least one latched fault.
 * @param next_state Optional continuation, committed only after acceptance.
 * @return Transition result or the active/latched fault preventing the change.
 * @note ERROR cancels pending continuations. Fault acknowledgement recovers
 *       through STOP rather than resuming the interrupted operation.
 */
FOC_StateTransitionTypeDef FOC_SetState(FOC_HandleTypeDef *hfoc, FOC_StateTypeDef state, FOC_StateTypeDef next_state){
    if(hfoc->calibration_upload.active && state != FOC_STATE_STOP && state != FOC_STATE_ERROR && state != FOC_STATE_IDLE)
        return FOC_STATETRANSITION_DENIED;
    if(state == hfoc->state){
        if(next_state != FOC_STATE_NONE) hfoc->next_state = next_state;
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

    /* The existing destination transition functions include entry side effects.
       Migrated routine cleanup never enables output. A cleanup failure overrides
       the destination with immediate ERROR rather than allowing motion. */
    if(FOC_StateFinishRoutine(hfoc) == FOC_LOOP_ERROR && state != FOC_STATE_ERROR){
        hfoc->active_errors |= FOC_ERROR_OPERATION_FAILED;
        hfoc->latched_errors |= FOC_ERROR_OPERATION_FAILED;
        FOC_StateError_Transition(hfoc);
        state = FOC_STATE_ERROR;
        transition_result = FOC_STATETRANSITION_ERROR;
    }
    if(state == FOC_STATE_ERROR){
        hfoc->next_state = FOC_STATE_NONE;
    }else{
        hfoc->next_state = next_state;
    }
    Debug_SendTextResponse("Transitioning from state %s to state %s\n", FOC_StateToString(hfoc->state), FOC_StateToString(state));
    
    hfoc->previous_state = hfoc->state;
    hfoc->state = state;
    if(state != FOC_STATE_CHECKLIST && hfoc->next_state != FOC_STATE_CHECKLIST){
        hfoc->checklist_step = 0;
    }
    if(FOC_StateStartRoutine(hfoc) != FOC_STATETRANSITION_OK)
        return FOC_STATETRANSITION_ERROR;
    return transition_result;
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
