#include "FOC_States.h"
#include "FOC_Handle.h"
#include "FOC_Loops.h"
#include "FOC_USB_Debug.h"
#include "FOC_Diagnostics.h"
#include "WS2812b_Driver.h"
#include <math.h>

/**
 * @brief Validates stationary, fault-free entry to motor-driving setup routines.
 * @param hfoc Controller with current sensor and bus measurements.
 * @note CHECKLIST is allowed for startup; manual setup must start from IDLE.
 *       This check has no output side effects. Output is enabled after cleanup
 *       of the previous routine, once the state transition has been accepted.
 */
static FOC_StateTransitionTypeDef FOC_StateMotorSetup_Validate(FOC_HandleTypeDef *hfoc){
    if(hfoc->active_errors) return FOC_STATETRANSITION_ACTIVE_FAULTS;
    if(hfoc->latched_errors) return FOC_STATETRANSITION_LATCHED_FAULTS;
    if(hfoc->state != FOC_STATE_IDLE && hfoc->state != FOC_STATE_CHECKLIST)
        return FOC_STATETRANSITION_MUST_BE_IDLE;
    if(!hfoc->adc_calibrated) return FOC_STATETRANSITION_ADC_NOT_CALIBRATED;
    if(!isfinite(hfoc->encoder_speed_mechanical) || fabsf(hfoc->encoder_speed_mechanical) > 0.1f)
        return FOC_STATETRANSITION_MOTOR_MOVING;
    if(!isfinite(hfoc->adc_values.vbus) || hfoc->adc_values.vbus <= 0.0f)
        return FOC_STATETRANSITION_DENIED;
    return FOC_STATETRANSITION_OK;
}

/** @brief Allows encoder calibration only from stationary, fault-free IDLE. */
FOC_StateTransitionTypeDef FOC_StateEncoderCalibration_Transition(FOC_HandleTypeDef *hfoc){
    if(hfoc->state != FOC_STATE_IDLE) return FOC_STATETRANSITION_MUST_BE_IDLE;
    if(hfoc->active_errors) return FOC_STATETRANSITION_ACTIVE_FAULTS;
    if(hfoc->latched_errors) return FOC_STATETRANSITION_LATCHED_FAULTS;
    if(!hfoc->adc_calibrated) return FOC_STATETRANSITION_ADC_NOT_CALIBRATED;
    if(!hfoc->flash_data.encoder.offset_valid) return FOC_STATETRANSITION_ENCODER_NOT_ALIGNED;
    if(!isfinite(hfoc->encoder_speed_mechanical) || fabsf(hfoc->encoder_speed_mechanical) > 0.1f) return FOC_STATETRANSITION_MOTOR_MOVING;
    const FOC_EncoderSourceTypeDef *source = FOC_EncoderSource(hfoc->flash_data.encoder.source_id);
    if(source == NULL || !source->calibratable || !hfoc->encoder_runtime.healthy ||
       hfoc->flash_data.motor.pole_pairs == 0) return FOC_STATETRANSITION_DENIED;
    return FOC_STATETRANSITION_OK;
}

/** @brief Advances calibration; success stops, failure latches an operation fault. */
void FOC_StateEncoderCalibration(FOC_HandleTypeDef *hfoc){
    FOC_LoopStatusTypeDef status = FOC_EncoderCalibration_Update(hfoc);
    if(status == FOC_LOOP_COMPLETED) FOC_SetState(hfoc, FOC_STATE_STOP, FOC_STATE_NONE);
    else if(status == FOC_LOOP_ERROR) FOC_SetError(hfoc,
        (hfoc->active_errors | hfoc->latched_errors) ?
        (hfoc->active_errors | hfoc->latched_errors) : FOC_ERROR_OPERATION_FAILED);
}

/**
 * @brief Continues a migrated calibration or stops when no continuation exists.
 * @param hfoc FOC handle with a successfully completed routine.
 * @note A denied continuation falls back to STOP; completion never restarts it.
 */
static void FOC_StateCalibrationComplete(FOC_HandleTypeDef *hfoc){
    FOC_StateTypeDef destination = hfoc->next_state == FOC_STATE_NONE ? FOC_STATE_STOP : hfoc->next_state;
    if(FOC_SetState(hfoc, destination, FOC_STATE_NONE) != FOC_STATETRANSITION_OK &&
       hfoc->state != FOC_STATE_ERROR){
        FOC_SetState(hfoc, FOC_STATE_STOP, FOC_STATE_NONE);
    }
}

/*FOC_STATE_CURRENT_SENSOR_CALIBRATION*/

FOC_StateTransitionTypeDef FOC_StateCurrentSensorCalibration_Transition(FOC_HandleTypeDef* hfoc){
    UNUSED(hfoc);
    return FOC_STATETRANSITION_OK;
}

void FOC_StateCurrentSensorCalibration(FOC_HandleTypeDef* hfoc){
    FOC_LoopStatusTypeDef ret = FOC_CurrentSensorCalibration_Update(hfoc);
    if(ret == FOC_LOOP_COMPLETED){
        FOC_StateCalibrationComplete(hfoc);
    }else if(ret == FOC_LOOP_ERROR){
        Debug_SendTextResponse("Current sensor calibration failed!\n");
        FOC_SetError(hfoc, FOC_ERROR_OPERATION_FAILED);
    }
}

/*FOC_STATE_IDENTIFY*/

FOC_StateTransitionTypeDef FOC_StateIdentify_Transition(FOC_HandleTypeDef* hfoc){
    return FOC_StateMotorSetup_Validate(hfoc);
}

void FOC_StateIdentify(FOC_HandleTypeDef* hfoc){
    FOC_LoopStatusTypeDef ret = FOC_MotorIdentification_Update(hfoc);
    if(ret == FOC_LOOP_COMPLETED){
        FOC_StateCalibrationComplete(hfoc);
    } else if(ret == FOC_LOOP_ERROR){
        Debug_SendTextResponse("Motor identification failed!\n");
        FOC_SetError(hfoc, FOC_ERROR_OPERATION_FAILED);
    } 
}

/*FOC_STATE_PID_AUTOTUNE*/

FOC_StateTransitionTypeDef FOC_StatePIDAutotune_Transition(FOC_HandleTypeDef* hfoc){
    UNUSED(hfoc);
    return FOC_STATETRANSITION_OK;
}

void FOC_StatePIDAutotune(FOC_HandleTypeDef* hfoc){
    FOC_LoopStatusTypeDef ret = FOC_PIDAutotune_Update(hfoc);
    if(ret == FOC_LOOP_COMPLETED){
        hfoc->flash_data.controller.anticogging_data_valid = 0;
        hfoc->flash_data.controller.current_PID_FF_enabled = 0;
        FOC_StateCalibrationComplete(hfoc);
    } else if(ret == FOC_LOOP_ERROR){
        Debug_SendTextResponse("PID autotune failed!\n");
        FOC_SetError(hfoc, FOC_ERROR_OPERATION_FAILED);
    }
}

/*FOC_STATE_ALIGNMENT*/

FOC_StateTransitionTypeDef FOC_StateAlignment_Transition(FOC_HandleTypeDef* hfoc){
    FOC_StateTransitionTypeDef result = FOC_StateMotorSetup_Validate(hfoc);
    if(result != FOC_STATETRANSITION_OK) return result;
    if(!hfoc->encoder_runtime.healthy) return FOC_STATETRANSITION_DENIED;
    return FOC_STATETRANSITION_OK;
}

void FOC_StateAlignment(FOC_HandleTypeDef* hfoc){
    FOC_LoopStatusTypeDef ret = FOC_Alignment_Update(hfoc);
    if(ret == FOC_LOOP_COMPLETED){
        hfoc->flash_data.controller.anticogging_data_valid = 0;
        FOC_StateCalibrationComplete(hfoc);
    } else if(ret == FOC_LOOP_ERROR){
        Debug_SendTextResponse("Alignment failed!\n");
        FOC_SetError(hfoc, FOC_ERROR_OPERATION_FAILED);
    }
}

/*FOC_STATE_ALIGNMENT_TEST*/

FOC_StateTransitionTypeDef FOC_StateAlignmentTest_Transition(FOC_HandleTypeDef* hfoc){
    FOC_StateTransitionTypeDef result = FOC_StateMotorSetup_Validate(hfoc);
    if(result != FOC_STATETRANSITION_OK) return result;
    if(!hfoc->encoder_runtime.healthy || hfoc->flash_data.motor.pole_pairs == 0)
        return FOC_STATETRANSITION_DENIED;
    return FOC_STATETRANSITION_OK;
}

void FOC_StateAlignmentTest(FOC_HandleTypeDef* hfoc){
    FOC_LoopStatusTypeDef  ret = FOC_AlignmentTest_Update(hfoc);
    if(ret == FOC_LOOP_COMPLETED){
        FOC_StateCalibrationComplete(hfoc);
    } else if(ret == FOC_LOOP_ERROR){
        Debug_SendTextResponse("Alignment test failed!\n");
        FOC_SetError(hfoc, FOC_ERROR_OPERATION_FAILED);
    }
}

/*FOC_STATE_ANTICOGGING*/

FOC_StateTransitionTypeDef FOC_StateAntiCogging_Transition(FOC_HandleTypeDef* hfoc){
    if(hfoc->state != FOC_STATE_IDLE) return FOC_STATETRANSITION_MUST_BE_IDLE;
    if(hfoc->active_errors) return FOC_STATETRANSITION_ACTIVE_FAULTS;
    if(hfoc->latched_errors) return FOC_STATETRANSITION_LATCHED_FAULTS;
    if(!hfoc->adc_calibrated) return FOC_STATETRANSITION_ADC_NOT_CALIBRATED;
    if(!hfoc->flash_data.encoder.offset_valid) return FOC_STATETRANSITION_ENCODER_NOT_ALIGNED;
    if(!isfinite(hfoc->encoder_speed_mechanical) || fabsf(hfoc->encoder_speed_mechanical) > 0.1f) return FOC_STATETRANSITION_MOTOR_MOVING;
    if(!hfoc->encoder_runtime.healthy) return FOC_STATETRANSITION_DENIED;
    return FOC_STATETRANSITION_OK;
}

void FOC_StateAntiCogging(FOC_HandleTypeDef* hfoc){
    FOC_LoopStatusTypeDef ret = FOC_AntiCogging_Update(hfoc);
    if(ret == FOC_LOOP_COMPLETED){
        hfoc->flash_data.controller.anticogging_data_valid = 1;
        FOC_StateCalibrationComplete(hfoc);
    } else if(ret == FOC_LOOP_ERROR){
        Debug_SendTextResponse("Anti-cogging measurement failed!\n");
        FOC_SetError(hfoc, (hfoc->active_errors | hfoc->latched_errors) ?
            (hfoc->active_errors | hfoc->latched_errors) : FOC_ERROR_OPERATION_FAILED);
    }
}

FOC_StateTransitionTypeDef FOC_StateMotorProbe_Transition(FOC_HandleTypeDef *h){
    FOC_StateTransitionTypeDef result = FOC_StateMotorSetup_Validate(h);
    if(result != FOC_STATETRANSITION_OK) return result;
    const FOC_EncoderSourceTypeDef *source = FOC_EncoderSource(h->flash_data.encoder.source_id);
    if(!h->encoder_runtime.healthy || !source || !source->calibratable) return FOC_STATETRANSITION_DENIED;
    return FOC_STATETRANSITION_OK;
}
void FOC_StateMotorProbe(FOC_HandleTypeDef *h){
    FOC_LoopStatusTypeDef status = FOC_MotorProbe_Update(h);
    if(status == FOC_LOOP_COMPLETED) FOC_SetState(h, FOC_STATE_STOP, FOC_STATE_NONE);
    else if(status == FOC_LOOP_ERROR) FOC_SetError(h, FOC_ERROR_OPERATION_FAILED);
}
