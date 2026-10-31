#include "FOC_Diagnostics.h"
#include "FOC_States.h"
#include "FOC_Handle.h"
#include <math.h>
#include <stdio.h>
#include "FOC_USB_Debug.h"

/* Called once by each routine after cleanup. No blocking sensor access in the ISR. */
void FOC_ReportRoutineResult(FOC_HandleTypeDef *h, const char *name,
                            FOC_RoutineRuntimeTypeDef *r, unsigned stage){
    h->setup_result.status = (uint8_t)r->status;
    h->setup_result.error = (uint8_t)r->error;
    const char *fallback[] = {"No detailed reason recorded", "Routine not started",
        "Invalid routine stage/counters", "Invalid input/configuration or terminal status",
        "Routine timed out", "Encoder read failed", "Current-sensor calibration/driver failure",
        "Result failed validation"};
    const char *reason = r->detail ? r->detail : fallback[(unsigned)r->error < 8 ? r->error : 0];
    if(r->status == FOC_LOOP_CANCELLED && !r->detail) reason = "Stopped/cancelled by request or state change";
    const char *status = r->status == FOC_LOOP_COMPLETED ? "completed" :
                         r->status == FOC_LOOP_CANCELLED ? "cancelled" : "failed";
    snprintf(h->routine_result, sizeof(h->routine_result),
        "Routine result: %s %s. %s; stage=%u; code=%u; elapsed=%lums; active=0x%08lX; latched=0x%08lX.",
        name, status, r->status == FOC_LOOP_COMPLETED ? "Result accepted" : reason,
        stage, (unsigned)r->error, (unsigned long)(HAL_GetTick() - r->started_ms),
        (unsigned long)h->active_errors, (unsigned long)h->latched_errors);
    Debug_SendTextResponse("%s\n", h->routine_result);
}

/**
 * @brief Recomputes active faults, latches them, and updates the fault state.
 * @param hfoc FOC handle with the latest measurements and configured limits.
 * @note ERROR is retained until latched faults are acknowledged. Recovery goes
 *       to STOP only when both active and latched fault masks are clear.
 */
void FOC_CheckErrors(FOC_HandleTypeDef *hfoc){
    uint32_t active_errors = 0;
    if(hfoc->adc_values.current_sense_saturated) active_errors |= FOC_ERROR_CURRENT_SENSE_SATURATION;

    if(DRV8323_CheckFault(&hfoc->hdrv8323)) active_errors |= FOC_ERROR_DRIVER_FAULT;
    
    if(hfoc->adc_values.motor_temp > hfoc->flash_data.limits.motor_temp_trip_level) active_errors |= FOC_ERROR_MOTOR_OT;
    if(hfoc->adc_values.motor_temp < 0.0f) active_errors |= FOC_ERROR_MOTOR_UT;

    if(hfoc->adc_values.mosfet_temp > hfoc->flash_data.limits.mosfet_temp_trip_level) active_errors |= FOC_ERROR_MOSFET_OT;
    if(hfoc->adc_values.mosfet_temp < 0.0f) active_errors |= FOC_ERROR_MOSFET_UT;

    if(hfoc->adc_values.vbus > hfoc->flash_data.limits.vbus_overvoltage_trip_level) active_errors |= FOC_ERROR_VBUS_OV;
    if(hfoc->adc_values.vbus < hfoc->flash_data.limits.vbus_undervoltage_trip_level) active_errors |= FOC_ERROR_VBUS_UV;
    if(hfoc->ibus > hfoc->flash_data.limits.ibus_overcurrent_trip_level) active_errors |= FOC_ERROR_IBUS_OC;

    /* Raw phase measurements protect voltage-driven routines too. Bus current
       can remain small while winding current exceeds its configured limit. */
    if(hfoc->adc_calibrated){
        float limit = hfoc->flash_data.limits.max_dq_current;
        if(!isfinite(limit) || limit <= 0) active_errors |= FOC_ERROR_INVALID_CONFIGURATION;
        else if(!isfinite(hfoc->adc_values.phase_current.a) ||
                !isfinite(hfoc->adc_values.phase_current.b) ||
                !isfinite(hfoc->adc_values.phase_current.c)) active_errors |= FOC_ERROR_SYSTEM_FAULT;
        else if(fabsf(hfoc->adc_values.phase_current.a) > limit ||
                fabsf(hfoc->adc_values.phase_current.b) > limit ||
                fabsf(hfoc->adc_values.phase_current.c) > limit) active_errors |= FOC_ERROR_PHASE_OC;
    }

    hfoc->active_errors = active_errors;
    hfoc->latched_errors |= active_errors;

    if(FOC_StateIsMaintenance(hfoc->state)){
        /* Allow fault recovery/storage and firmware updates without motion. */
        DRV8323_SetHighImpedance(&hfoc->hdrv8323);
        FOC_SetPhaseVoltages(hfoc, (PhaseVoltagesTypeDef){0.0f, 0.0f, 0.0f});
    }else if(hfoc->latched_errors){
        FOC_SetState(hfoc, FOC_STATE_ERROR, FOC_STATE_NONE);
    }else if(hfoc->active_errors == 0 && hfoc->state == FOC_STATE_ERROR){
        FOC_SetState(hfoc, FOC_STATE_STOP, FOC_STATE_NONE);
    }
}

/**
 * @brief Acknowledges latched faults and immediately reevaluates active faults.
 * @param hfoc FOC handle to acknowledge.
 * @note Present faults relatch immediately; clearing does not resume RUN.
 */
void FOC_ClearLatchedErrors(FOC_HandleTypeDef *hfoc){
    hfoc->latched_errors = 0;
    FOC_CheckErrors(hfoc);
}

/**
 * @brief Latches a reported fault and requests ERROR.
 * @param hfoc FOC handle receiving the fault.
 * @param error Nonzero fault bitmap; one-shot software faults remain latched
 *              after active hardware faults are reevaluated.
 */
void FOC_SetError(FOC_HandleTypeDef *hfoc, uint32_t error){
    if(error == 0) return;
    hfoc->active_errors |= error;
    hfoc->latched_errors |= error;
    FOC_SetState(hfoc, FOC_STATE_ERROR, FOC_STATE_NONE);
}

uint32_t FOC_GetActiveErrors(FOC_HandleTypeDef *hfoc){
    return hfoc->active_errors;
}

uint32_t FOC_GetLatchedErrors(FOC_HandleTypeDef *hfoc){
    return hfoc->latched_errors;
}
