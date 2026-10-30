#include <math.h>
#include "FOC_Loops.h"
#include "FOC_Handle.h"
#include "FOC_ADC.h"
#include "FOC_USB_Debug.h"

#define CURRENT_SENSOR_CALIBRATION_STEPS 100U
#define CURRENT_SENSOR_CALIBRATION_TIMEOUT_MS 1000U

/**
 * @brief Starts a fresh current-sensor calibration with empty accumulators.
 * @param hfoc FOC handle owning the workspace.
 * @note Old ADC offsets remain installed, but are not considered calibrated.
 */
void FOC_CurrentSensorCalibration_Start(FOC_HandleTypeDef *hfoc){
    if(hfoc->current_calibration.runtime.active){
        if(FOC_CurrentSensorCalibration_Finish(hfoc, FOC_LOOP_CANCELLED) == FOC_LOOP_ERROR) return;
    }
    hfoc->current_calibration = (FOC_CurrentCalibrationContextTypeDef){
        .runtime = {.started_ms = HAL_GetTick(), .status = FOC_LOOP_IN_PROGRESS, .active = 1},
        .stage = FOC_CURRENT_CAL_START
    };
    hfoc->adc_calibrated = 0;
}

/**
 * @brief Leaves CSA calibration mode and commits offsets only on success.
 * @param hfoc FOC handle owning the routine.
 * @param result Completion, error, or cancellation; never IN_PROGRESS.
 * @return Retained result; a driver cleanup failure always returns ERROR.
 * @note Cleanup never enables output. Repeated calls have no hardware effects.
 */
FOC_LoopStatusTypeDef FOC_CurrentSensorCalibration_Finish(FOC_HandleTypeDef *hfoc, FOC_LoopStatusTypeDef result){
    FOC_CurrentCalibrationContextTypeDef *ctx = &hfoc->current_calibration;
    if(!ctx->runtime.active) return ctx->runtime.status;
    if(result == FOC_LOOP_CANCELLED && (hfoc->active_errors || hfoc->latched_errors)){
        ctx->runtime.detail = "Interrupted by fault; see active/latched flags";
    }
    if(ctx->calibration_enabled){
        ctx->calibration_enabled = 0;
        if(DRV8323_CSACALStop(&hfoc->hdrv8323) != DRV8323_OK){
            ctx->runtime.detail = "Could not exit current-sense amplifier calibration mode";
            ctx->runtime.error = FOC_ROUTINE_ERROR_CURRENT_SENSE;
            result = FOC_LOOP_ERROR;
            DRV8323_SetHighImpedance(&hfoc->hdrv8323);
        }
    }
    if(result == FOC_LOOP_COMPLETED && !ctx->candidate_valid){
        ctx->runtime.error = FOC_ROUTINE_ERROR_INVALID_RESULT;
        result = FOC_LOOP_ERROR;
    }
    if(result != FOC_LOOP_COMPLETED && result != FOC_LOOP_CANCELLED && result != FOC_LOOP_ERROR){
        ctx->runtime.error = FOC_ROUTINE_ERROR_INVALID_INPUT;
        result = FOC_LOOP_ERROR;
    }
    if(result == FOC_LOOP_COMPLETED){
        PhaseCurrentsTypeDef offsets = {ctx->candidate_a, ctx->candidate_b, ctx->candidate_c};
        FOC_ADC_SetPhaseCurrentOffsets(&offsets);
        hfoc->adc_calibrated = 1;
    }
    ctx->runtime.active = 0;
    ctx->runtime.status = result;
    FOC_ReportRoutineResult(hfoc, "Current sensor calibration", &ctx->runtime, (unsigned)ctx->stage);
    if(result == FOC_LOOP_ERROR) ctx->failed_stage = ctx->stage;
    ctx->stage = result == FOC_LOOP_COMPLETED ? FOC_CURRENT_CAL_DONE :
                 result == FOC_LOOP_CANCELLED ? FOC_CURRENT_CAL_CANCELLED : FOC_CURRENT_CAL_FAILED;
    return result;
}

/**
 * @brief Executes one bounded stage and collects at most one ADC sample per call.
 * @param hfoc FOC handle previously passed to Start.
 * @return IN_PROGRESS or the retained terminal result.
 * @note Collection remains synchronous with the existing FOC sample loop.
 */
FOC_LoopStatusTypeDef FOC_CurrentSensorCalibration_Update(FOC_HandleTypeDef *hfoc){
    FOC_CurrentCalibrationContextTypeDef *ctx = &hfoc->current_calibration;
    if(!ctx->runtime.active){
        if(ctx->stage == FOC_CURRENT_CAL_IDLE){
            ctx->runtime.error = FOC_ROUTINE_ERROR_NOT_STARTED;
            ctx->runtime.status = FOC_LOOP_ERROR;
        }
        return ctx->runtime.status;
    }
    uint32_t now = HAL_GetTick();
    if(FOC_RoutineElapsed(now, ctx->runtime.started_ms, CURRENT_SENSOR_CALIBRATION_TIMEOUT_MS)){
        ctx->runtime.error = FOC_ROUTINE_ERROR_TIMEOUT;
        return FOC_CurrentSensorCalibration_Finish(hfoc, FOC_LOOP_ERROR);
    }
    switch(ctx->stage){
        case FOC_CURRENT_CAL_START:
            Debug_SendTextResponse("Calibrating current sensor\n");
            ctx->stage = FOC_CURRENT_CAL_ENABLE;
            ctx->runtime.stage_started_ms = now;
            break;
        case FOC_CURRENT_CAL_ENABLE:
            if(FOC_RoutineElapsed(now, ctx->runtime.stage_started_ms, 1U)){
                /* Even a failed SPI reply may have applied the command. */
                ctx->calibration_enabled = 1;
                if(DRV8323_CSACALStart(&hfoc->hdrv8323) != DRV8323_OK){
                    ctx->runtime.detail = "Could not enable current-sense amplifier calibration mode";
                    ctx->runtime.error = FOC_ROUTINE_ERROR_CURRENT_SENSE;
                    return FOC_CurrentSensorCalibration_Finish(hfoc, FOC_LOOP_ERROR);
                }
                ctx->stage = FOC_CURRENT_CAL_COLLECT;
                ctx->runtime.stage_started_ms = now;
            }
            break;
        case FOC_CURRENT_CAL_COLLECT:
            if(FOC_RoutineElapsed(now, ctx->runtime.stage_started_ms, 1U)){
                ctx->sum_a += hfoc->adc_values.phase_current.a;
                ctx->sum_b += hfoc->adc_values.phase_current.b;
                ctx->sum_c += hfoc->adc_values.phase_current.c;
                if(++ctx->sample_count >= CURRENT_SENSOR_CALIBRATION_STEPS){
                    ctx->candidate_a = ctx->sum_a / CURRENT_SENSOR_CALIBRATION_STEPS;
                    ctx->candidate_b = ctx->sum_b / CURRENT_SENSOR_CALIBRATION_STEPS;
                    ctx->candidate_c = ctx->sum_c / CURRENT_SENSOR_CALIBRATION_STEPS;
                    ctx->candidate_valid = isfinite(ctx->candidate_a) &&
                                           isfinite(ctx->candidate_b) && isfinite(ctx->candidate_c);
                    if(!ctx->candidate_valid){
                        ctx->runtime.detail = "Current-sensor averaged offsets are non-finite";
                        ctx->runtime.error = FOC_ROUTINE_ERROR_INVALID_RESULT;
                        return FOC_CurrentSensorCalibration_Finish(hfoc, FOC_LOOP_ERROR);
                    }
                    ctx->calibration_enabled = 0;
                    if(DRV8323_CSACALStop(&hfoc->hdrv8323) != DRV8323_OK){
                        ctx->runtime.detail = "Could not disable current-sense amplifier calibration mode";
                        ctx->runtime.error = FOC_ROUTINE_ERROR_CURRENT_SENSE;
                        DRV8323_SetHighImpedance(&hfoc->hdrv8323);
                        return FOC_CurrentSensorCalibration_Finish(hfoc, FOC_LOOP_ERROR);
                    }
                    ctx->stage = FOC_CURRENT_CAL_REPORT;
                    ctx->runtime.stage_started_ms = now;
                }
            }
            break;
        case FOC_CURRENT_CAL_REPORT:
            if(FOC_RoutineElapsed(now, ctx->runtime.stage_started_ms, 1U)){
                ctx->stage = FOC_CURRENT_CAL_WAIT_FINISH;
                ctx->runtime.stage_started_ms = now;
            }
            break;
        case FOC_CURRENT_CAL_WAIT_FINISH:
            if(FOC_RoutineElapsed(now, ctx->runtime.stage_started_ms, 10U)){
                Debug_SendTextResponse("Current sensor calibration complete!\n");
                return FOC_CurrentSensorCalibration_Finish(hfoc, FOC_LOOP_COMPLETED);
            }
            break;
        default:
            ctx->runtime.error = FOC_ROUTINE_ERROR_INVALID_STAGE;
            return FOC_CurrentSensorCalibration_Finish(hfoc, FOC_LOOP_ERROR);
    }
    return FOC_LOOP_IN_PROGRESS;
}
