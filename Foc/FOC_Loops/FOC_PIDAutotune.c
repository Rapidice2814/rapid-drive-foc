#include <math.h>
#include "FOC_Loops.h"
#include "FOC_Handle.h"
#include "FOC_USB_Debug.h"

/**
 * @brief Starts a fresh, non-blocking current PI tuning operation.
 * @param hfoc FOC handle owning the workspace and motor parameters.
 */
void FOC_PIDAutotune_Start(FOC_HandleTypeDef *hfoc){
    if(hfoc->pid_autotune.runtime.active) FOC_PIDAutotune_Finish(hfoc, FOC_LOOP_CANCELLED);
    hfoc->pid_autotune = (FOC_PIDAutotuneContextTypeDef){
        .runtime = {.active = 1, .status = FOC_LOOP_IN_PROGRESS, .started_ms = HAL_GetTick()},
        .stage = FOC_PID_TUNE_START
    };
}

/**
 * @brief Retains a terminal tuning result, committing validated gains on success.
 * @param hfoc FOC handle owning the tuning operation.
 * @param result Terminal status; cancellation leaves existing gains unchanged.
 * @return Retained terminal status.
 */
FOC_LoopStatusTypeDef FOC_PIDAutotune_Finish(FOC_HandleTypeDef *hfoc, FOC_LoopStatusTypeDef result){
    FOC_PIDAutotuneContextTypeDef *ctx = &hfoc->pid_autotune;
    if(!ctx->runtime.active) return ctx->runtime.status;
    if(result == FOC_LOOP_CANCELLED && (hfoc->active_errors || hfoc->latched_errors)){
        ctx->runtime.detail = "Interrupted by fault; see active/latched flags";
    }
    if(result == FOC_LOOP_COMPLETED && !ctx->candidate_valid){
        ctx->runtime.error = FOC_ROUTINE_ERROR_INVALID_RESULT;
        result = FOC_LOOP_ERROR;
    }
    if(result != FOC_LOOP_COMPLETED && result != FOC_LOOP_ERROR && result != FOC_LOOP_CANCELLED){
        ctx->runtime.error = FOC_ROUTINE_ERROR_INVALID_INPUT;
        result = FOC_LOOP_ERROR;
    }
    if(result == FOC_LOOP_COMPLETED){
        hfoc->flash_data.controller.PID_gains_d.Kp = ctx->candidate_kp;
        hfoc->flash_data.controller.PID_gains_q.Kp = ctx->candidate_kp;
        hfoc->flash_data.controller.PID_gains_d.Ki = ctx->candidate_ki;
        hfoc->flash_data.controller.PID_gains_q.Ki = ctx->candidate_ki;
    }
    if(result == FOC_LOOP_ERROR) ctx->failed_stage = ctx->stage;
    ctx->runtime.active = 0;
    ctx->runtime.status = result;
    FOC_ReportRoutineResult(hfoc, "Current PI tuning", &ctx->runtime, (unsigned)ctx->stage);
    ctx->stage = result == FOC_LOOP_COMPLETED ? FOC_PID_TUNE_DONE :
                 result == FOC_LOOP_CANCELLED ? FOC_PID_TUNE_CANCELLED : FOC_PID_TUNE_FAILED;
    return result;
}

/**
 * @brief Executes one tuning stage without changing gains until validation passes.
 * @param hfoc FOC handle previously started for tuning.
 * @return IN_PROGRESS or the retained terminal result.
 */
FOC_LoopStatusTypeDef FOC_PIDAutotune_Update(FOC_HandleTypeDef *hfoc){
    FOC_PIDAutotuneContextTypeDef *ctx = &hfoc->pid_autotune;
    if(!ctx->runtime.active){
        if(ctx->stage == FOC_PID_TUNE_IDLE){
            ctx->runtime.status = FOC_LOOP_ERROR;
            ctx->runtime.error = FOC_ROUTINE_ERROR_NOT_STARTED;
        }
        return ctx->runtime.status;
    }
    uint32_t now = HAL_GetTick();
    if(FOC_RoutineElapsed(now, ctx->runtime.started_ms, 1000U)){
        ctx->runtime.error = FOC_ROUTINE_ERROR_TIMEOUT;
        return FOC_PIDAutotune_Finish(hfoc, FOC_LOOP_ERROR);
    }
    switch(ctx->stage){
        case FOC_PID_TUNE_START:
            Debug_SendTextResponse("Starting PID Autotune\n");
            ctx->runtime.stage_started_ms = now;
            ctx->stage = FOC_PID_TUNE_COMPUTE;
            break;
        case FOC_PID_TUNE_COMPUTE:
            if(FOC_RoutineElapsed(now, ctx->runtime.stage_started_ms, 10U)){
                float bandwidth = hfoc->flash_data.controller.current_control_bandwidth;
                float r = hfoc->flash_data.motor.phase_resistance;
                float l = hfoc->flash_data.motor.phase_inductance;
                if(!isfinite(bandwidth) || bandwidth < CONFIG_ABS_BANDWIDTH_MIN || bandwidth > CONFIG_ABS_BANDWIDTH_MAX ||
                   !isfinite(r) || r <= 0 || !isfinite(l) || l <= 0){
                    ctx->runtime.error = FOC_ROUTINE_ERROR_INVALID_INPUT;
                    ctx->runtime.detail = "Invalid R/L or current-control bandwidth; measure motor parameters first";
                    return FOC_PIDAutotune_Finish(hfoc, FOC_LOOP_ERROR);
                }
                ctx->candidate_kp = l * bandwidth;
                ctx->candidate_ki = r * bandwidth;
                ctx->candidate_valid = isfinite(ctx->candidate_kp) && isfinite(ctx->candidate_ki);
                if(!ctx->candidate_valid){
                    ctx->runtime.error = FOC_ROUTINE_ERROR_INVALID_RESULT;
                    ctx->runtime.detail = "Computed PI gains are non-finite";
                    return FOC_PIDAutotune_Finish(hfoc, FOC_LOOP_ERROR);
                }
                Debug_SendTextResponse("PID gains tuned successfully\n");
                return FOC_PIDAutotune_Finish(hfoc, FOC_LOOP_COMPLETED);
            }
            break;
        default:
            ctx->runtime.error = FOC_ROUTINE_ERROR_INVALID_STAGE;
            return FOC_PIDAutotune_Finish(hfoc, FOC_LOOP_ERROR);
    }
    return FOC_LOOP_IN_PROGRESS;
}
