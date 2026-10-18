#include <math.h>
#include "FOC_Loops.h"
#include "FOC_Handle.h"
#include "FOC_USB_Debug.h"
#include "Cordic.h"

#define ALIGNMENT_STEPS 10U
#define STARTING_PHASE 0.2f
#define PHASE_STEP (STARTING_PHASE / ALIGNMENT_STEPS)
#define ALIGNMENT_SETTLE_MS 100U
#define ALIGNMENT_TIMEOUT_MS 5000U

/**
 * @brief Starts a fresh encoder alignment; updates execute in the sample loop.
 * @param hfoc FOC handle owning the workspace.
 * @param magnitude Positive alignment voltage magnitude.
 * @note Accepted configuration is unchanged until successful completion.
 */
void FOC_Alignment_Start(FOC_HandleTypeDef *hfoc, float magnitude){
    if(hfoc->alignment.runtime.active){
        FOC_Alignment_Finish(hfoc, FOC_LOOP_CANCELLED);
    }
    hfoc->alignment = (FOC_AlignmentContextTypeDef){
        .runtime = {.started_ms = HAL_GetTick(), .status = FOC_LOOP_IN_PROGRESS, .active = 1},
        .stage = FOC_ALIGNMENT_START,
        .phase = STARTING_PHASE,
        .magnitude = magnitude
    };
    if(!isfinite(magnitude) || magnitude <= 0.0f){
        hfoc->alignment.runtime.error = FOC_ROUTINE_ERROR_INVALID_INPUT;
        FOC_Alignment_Finish(hfoc, FOC_LOOP_ERROR);
    }
}

/**
 * @brief Removes alignment voltage and retains a terminal result exactly once.
 * @param hfoc FOC handle owning the routine.
 * @param result Completion, error, or cancellation; never IN_PROGRESS.
 * @return Retained terminal status.
 * @note Cleanup never enables the gate driver or restores motion.
 */
FOC_LoopStatusTypeDef FOC_Alignment_Finish(FOC_HandleTypeDef *hfoc, FOC_LoopStatusTypeDef result){
    FOC_AlignmentContextTypeDef *ctx = &hfoc->alignment;
    if(!ctx->runtime.active) return ctx->runtime.status;
    FOC_SetPhaseVoltages(hfoc, (PhaseVoltagesTypeDef){0});
    if(result == FOC_LOOP_COMPLETED && !ctx->candidate_valid){
        ctx->runtime.error = FOC_ROUTINE_ERROR_INVALID_RESULT;
        result = FOC_LOOP_ERROR;
    }
    if(result != FOC_LOOP_COMPLETED && result != FOC_LOOP_CANCELLED && result != FOC_LOOP_ERROR){
        ctx->runtime.error = FOC_ROUTINE_ERROR_INVALID_INPUT;
        result = FOC_LOOP_ERROR;
    }
    if(result == FOC_LOOP_COMPLETED){
        hfoc->flash_data.encoder.mechanical_offset = ctx->candidate_offset;
        hfoc->flash_data.encoder.raw_zero = ctx->candidate_raw_zero;
        hfoc->flash_data.encoder.offset_valid = 1;
    }
    ctx->runtime.active = 0;
    ctx->runtime.status = result;
    if(result == FOC_LOOP_ERROR) ctx->failed_stage = ctx->stage;
    ctx->stage = result == FOC_LOOP_COMPLETED ? FOC_ALIGNMENT_DONE :
                 result == FOC_LOOP_CANCELLED ? FOC_ALIGNMENT_CANCELLED : FOC_ALIGNMENT_FAILED;
    return result;
}

/**
 * @brief Executes one bounded alignment stage without blocking or auto-restarting.
 * @param hfoc FOC handle previously passed to Start.
 * @return IN_PROGRESS or the retained terminal result.
 */
FOC_LoopStatusTypeDef FOC_Alignment_Update(FOC_HandleTypeDef *hfoc){
    FOC_AlignmentContextTypeDef *ctx = &hfoc->alignment;
    if(!ctx->runtime.active){
        if(ctx->stage == FOC_ALIGNMENT_IDLE){
            ctx->runtime.error = FOC_ROUTINE_ERROR_NOT_STARTED;
            ctx->runtime.status = FOC_LOOP_ERROR;
        }
        return ctx->runtime.status;
    }
    uint32_t now = HAL_GetTick();
    if(FOC_RoutineElapsed(now, ctx->runtime.started_ms, ALIGNMENT_TIMEOUT_MS)){
        ctx->runtime.error = FOC_ROUTINE_ERROR_TIMEOUT;
        return FOC_Alignment_Finish(hfoc, FOC_LOOP_ERROR);
    }
    switch(ctx->stage){
        case FOC_ALIGNMENT_START:
            Debug_SendTextResponse("Starting alignment with %dmV\n", (int)(ctx->magnitude * 1000));
            ctx->stage = FOC_ALIGNMENT_SWEEP;
            ctx->runtime.stage_started_ms = now;
            break;
        case FOC_ALIGNMENT_SWEEP:
            if(!FOC_RoutineElapsed(now, ctx->runtime.stage_started_ms, ALIGNMENT_SETTLE_MS)) break;
            if(ctx->sweep_index < ALIGNMENT_STEPS){
                float cos_phase, sin_phase;
                Cordic_CalculateSinCos(ctx->phase, &cos_phase, &sin_phase);
                ABVoltagesTypeDef voltage = {cos_phase * ctx->magnitude, sin_phase * ctx->magnitude};
                FOC_SetPhaseVoltages(hfoc, InvClarke_transform(voltage));
                ctx->phase = -ctx->phase - (ctx->phase > 0 ? -PHASE_STEP : PHASE_STEP);
                ctx->sweep_index++;
            }else if(ctx->sweep_index == ALIGNMENT_STEPS){
                FOC_SetPhaseVoltages(hfoc, InvClarke_transform((ABVoltagesTypeDef){ctx->magnitude, 0.0f}));
                ctx->sweep_index++;
            }else{
                ctx->stage = FOC_ALIGNMENT_SET_ZERO;
            }
            ctx->runtime.stage_started_ms = now;
            break;
        case FOC_ALIGNMENT_SET_ZERO:
            if(FOC_RoutineElapsed(now, ctx->runtime.stage_started_ms, ALIGNMENT_SETTLE_MS)){
                /* The legacy encoder helper writes configuration. Stage its result
                   and restore the accepted values until the routine completes. */
                float previous_offset = hfoc->flash_data.encoder.mechanical_offset;
                float previous_raw_zero = hfoc->flash_data.encoder.raw_zero;
                uint8_t previous_valid = hfoc->flash_data.encoder.offset_valid;
                FOC_StatusTypeDef result = FOC_SetEncoderZero(hfoc);
                ctx->candidate_offset = hfoc->flash_data.encoder.mechanical_offset;
                ctx->candidate_raw_zero = hfoc->flash_data.encoder.raw_zero;
                ctx->candidate_valid = result == FOC_OK && isfinite(ctx->candidate_offset);
                hfoc->flash_data.encoder.mechanical_offset = previous_offset;
                hfoc->flash_data.encoder.raw_zero = previous_raw_zero;
                hfoc->flash_data.encoder.offset_valid = previous_valid;
                if(!ctx->candidate_valid){
                    ctx->runtime.error = FOC_ROUTINE_ERROR_ENCODER;
                    Debug_SendTextResponse("Error setting encoder zero\n");
                    return FOC_Alignment_Finish(hfoc, FOC_LOOP_ERROR);
                }
                ctx->stage = FOC_ALIGNMENT_RELEASE;
                ctx->runtime.stage_started_ms = now;
            }
            break;
        case FOC_ALIGNMENT_RELEASE:
            if(FOC_RoutineElapsed(now, ctx->runtime.stage_started_ms, ALIGNMENT_SETTLE_MS)){
                FOC_SetPhaseVoltages(hfoc, (PhaseVoltagesTypeDef){0});
                ctx->stage = FOC_ALIGNMENT_WAIT_RELEASE;
                ctx->runtime.stage_started_ms = now;
            }
            break;
        case FOC_ALIGNMENT_WAIT_RELEASE:
            if(FOC_RoutineElapsed(now, ctx->runtime.stage_started_ms, ALIGNMENT_SETTLE_MS)){
                Debug_SendTextResponse("Encoder zero set successfully\n");
                Debug_SendTextResponse("Encoder angle mechanical offset: %dmRad\n", (int)(ctx->candidate_offset * 1000));
                return FOC_Alignment_Finish(hfoc, FOC_LOOP_COMPLETED);
            }
            break;
        default:
            ctx->runtime.error = FOC_ROUTINE_ERROR_INVALID_STAGE;
            return FOC_Alignment_Finish(hfoc, FOC_LOOP_ERROR);
    }
    return FOC_LOOP_IN_PROGRESS;
}
