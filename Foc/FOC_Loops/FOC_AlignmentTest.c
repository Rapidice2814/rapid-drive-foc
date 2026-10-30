#include <math.h>
#include "FOC_Loops.h"
#include "FOC_Handle.h"
#include "Cordic.h"
#include "FOC_USB_Debug.h"

#define ALIGNMENT_TEST_TIMEOUT_MS 30000U
#define ALIGNMENT_TEST_STEPS 200U

/**
 * @brief Starts a fresh bidirectional alignment test without committing direction.
 * @param hfoc FOC handle owning the test.
 * @param magnitude Positive voltage magnitude used during rotation.
 */
void FOC_AlignmentTest_Start(FOC_HandleTypeDef *hfoc, float magnitude){
    if(hfoc->alignment_test.runtime.active) FOC_AlignmentTest_Finish(hfoc, FOC_LOOP_CANCELLED);
    hfoc->alignment_test = (FOC_AlignmentTestContextTypeDef){
        .runtime = {.active = 1, .status = FOC_LOOP_IN_PROGRESS, .started_ms = HAL_GetTick()},
        .stage = FOC_ALIGNMENT_TEST_START, .magnitude = magnitude,
        .candidate_direction = hfoc->flash_data.motor.direction
    };
    if(!isfinite(magnitude) || magnitude <= 0 || hfoc->flash_data.motor.pole_pairs == 0){
        hfoc->alignment_test.runtime.error = FOC_ROUTINE_ERROR_INVALID_INPUT;
        FOC_AlignmentTest_Finish(hfoc, FOC_LOOP_ERROR);
    }
}

/**
 * @brief Removes test voltage and commits direction only on successful completion.
 * @param hfoc FOC handle owning the test.
 * @param result Terminal status.
 * @return Retained terminal status; cleanup never enables motor outputs.
 */
FOC_LoopStatusTypeDef FOC_AlignmentTest_Finish(FOC_HandleTypeDef *hfoc, FOC_LoopStatusTypeDef result){
    FOC_AlignmentTestContextTypeDef *ctx = &hfoc->alignment_test;
    if(!ctx->runtime.active) return ctx->runtime.status;
    if(result == FOC_LOOP_CANCELLED && (hfoc->active_errors || hfoc->latched_errors)){
        ctx->runtime.detail = "Interrupted by fault; see active/latched flags";
    }
    FOC_SetPhaseVoltages(hfoc, (PhaseVoltagesTypeDef){0});
    if(result == FOC_LOOP_COMPLETED && !ctx->candidate_valid){
        ctx->runtime.error = FOC_ROUTINE_ERROR_INVALID_RESULT;
        result = FOC_LOOP_ERROR;
    }
    if(result != FOC_LOOP_COMPLETED && result != FOC_LOOP_ERROR && result != FOC_LOOP_CANCELLED){
        ctx->runtime.error = FOC_ROUTINE_ERROR_INVALID_INPUT;
        result = FOC_LOOP_ERROR;
    }
    if(result == FOC_LOOP_COMPLETED) hfoc->flash_data.motor.direction = ctx->candidate_direction;
    if(result == FOC_LOOP_ERROR) ctx->failed_stage = ctx->stage;
    ctx->runtime.active = 0;
    ctx->runtime.status = result;
    FOC_ReportRoutineResult(hfoc, "Encoder direction test", &ctx->runtime, (unsigned)ctx->stage);
    ctx->stage = result == FOC_LOOP_COMPLETED ? FOC_ALIGNMENT_TEST_DONE :
                 result == FOC_LOOP_CANCELLED ? FOC_ALIGNMENT_TEST_CANCELLED : FOC_ALIGNMENT_TEST_FAILED;
    return result;
}

/**
 * @brief Advances one sweep sample or stage, retaining terminal status until Start.
 * @param hfoc FOC handle previously started for alignment testing.
 * @return IN_PROGRESS or a terminal result.
 */
FOC_LoopStatusTypeDef FOC_AlignmentTest_Update(FOC_HandleTypeDef *hfoc){
    FOC_AlignmentTestContextTypeDef *ctx = &hfoc->alignment_test;
    if(!ctx->runtime.active){
        if(ctx->stage == FOC_ALIGNMENT_TEST_IDLE){
            ctx->runtime.status = FOC_LOOP_ERROR;
            ctx->runtime.error = FOC_ROUTINE_ERROR_NOT_STARTED;
        }
        return ctx->runtime.status;
    }
    uint32_t now = HAL_GetTick();
    if(FOC_RoutineElapsed(now, ctx->runtime.started_ms, ALIGNMENT_TEST_TIMEOUT_MS)){
        ctx->runtime.error = FOC_ROUTINE_ERROR_TIMEOUT;
        return FOC_AlignmentTest_Finish(hfoc, FOC_LOOP_ERROR);
    }
    if(!FOC_RoutineElapsed(now, ctx->runtime.stage_started_ms, ctx->wait_ms)) return FOC_LOOP_IN_PROGRESS;
    switch(ctx->stage){
        case FOC_ALIGNMENT_TEST_START:
            ctx->sample_index = 0;
            ctx->reference_angle = ctx->electrical_angle = 0;
            ctx->error_cw = ctx->error_ccw = 0;
            ctx->direction = ctx->dir_swapped = 0;
            FOC_SetPhaseVoltages(hfoc, InvClarke_transform((ABVoltagesTypeDef){0.3f, 0}));
            ctx->stage = FOC_ALIGNMENT_TEST_ROTATE;
            ctx->wait_ms = 10;
            break;
        case FOC_ALIGNMENT_TEST_ROTATE: {
            if(!isfinite(hfoc->encoder_angle_electrical) || !isfinite(hfoc->encoder_speed_electrical)){
                ctx->runtime.error = FOC_ROUTINE_ERROR_INVALID_RESULT;
                ctx->runtime.detail = "Encoder electrical angle or speed is non-finite";
                return FOC_AlignmentTest_Finish(hfoc, FOC_LOOP_ERROR);
            }
            float diff = ctx->candidate_direction == 1 ?
                2 * M_PIF - ctx->electrical_angle - hfoc->encoder_angle_electrical :
                ctx->electrical_angle - hfoc->encoder_angle_electrical;
            normalize_angle_pm_pi(&diff);
            if(!ctx->direction){
                ctx->reference_angle += 2 * M_PIF / ALIGNMENT_TEST_STEPS;
                ctx->error_cw += fabsf(diff) / ALIGNMENT_TEST_STEPS;
            }else{
                ctx->reference_angle -= 2 * M_PIF / ALIGNMENT_TEST_STEPS;
                ctx->error_ccw += fabsf(diff) / ALIGNMENT_TEST_STEPS;
            }
            normalize_angle_0_2pi(&ctx->reference_angle);
            ctx->electrical_angle = ctx->reference_angle * hfoc->flash_data.motor.pole_pairs;
            normalize_angle_0_2pi(&ctx->electrical_angle);
            float c, s;
            Cordic_CalculateSinCos(ctx->electrical_angle, &c, &s);
            FOC_SetPhaseVoltages(hfoc, InvClarke_transform((ABVoltagesTypeDef){ctx->magnitude * c, ctx->magnitude * s}));
            ctx->wait_ms = 10;
            if(++ctx->sample_index > ALIGNMENT_TEST_STEPS){
                ctx->sample_index = 0;
                ctx->wait_ms = 100;
                if(!ctx->direction){
                    ctx->dir_swapped = hfoc->encoder_speed_electrical < 0;
                    ctx->direction = 1;
                }else{
                    if(hfoc->encoder_speed_electrical > 0) ctx->candidate_direction = ctx->dir_swapped;
                    ctx->direction = 0;
                    ctx->stage = FOC_ALIGNMENT_TEST_VALIDATE;
                }
            }
            break;
        }
        case FOC_ALIGNMENT_TEST_VALIDATE:
            FOC_SetPhaseVoltages(hfoc, (PhaseVoltagesTypeDef){0});
            Debug_SendTextResponse("Alignment test: CW=%d, CCW=%d, direction=%d\n",
                (int)(ctx->error_cw * 1000), (int)(ctx->error_ccw * 1000), ctx->candidate_direction);
            if(ctx->error_cw < 0.3f && ctx->error_ccw < 0.3f){
                ctx->candidate_valid = 1;
                ctx->stage = FOC_ALIGNMENT_TEST_WAIT_FINISH;
                ctx->wait_ms = 100;
            }else if(ctx->attempt < 3){
                ctx->attempt++;
                ctx->stage = FOC_ALIGNMENT_TEST_START;
                ctx->wait_ms = 1;
            }else{
                ctx->runtime.error = FOC_ROUTINE_ERROR_INVALID_RESULT;
                ctx->runtime.detail = "CW/CCW tracking error exceeds 0.3 rad after direction retries";
                return FOC_AlignmentTest_Finish(hfoc, FOC_LOOP_ERROR);
            }
            break;
        case FOC_ALIGNMENT_TEST_WAIT_FINISH:
            return FOC_AlignmentTest_Finish(hfoc, FOC_LOOP_COMPLETED);
        default:
            ctx->runtime.error = FOC_ROUTINE_ERROR_INVALID_STAGE;
            return FOC_AlignmentTest_Finish(hfoc, FOC_LOOP_ERROR);
    }
    ctx->runtime.stage_started_ms = now;
    return FOC_LOOP_IN_PROGRESS;
}
