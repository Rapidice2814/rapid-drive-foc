#include <math.h>
#include "FOC_Loops.h"
#include "FOC_Handle.h"
#include "FOC_USB_Debug.h"

#define MEASUREMENT_VOLTAGE 1.0f
#define IDENTIFICATION_TIMEOUT_MS 60000U
#define IDENTIFICATION_MAX_ITERATIONS 4096U

/** @brief Models the current response of the original RL fitting algorithm. */
static float model_current(float t, float r, float l, float voltage){
    return (voltage / r) * (1.0 - exp(-r * t / l));
}

/** @brief Computes bounded squared error over one captured response. */
static float compute_error(const float *predicted, const float *measured){
    float error = 0;
    for(unsigned i = 0; i < FOC_IDENTIFICATION_SAMPLES; i++){
        float diff = predicted[i] - measured[i];
        error += diff * diff;
    }
    return error;
}

/** @brief Computes the spread of the three phase-pair estimates. */
static float estimate_spread(const float *values){
    float min = values[0], max = values[0];
    for(unsigned i = 1; i < 3; i++){
        if(values[i] < min) min = values[i];
        if(values[i] > max) max = values[i];
    }
    return max > 0 ? (max - min) / max * 100.0f : 100.0f;
}

/** @brief Selects a named stage and its non-blocking settling interval. */
static void identification_stage(FOC_IdentificationContextTypeDef *ctx,
                                 FOC_IdentificationStageTypeDef stage, uint32_t now, uint32_t wait_ms){
    ctx->stage = stage;
    ctx->runtime.stage_started_ms = now;
    ctx->wait_ms = wait_ms;
}

/** @brief Applies the original test voltage to the selected phase. */
static void identification_voltage(FOC_HandleTypeDef *hfoc, uint8_t selector){
    PhaseVoltagesTypeDef voltage = {0};
    float excitation = fminf(MEASUREMENT_VOLTAGE, hfoc->flash_data.limits.max_dq_voltage);
    if(selector == 0) voltage.a = excitation;
    else if(selector == 1) voltage.b = excitation;
    else voltage.c = excitation;
    FOC_SetPhaseVoltages(hfoc, voltage);
}

/**
 * @brief Starts fresh motor identification without modifying accepted R/L values.
 * @param hfoc FOC handle owning all sample buffers and fitting state.
 */
void FOC_MotorIdentification_Start(FOC_HandleTypeDef *hfoc){
    if(hfoc->identification.runtime.active) FOC_MotorIdentification_Finish(hfoc, FOC_LOOP_CANCELLED);
    hfoc->identification = (FOC_IdentificationContextTypeDef){
        .runtime = {.active = 1, .status = FOC_LOOP_IN_PROGRESS, .started_ms = HAL_GetTick()},
        .stage = FOC_IDENT_START
    };
}

/**
 * @brief Removes test voltage and commits validated phase R/L only on success.
 * @param hfoc FOC handle owning the identification operation.
 * @param result Terminal status.
 * @return Retained terminal result; repeated cleanup has no hardware effects.
 */
FOC_LoopStatusTypeDef FOC_MotorIdentification_Finish(FOC_HandleTypeDef *hfoc, FOC_LoopStatusTypeDef result){
    FOC_IdentificationContextTypeDef *ctx = &hfoc->identification;
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
    if(result == FOC_LOOP_COMPLETED){
        hfoc->flash_data.motor.phase_resistance = ctx->candidate_r;
        hfoc->flash_data.motor.phase_inductance = ctx->candidate_l;
    }
    if(result == FOC_LOOP_ERROR) ctx->failed_stage = ctx->stage;
    ctx->runtime.active = 0;
    ctx->runtime.status = result;
    FOC_ReportRoutineResult(hfoc, "Motor R/L identification", &ctx->runtime, (unsigned)ctx->stage);
    ctx->stage = result == FOC_LOOP_COMPLETED ? FOC_IDENT_DONE :
                 result == FOC_LOOP_CANCELLED ? FOC_IDENT_CANCELLED : FOC_IDENT_FAILED;
    return result;
}

/**
 * @brief Advances one capture sample, model sample, or fitting stage per call.
 * @param hfoc FOC handle previously started for identification.
 * @return IN_PROGRESS or retained terminal status.
 * @note Preserves the phase excitation, 25-sample capture, perturbations, gradient
 *       scales, convergence thresholds, and three-phase spread check.
 */
FOC_LoopStatusTypeDef FOC_MotorIdentification_Update(FOC_HandleTypeDef *hfoc){
    FOC_IdentificationContextTypeDef *ctx = &hfoc->identification;
    if(!ctx->runtime.active){
        if(ctx->stage == FOC_IDENT_IDLE){
            ctx->runtime.status = FOC_LOOP_ERROR;
            ctx->runtime.error = FOC_ROUTINE_ERROR_NOT_STARTED;
        }
        return ctx->runtime.status;
    }
    uint32_t now = HAL_GetTick();
    if(FOC_RoutineElapsed(now, ctx->runtime.started_ms, IDENTIFICATION_TIMEOUT_MS)){
        ctx->runtime.error = FOC_ROUTINE_ERROR_TIMEOUT;
        return FOC_MotorIdentification_Finish(hfoc, FOC_LOOP_ERROR);
    }
    if(ctx->selector > 2 || ctx->capture_index > FOC_IDENTIFICATION_SAMPLES + 1 ||
       ctx->model_index > FOC_IDENTIFICATION_SAMPLES + 1){
        ctx->runtime.error = FOC_ROUTINE_ERROR_INVALID_STAGE;
        return FOC_MotorIdentification_Finish(hfoc, FOC_LOOP_ERROR);
    }
    if(!FOC_RoutineElapsed(now, ctx->runtime.stage_started_ms, ctx->wait_ms)) return FOC_LOOP_IN_PROGRESS;
    switch(ctx->stage){
        case FOC_IDENT_START:
            Debug_SendTextResponse("Starting motor identification attempt %d\n", ctx->attempt + 1);
            FOC_SetPhaseVoltages(hfoc, (PhaseVoltagesTypeDef){0});
            ctx->selector = 0;
            identification_stage(ctx, FOC_IDENT_PREPARE, now, 100);
            break;
        case FOC_IDENT_PREPARE:
            ctx->estimated_r = 0;
            ctx->estimated_l = 0;
            ctx->capture_index = ctx->model_index = 0;
            ctx->iterations = 0;
            ctx->fit_stage = FOC_IDENT_FIT_BASELINE;
            for(unsigned i = 0; i < FOC_IDENTIFICATION_SAMPLES; i++){
                ctx->time[i] = (float)i / CURRENT_LOOP_FREQUENCY;
                ctx->measured[i] = 0;
            }
            identification_voltage(hfoc, ctx->selector);
            identification_stage(ctx, FOC_IDENT_REMOVE_VOLTAGE, now, 100);
            break;
        case FOC_IDENT_REMOVE_VOLTAGE:
            FOC_SetPhaseVoltages(hfoc, (PhaseVoltagesTypeDef){0});
            identification_stage(ctx, FOC_IDENT_WAIT_CAPTURE, now, 100);
            break;
        case FOC_IDENT_WAIT_CAPTURE:
            identification_stage(ctx, FOC_IDENT_CAPTURE, now, 0);
            break;
        case FOC_IDENT_CAPTURE:
            if(ctx->capture_index == 0){
                identification_voltage(hfoc, ctx->selector);
                ctx->capture_index++;
            }else if(ctx->capture_index <= FOC_IDENTIFICATION_SAMPLES){
                float current = ctx->selector == 0 ? hfoc->adc_values.phase_current.a :
                                ctx->selector == 1 ? hfoc->adc_values.phase_current.b : hfoc->adc_values.phase_current.c;
                if(!isfinite(current)){ctx->runtime.detail = "Measured phase current is non-finite"; goto invalid_result;}
                ctx->measured[ctx->capture_index++ - 1] = current;
            }else{
                ctx->capture_index = 0;
                FOC_SetPhaseVoltages(hfoc, (PhaseVoltagesTypeDef){0});
                identification_stage(ctx, FOC_IDENT_SELECT_MODEL, now, 0);
            }
            break;
        case FOC_IDENT_SELECT_MODEL:
            if(ctx->estimated_r == 0){
                /* Seed from the measured RL step, with no assumed motor model. */
                float first = ctx->measured[1], second = ctx->measured[2];
                float decay = second / first - 1.0f;
                if(!isfinite(decay) || first <= 0 || decay <= 0 || decay >= 1){
                    ctx->runtime.detail = "RL response cannot seed a positive model";
                    goto invalid_result;
                }
                ctx->estimated_r = fminf(MEASUREMENT_VOLTAGE, hfoc->flash_data.limits.max_dq_voltage) * (1.0f - decay) / first;
                ctx->estimated_l = -ctx->estimated_r / (CURRENT_LOOP_FREQUENCY * logf(decay));
            }
            ctx->model_r = ctx->estimated_r;
            ctx->model_l = ctx->estimated_l;
            switch(ctx->fit_stage){
                case FOC_IDENT_FIT_BASELINE:
                    ctx->fit_stage = FOC_IDENT_FIT_INDUCTANCE;
                    break;
                case FOC_IDENT_FIT_INDUCTANCE:
                    ctx->baseline_error = ctx->error;
                    ctx->model_l += 1e-6f;
                    ctx->fit_stage = FOC_IDENT_FIT_RESISTANCE;
                    break;
                case FOC_IDENT_FIT_RESISTANCE:
                    ctx->inductance_error = ctx->error;
                    ctx->model_r += 1e-2f;
                    ctx->fit_stage = FOC_IDENT_FIT_APPLY;
                    break;
                case FOC_IDENT_FIT_APPLY: {
                    float gradient_l = (ctx->inductance_error - ctx->baseline_error) * 1e-5f;
                    float gradient_r = (ctx->error - ctx->baseline_error) * 1e-3f;
                    ctx->estimated_l -= gradient_l;
                    ctx->estimated_r -= gradient_r;
                    if(!isfinite(ctx->estimated_r) || ctx->estimated_r <= 0 ||
                       !isfinite(ctx->estimated_l) || ctx->estimated_l <= 0){ctx->runtime.detail = "R/L fit produced nonpositive or non-finite estimates"; goto invalid_result;}
                    ctx->fit_stage = FOC_IDENT_FIT_BASELINE;
                    if(ctx->error < 0.5f || (fabsf(gradient_l) < 1e-7f && fabsf(gradient_r) < 1e-4f)){
                        ctx->resistance[ctx->selector] = ctx->estimated_r;
                        ctx->inductance[ctx->selector] = ctx->estimated_l;
                        identification_stage(ctx, FOC_IDENT_NEXT_PHASE, now, 10);
                    }else if(++ctx->iterations >= IDENTIFICATION_MAX_ITERATIONS){
                        ctx->runtime.error = FOC_ROUTINE_ERROR_TIMEOUT;
                        return FOC_MotorIdentification_Finish(hfoc, FOC_LOOP_ERROR);
                    }
                    return FOC_LOOP_IN_PROGRESS;
                }
                default:
                    ctx->runtime.error = FOC_ROUTINE_ERROR_INVALID_STAGE;
                    return FOC_MotorIdentification_Finish(hfoc, FOC_LOOP_ERROR);
            }
            ctx->model_index = 0;
            identification_stage(ctx, FOC_IDENT_EVALUATE_MODEL, now, 0);
            break;
        case FOC_IDENT_EVALUATE_MODEL:
            if(!isfinite(ctx->model_r) || ctx->model_r <= 0 ||
               !isfinite(ctx->model_l) || ctx->model_l <= 0){ctx->runtime.detail = "R/L candidate model has invalid resistance/inductance"; goto invalid_result;}
            if(ctx->model_index < FOC_IDENTIFICATION_SAMPLES){
                float predicted = model_current(ctx->time[ctx->model_index], ctx->model_r, ctx->model_l, fminf(MEASUREMENT_VOLTAGE, hfoc->flash_data.limits.max_dq_voltage));
                if(!isfinite(predicted)){ctx->runtime.detail = "R/L model current prediction is non-finite"; goto invalid_result;}
                ctx->predicted[ctx->model_index++] = predicted;
            }else if(ctx->model_index == FOC_IDENTIFICATION_SAMPLES){
                ctx->error = compute_error(ctx->predicted, ctx->measured);
                if(!isfinite(ctx->error)){ctx->runtime.detail = "R/L model fit error is non-finite"; goto invalid_result;}
                ctx->model_index++;
            }else{
                ctx->model_index = 0;
                identification_stage(ctx, FOC_IDENT_SELECT_MODEL, now, 0);
            }
            break;
        case FOC_IDENT_NEXT_PHASE:
            if(ctx->selector < 2){
                ctx->selector++;
                identification_stage(ctx, FOC_IDENT_PREPARE, now, 0);
            }else{
                identification_stage(ctx, FOC_IDENT_VALIDATE, now, 0);
            }
            break;
        case FOC_IDENT_VALIDATE:
            if(estimate_spread(ctx->resistance) < 30 && estimate_spread(ctx->inductance) < 30){
                ctx->candidate_r = (ctx->resistance[0] + ctx->resistance[1] + ctx->resistance[2]) * (2.0f / 9.0f);
                ctx->candidate_l = (ctx->inductance[0] + ctx->inductance[1] + ctx->inductance[2]) * (2.0f / 9.0f);
                ctx->candidate_valid = isfinite(ctx->candidate_r) && ctx->candidate_r > 0 &&
                                       isfinite(ctx->candidate_l) && ctx->candidate_l > 0;
                if(!ctx->candidate_valid){ctx->runtime.detail = "Final R/L averages are nonpositive or non-finite"; goto invalid_result;}
                identification_stage(ctx, FOC_IDENT_WAIT_FINISH, now, 10);
            }else if(ctx->attempt < 3){
                ctx->attempt++;
                identification_stage(ctx, FOC_IDENT_START, now, 10);
            }else {ctx->runtime.detail = "Phase R/L estimates disagree after retries"; goto invalid_result;}
            break;
        case FOC_IDENT_WAIT_FINISH:
            return FOC_MotorIdentification_Finish(hfoc, FOC_LOOP_COMPLETED);
        default:
            ctx->runtime.error = FOC_ROUTINE_ERROR_INVALID_STAGE;
            return FOC_MotorIdentification_Finish(hfoc, FOC_LOOP_ERROR);
    }
    return FOC_LOOP_IN_PROGRESS;
invalid_result:
    if(!ctx->runtime.detail) ctx->runtime.detail = "R/L result failed validation";
    ctx->runtime.error = FOC_ROUTINE_ERROR_INVALID_RESULT;
    return FOC_MotorIdentification_Finish(hfoc, FOC_LOOP_ERROR);
}
