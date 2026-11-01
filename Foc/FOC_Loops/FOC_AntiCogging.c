#include <math.h>
#include <stdio.h>
#include "FOC_Loops.h"
#include "FOC_Handle.h"
#include "FOC_USB_Debug.h"
#include "Utils.h"
#include "FOC_Config.h"
#include "FOC_Diagnostics.h"

#define ANTICOG_TIMEOUT_MS 1800000U /* 30 minutes per configured revolution. */
#define ANTICOG_POINT_TIMEOUT_MS 10000U

/* Reset all statistics after each move/rejected window. No previous-bin samples survive. */
static void anticog_reset_window(FOC_AntiCoggingContextTypeDef *ctx, uint32_t now){
    ctx->window_started_ms = now;
    ctx->window_samples = 0;
    ctx->current_mean = ctx->current_m2 = ctx->speed_sum = 0;
    ctx->window_error_max = ctx->window_command_max = 0;
}

static float anticog_target(const FOC_AntiCoggingContextTypeDef *ctx){
    unsigned step = ctx->direction ?
        (ctx->revolutions - 1 - ctx->revolution) * NUMBER_OF_ANTICOG_MEASUREMENTS +
        NUMBER_OF_ANTICOG_MEASUREMENTS - 1 - ctx->sample_index :
        ctx->revolution * NUMBER_OF_ANTICOG_MEASUREMENTS + ctx->sample_index;
    return ctx->turn_base + step * ANTICOG_ANGLE_STEP;
}

/* Return the failed acceptance criterion; NULL means the entire window is stable. */
static const char *anticog_window_reason(FOC_HandleTypeDef *h){
    FOC_AntiCoggingContextTypeDef *c = &h->anticogging;
    c->last_error = c->window_error_max;
    c->last_span = c->position_max - c->position_min;
    c->last_speed = c->window_samples ? c->speed_sum / c->window_samples : 0;
    c->last_std = c->window_samples > 1 ? sqrtf(fmaxf(0, c->current_m2 / (c->window_samples - 1))) : 0;
    c->last_mean = c->current_mean;
    if(c->window_samples < ANTICOG_MIN_SAMPLES) return "Insufficient samples in measurement window";
    if(c->window_command_max >= 0.95f * h->flash_data.limits.max_dq_current)
        return "Current command saturated (>=95 percent of current limit)";
    if(c->window_error_max > ANTICOG_POSITION_TOLERANCE)
        return "Position error exceeds tolerance during measurement window";
    if(c->position_max - c->position_min > ANTICOG_POSITION_SPAN)
        return "Rotor oscillation/drift: position span exceeds window limit";
    if(c->speed_sum / c->window_samples > ANTICOG_MEAN_ABS_SPEED)
        return "Mean absolute speed exceeds measurement window limit";
    float variance = c->current_m2 / (c->window_samples - 1);
    float limit = fmaxf(ANTICOG_CURRENT_STD_ABS, fabsf(c->current_mean) * ANTICOG_CURRENT_STD_REL);
    if(variance > limit * limit) return "Measured current is unstable: standard deviation exceeds window limit";
    return NULL;
}

/** @brief Formats a scaled measurement without requiring floating-point printf support. */
static void anticog_format_value(char *text, size_t size, float value, float scale){
    float scaled = value * scale;
    if(!isfinite(scaled) || scaled < -2147483000.0f || scaled > 2147483000.0f)
        snprintf(text, size, "invalid");
    else snprintf(text, size, "%ld", (long)scaled);
}

/**
 * @brief Reports one terminal failure after excitation has been removed.
 * @param target Position target captured before cleanup clears control commands.
 * @param q_command Current command captured before cleanup zeroes it.
 * @note Both timeout paths retain routine error code 4; text distinguishes them.
 */
static void anticog_report_failure(FOC_HandleTypeDef *hfoc, float target, float q_command){
    const FOC_AntiCoggingContextTypeDef *ctx = &hfoc->anticogging;
    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - ctx->runtime.started_ms;
    uint32_t overall_limit = ANTICOG_TIMEOUT_MS * ctx->revolutions;
    const char *reason = "invalid calibration result";
    if(ctx->runtime.error == FOC_ROUTINE_ERROR_TIMEOUT)
        reason = elapsed >= overall_limit ? "overall calibration timeout" : "point settling timeout";
    else if(ctx->runtime.error == FOC_ROUTINE_ERROR_INVALID_STAGE) reason = "invalid routine stage or counters";
    else if(ctx->runtime.error == FOC_ROUTINE_ERROR_INVALID_INPUT) reason = "invalid terminal status";
    else if(ctx->runtime.error == FOC_ROUTINE_ERROR_INVALID_RESULT)
        reason = "fault, invalid measurement/current limit, or incomplete result";
    char values[8][16];
    anticog_format_value(values[0], sizeof(values[0]), ctx->last_error, 1000000);
    anticog_format_value(values[1], sizeof(values[1]), ctx->last_span, 1000000);
    anticog_format_value(values[2], sizeof(values[2]), ctx->last_speed, 1000000);
    anticog_format_value(values[3], sizeof(values[3]), ctx->last_mean, 1000);
    anticog_format_value(values[4], sizeof(values[4]), ctx->last_std, 1000);
    anticog_format_value(values[5], sizeof(values[5]), target, 1000);
    anticog_format_value(values[6], sizeof(values[6]), q_command, 1000);
    anticog_format_value(values[7], sizeof(values[7]), fmaxf(ANTICOG_CURRENT_STD_ABS,
        fabsf(ctx->last_mean) * ANTICOG_CURRENT_STD_REL), 1000);
    int bin = ctx->sample_index >= NUMBER_OF_ANTICOG_MEASUREMENTS ? -1 :
        ctx->direction ? NUMBER_OF_ANTICOG_MEASUREMENTS - ctx->sample_index - 1 : ctx->sample_index;
    snprintf(hfoc->routine_result, sizeof(hfoc->routine_result),
        "Routine result: Anti-cogging failed: %s (code %u). %.80s; stage=%u; direction=%s; revolution=%u/%u; bin=%d; progress=%u.%u%%. "
        "total=%lums/%lums; point=%lums/%ums. Last window: error=%s/%ldurad; span=%s/%ldurad; speed=%s/%ldurad/s; Iq=%smA; std=%s/%smA; target=%smrad; command=%smA; active=0x%08lX; latched=0x%08lX.",
        reason, (unsigned)ctx->runtime.error, ctx->runtime.detail ? ctx->runtime.detail : "No stable window",
        (unsigned)ctx->failed_stage, ctx->direction ? "reverse" : "forward", (unsigned)ctx->revolution + 1,
        (unsigned)ctx->revolutions, bin, (unsigned)ctx->progress / 10, (unsigned)ctx->progress % 10,
        (unsigned long)elapsed, (unsigned long)overall_limit,
        (unsigned long)(ctx->failed_stage == FOC_ANTICOG_COLLECT ? now - ctx->point_started_ms : 0),
        (unsigned)ANTICOG_POINT_TIMEOUT_MS,
        values[0], (long)(ANTICOG_POSITION_TOLERANCE * 1000000),
        values[1], (long)(ANTICOG_POSITION_SPAN * 1000000),
        values[2], (long)(ANTICOG_MEAN_ABS_SPEED * 1000000),
        values[3], values[4], values[7], values[5], values[6],
        (unsigned long)hfoc->active_errors, (unsigned long)hfoc->latched_errors);
    Debug_SendTextResponse("%s\n", hfoc->routine_result);
}

/**
 * @brief Starts a fresh in-place anti-cogging measurement with an invalid LUT.
 * @param hfoc FOC handle owning the workspace and measurement table.
 * @note No second LUT is allocated; cancellation leaves partial data invalid.
 */
void FOC_AntiCogging_Start(FOC_HandleTypeDef *hfoc){
    if(hfoc->anticogging.runtime.active) FOC_AntiCogging_Finish(hfoc, FOC_LOOP_CANCELLED);
    hfoc->anticogging = (FOC_AntiCoggingContextTypeDef){
        .runtime = {.active = 1, .status = FOC_LOOP_IN_PROGRESS, .started_ms = HAL_GetTick()},
        .stage = FOC_ANTICOG_START
    };
    hfoc->anticogging.revolutions = hfoc->calibration_revolutions[1] ? hfoc->calibration_revolutions[1] : 1;
    PIDValuesTypeDef gains = hfoc->flash_data.anticogging_position_gains;
    if(!isfinite(gains.Kp) || gains.Kp <= 0 || !isfinite(gains.Ki) || gains.Ki < 0 ||
       !isfinite(gains.Kd) || gains.Kd < 0){
        hfoc->anticogging.runtime.error = FOC_ROUTINE_ERROR_INVALID_INPUT;
        hfoc->anticogging.runtime.detail = "Configure valid anti-cogging PID gains (Kp>0, Ki/Kd>=0) before anti-cogging";
        FOC_AntiCogging_Finish(hfoc, FOC_LOOP_ERROR);
        return;
    }
    PID_Init(&hfoc->pid_anticogging, 1.0f/(CURRENT_LOOP_FREQUENCY / SPEED_LOOP_CLOCK_DIVIDER), 0.01f,
             &hfoc->flash_data.limits.max_dq_current, &hfoc->flash_data.anticogging_position_gains, 0);
    hfoc->anticogging.saved_ff = hfoc->flash_data.controller.anticogging_FF_enabled;
    hfoc->flash_data.controller.anticogging_FF_enabled = 0;
    hfoc->flash_data.controller.anticogging_data_valid = 0;
    for(unsigned d = 0; d < 2; d++)
        for(unsigned i = 0; i < NUMBER_OF_ANTICOG_MEASUREMENTS; i++)
            hfoc->flash_data.controller.anticogging_array[d][i] = 0;
}

/**
 * @brief Restores temporary PID gains and removes all measurement actuation.
 * @param hfoc FOC handle owning the measurement.
 * @param result Terminal status; only success marks the LUT valid.
 * @return Retained terminal result.
 * @note Never restores previous motor targets or enables the gate driver.
 */
FOC_LoopStatusTypeDef FOC_AntiCogging_Finish(FOC_HandleTypeDef *hfoc, FOC_LoopStatusTypeDef result){
    FOC_AntiCoggingContextTypeDef *ctx = &hfoc->anticogging;
    if(!ctx->runtime.active) return ctx->runtime.status;
    if(result == FOC_LOOP_CANCELLED && (hfoc->active_errors || hfoc->latched_errors)){
        ctx->runtime.detail = "Interrupted by fault; see active/latched flags";
    }
    float failed_target = hfoc->angle_setpoint;
    float failed_q_command = hfoc->dq_current_setpoint.q;
    if(ctx->gains_saved){
        hfoc->flash_data.controller.PID_gains_position = ctx->saved_position_gains;
        FOC_SetControlMode(hfoc, CONTROL_MODE_OPENLOOP);
        PID_Reset(&hfoc->pid_position);
        PID_Reset(&hfoc->pid_anticogging);
        PID_Reset(&hfoc->pid_speed);
        ctx->gains_saved = 0;
    }
    hfoc->dq_current_setpoint = (DQCurrentsTypeDef){0};
    hfoc->angle_setpoint = hfoc->speed_setpoint = 0;
    FOC_SetPhaseVoltages(hfoc, (PhaseVoltagesTypeDef){0});
    DRV8323_SetHighImpedance(&hfoc->hdrv8323);
    if(result == FOC_LOOP_COMPLETED && !ctx->candidate_valid){
        ctx->runtime.error = FOC_ROUTINE_ERROR_INVALID_RESULT;
        result = FOC_LOOP_ERROR;
    }
    if(result != FOC_LOOP_COMPLETED && result != FOC_LOOP_ERROR && result != FOC_LOOP_CANCELLED){
        ctx->runtime.error = FOC_ROUTINE_ERROR_INVALID_INPUT;
        result = FOC_LOOP_ERROR;
    }
    hfoc->flash_data.controller.anticogging_data_valid = result == FOC_LOOP_COMPLETED;
    hfoc->flash_data.controller.anticogging_FF_enabled = 0; /* New/partial table needs explicit enable. */
    if(result == FOC_LOOP_COMPLETED) ctx->progress = 1000;
    if(result == FOC_LOOP_ERROR) ctx->failed_stage = ctx->stage;
    ctx->runtime.active = 0;
    ctx->runtime.status = result;
    hfoc->setup_result.status = (uint8_t)result;
    hfoc->setup_result.error = (uint8_t)ctx->runtime.error;
    if(result != FOC_LOOP_ERROR) FOC_ReportRoutineResult(hfoc, "Anti-cogging calibration", &ctx->runtime, (unsigned)ctx->stage);
    ctx->stage = result == FOC_LOOP_COMPLETED ? FOC_ANTICOG_DONE :
                 result == FOC_LOOP_CANCELLED ? FOC_ANTICOG_CANCELLED : FOC_ANTICOG_FAILED;
    if(result == FOC_LOOP_ERROR) anticog_report_failure(hfoc, failed_target, failed_q_command);
    return result;
}

/**
 * @brief Runs current control at sample rate and measurement/speed work divided.
 * @param hfoc FOC handle previously started for anti-cogging measurement.
 * @return IN_PROGRESS or retained terminal status without further actuation.
 */
FOC_LoopStatusTypeDef FOC_AntiCogging_Update(FOC_HandleTypeDef *hfoc){
    FOC_AntiCoggingContextTypeDef *ctx = &hfoc->anticogging;
    if(!ctx->runtime.active){
        if(ctx->stage == FOC_ANTICOG_IDLE){
            ctx->runtime.status = FOC_LOOP_ERROR;
            ctx->runtime.error = FOC_ROUTINE_ERROR_NOT_STARTED;
        }
        return ctx->runtime.status;
    }
    uint32_t now = HAL_GetTick();
    if(FOC_RoutineElapsed(now, ctx->runtime.started_ms, ANTICOG_TIMEOUT_MS * ctx->revolutions)){
        ctx->runtime.error = FOC_ROUTINE_ERROR_TIMEOUT;
        return FOC_AntiCogging_Finish(hfoc, FOC_LOOP_ERROR);
    }
    if(ctx->stage < FOC_ANTICOG_START || ctx->stage > FOC_ANTICOG_WAIT_FINISH ||
       ctx->direction > 1 || ctx->report_direction > 1 || ctx->revolutions < 1 || ctx->revolutions > 8 ||
       ctx->sample_index > NUMBER_OF_ANTICOG_MEASUREMENTS ||
       ctx->report_index > NUMBER_OF_ANTICOG_MEASUREMENTS + 1){
        ctx->runtime.error = FOC_ROUTINE_ERROR_INVALID_STAGE;
        return FOC_AntiCogging_Finish(hfoc, FOC_LOOP_ERROR);
    }
    Current_Loop(hfoc);
    ctx->filtered_current = ctx->filtered_current * 0.99f + hfoc->dq_current.q * 0.01f;
    if(isfinite(ctx->filtered_current) && isfinite(hfoc->flash_data.limits.max_dq_current) &&
       hfoc->flash_data.limits.max_dq_current > 0 &&
       fabsf(ctx->filtered_current) > hfoc->flash_data.limits.max_dq_current){
        ctx->runtime.error = FOC_ROUTINE_ERROR_INVALID_RESULT;
        ctx->runtime.detail = "Filtered Iq exceeds maximum d/q current";
        FOC_AntiCogging_Finish(hfoc, FOC_LOOP_ERROR);
        FOC_SetError(hfoc, FOC_ERROR_PHASE_OC);
        return FOC_LOOP_ERROR;
    }
    if(hfoc->active_errors || hfoc->latched_errors || !isfinite(ctx->filtered_current) ||
       !isfinite(hfoc->flash_data.limits.max_dq_current) || hfoc->flash_data.limits.max_dq_current <= 0 ||
       !isfinite(hfoc->encoder_speed_mechanical) ||
       !isfinite(hfoc->encoder_angle_mechanical_wrapped) ||
       !isfinite(hfoc->encoder_angle_mechanical_unwrapped) || !isfinite(hfoc->dq_current.q) ||
       !isfinite(hfoc->dq_current_setpoint.q)){
        ctx->runtime.detail = "Fault or non-finite encoder/current measurement";
        ctx->runtime.error = FOC_ROUTINE_ERROR_INVALID_RESULT;
        return FOC_AntiCogging_Finish(hfoc, FOC_LOOP_ERROR);
    }
    if(++ctx->divider < SPEED_LOOP_CLOCK_DIVIDER) return FOC_LOOP_IN_PROGRESS;
    ctx->divider = 0;
    if(FOC_RoutineElapsed(now, ctx->runtime.stage_started_ms, ctx->wait_ms)){
        switch(ctx->stage){
            case FOC_ANTICOG_START:
                ctx->saved_position_gains = hfoc->flash_data.controller.PID_gains_position;
                ctx->gains_saved = 1;
                FOC_SetControlMode(hfoc, CONTROL_MODE_POSITION);
                PID_Reset(&hfoc->pid_position);
        PID_Reset(&hfoc->pid_anticogging);
                hfoc->pid_anticogging.prevMeasurement = hfoc->encoder_angle_mechanical_unwrapped;
                PID_Reset(&hfoc->pid_speed);
                PID_Reset(&hfoc->pid_current_d);
                PID_Reset(&hfoc->pid_current_q);
                hfoc->dq_current_setpoint = (DQCurrentsTypeDef){0};
                FOC_SetPhaseVoltages(hfoc, (PhaseVoltagesTypeDef){0});
                if(DRV8323_ExitHighImpedance(&hfoc->hdrv8323) != DRV8323_OK){
                    ctx->runtime.detail = "Gate driver refused inverter output enable";
                    ctx->runtime.error = FOC_ROUTINE_ERROR_INVALID_INPUT;
                    return FOC_AntiCogging_Finish(hfoc, FOC_LOOP_ERROR);
                }
                ctx->turn_base = hfoc->encoder_angle_mechanical_unwrapped -
                    remainderf(hfoc->encoder_angle_mechanical_unwrapped, M_2PIF);
                hfoc->angle_setpoint = hfoc->encoder_angle_mechanical_unwrapped;
                ctx->stage = FOC_ANTICOG_POSITION;
                ctx->wait_ms = 0;
                break;
            case FOC_ANTICOG_POSITION:
                /* Approach the first bin from the same direction as the sweep.
                   Adjacent revolutions use continuous targets, never jump back to zero. */
                hfoc->angle_setpoint = ctx->turn_base + (ctx->direction ?
                    (ctx->revolutions - ctx->revolution) * M_2PIF :
                    ctx->revolution * M_2PIF - ANTICOG_ANGLE_STEP);
                ctx->approaching = 1;
                anticog_reset_window(ctx, now + ANTICOG_MOVE_SETTLE_MS);
                ctx->stage = FOC_ANTICOG_COLLECT;
                ctx->wait_ms = ANTICOG_MOVE_SETTLE_MS;
                ctx->point_started_ms = now;
                break;
            case FOC_ANTICOG_COLLECT:
                if(ctx->sample_index < NUMBER_OF_ANTICOG_MEASUREMENTS){
                    if(FOC_RoutineElapsed(now, ctx->point_started_ms, ANTICOG_POINT_TIMEOUT_MS)){
                        ctx->runtime.error = FOC_ROUTINE_ERROR_TIMEOUT;
                        if(!ctx->runtime.detail) ctx->runtime.detail = "Point settling timeout: no stable measurement window";
                        return FOC_AntiCogging_Finish(hfoc, FOC_LOOP_ERROR);
                    }
                    float position = hfoc->encoder_angle_mechanical_unwrapped;
                    float error = hfoc->angle_setpoint - position;
                    if(ctx->window_samples == 0){
                        ctx->position_min = ctx->position_max = position;
                        ctx->window_started_ms = now;
                    }
                    ctx->position_min = fminf(ctx->position_min, position);
                    ctx->position_max = fmaxf(ctx->position_max, position);
                    ctx->window_error_max = fmaxf(ctx->window_error_max, fabsf(error));
                    ctx->window_command_max = fmaxf(ctx->window_command_max, fabsf(hfoc->dq_current_setpoint.q));
                    ctx->speed_sum += fabsf(hfoc->encoder_speed_mechanical);
                    ctx->window_samples++;
                    float delta = hfoc->dq_current.q - ctx->current_mean;
                    ctx->current_mean += delta / ctx->window_samples;
                    ctx->current_m2 += delta * (hfoc->dq_current.q - ctx->current_mean);
                    ctx->wait_ms = 0; /* Sample each divided position-loop tick. */
                    if(!FOC_RoutineElapsed(now, ctx->window_started_ms, ANTICOG_WINDOW_MS)) break;
                    const char *reason = anticog_window_reason(hfoc);
                    if(reason){
                        ctx->runtime.detail = reason;
                        anticog_reset_window(ctx, now);
                        break;
                    }
                    if(ctx->approaching){
                        ctx->approaching = 0;
                    }else{
                        unsigned index = ctx->direction ? NUMBER_OF_ANTICOG_MEASUREMENTS - 1 - ctx->sample_index : ctx->sample_index;
                        hfoc->flash_data.controller.anticogging_array[ctx->direction][index] += ctx->current_mean / ctx->revolutions;
                        ctx->sample_index++;
                        ctx->progress = ((ctx->direction * ctx->revolutions + ctx->revolution) * NUMBER_OF_ANTICOG_MEASUREMENTS + ctx->sample_index) * 1000UL /
                            (2UL * ctx->revolutions * NUMBER_OF_ANTICOG_MEASUREMENTS);
                    }
                    ctx->runtime.detail = NULL;
                    if(ctx->sample_index < NUMBER_OF_ANTICOG_MEASUREMENTS)
                        hfoc->angle_setpoint = anticog_target(ctx);
                    anticog_reset_window(ctx, now);
                    ctx->point_started_ms = now;
                    ctx->wait_ms = ANTICOG_MOVE_SETTLE_MS;
                }else{
                    ctx->sample_index = 0;
                    if(++ctx->revolution < ctx->revolutions){
                        ctx->stage = FOC_ANTICOG_POSITION;
                        ctx->wait_ms = 0;
                    }else if(ctx->direction == 0){
                        ctx->revolution = 0;
                        ctx->direction = 1;
                        ctx->stage = FOC_ANTICOG_POSITION;
                        ctx->wait_ms = 0;
                    }else{
                        ctx->candidate_valid = 1;
                        ctx->stage = FOC_ANTICOG_WAIT_FINISH;
                        ctx->wait_ms = 0;
                    }
                }
                break;
            case FOC_ANTICOG_REPORT:
                if(ctx->report_index == 0){
                    Debug_SendTextResponse("Current measurements for direction %d:\n", ctx->report_direction);
                    ctx->report_index++;
                    ctx->wait_ms = 10;
                }else if(ctx->report_index <= NUMBER_OF_ANTICOG_MEASUREMENTS){
                    Debug_SendTextResponse("%d,", (int)(hfoc->flash_data.controller.anticogging_array[ctx->report_direction][ctx->report_index - 1] * 1000));
                    ctx->report_index++;
                    ctx->wait_ms = 2;
                }else{
                    Debug_SendTextResponse("\n");
                    ctx->report_index = 0;
                    if(ctx->report_direction == 0){
                        ctx->report_direction = 1;
                        ctx->wait_ms = 2;
                    }else{
                        ctx->stage = FOC_ANTICOG_WAIT_FINISH;
                        ctx->wait_ms = 1000;
                    }
                }
                break;
            case FOC_ANTICOG_WAIT_FINISH:
                return FOC_AntiCogging_Finish(hfoc, FOC_LOOP_COMPLETED);
            default:
                ctx->runtime.error = FOC_ROUTINE_ERROR_INVALID_STAGE;
                return FOC_AntiCogging_Finish(hfoc, FOC_LOOP_ERROR);
        }
        ctx->runtime.stage_started_ms = now;
    }
    Speed_Loop(hfoc);
    return FOC_LOOP_IN_PROGRESS;
}
