#include <math.h>
#include <stdio.h>
#include "FOC_Loops.h"
#include "FOC_Handle.h"
#include "FOC_USB_Debug.h"
#include "Utils.h"
#include "FOC_Config.h"

#define ANTICOG_TIMEOUT_MS 600000U
#define ANTICOG_POINT_TIMEOUT_MS 10000U
#define ANTICOG_SETTLE_SPEED 0.001f
#define ANTICOG_SETTLE_ERROR 0.005f
#define ANTICOG_HOLD_CHECKS 11U

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
    float wrapped_error = target - hfoc->encoder_angle_mechanical_wrapped;
    /* Avoid an unbounded normalization loop for corrupt/non-finite measurements. */
    if(isfinite(wrapped_error)) wrapped_error = remainderf(wrapped_error, M_2PIF);
    char values[8][16];
    anticog_format_value(values[0], sizeof(values[0]), target, 1000);
    anticog_format_value(values[1], sizeof(values[1]), hfoc->encoder_angle_mechanical_wrapped, 1000);
    anticog_format_value(values[2], sizeof(values[2]), hfoc->encoder_angle_mechanical_unwrapped, 1000);
    anticog_format_value(values[3], sizeof(values[3]), wrapped_error, 1000000);
    anticog_format_value(values[4], sizeof(values[4]), target - hfoc->encoder_angle_mechanical_unwrapped, 1000000);
    anticog_format_value(values[5], sizeof(values[5]), hfoc->encoder_speed_mechanical, 1000000);
    anticog_format_value(values[6], sizeof(values[6]), ctx->filtered_current, 1000);
    anticog_format_value(values[7], sizeof(values[7]), q_command, 1000);
    int bin = ctx->sample_index >= NUMBER_OF_ANTICOG_MEASUREMENTS ? -1 :
        ctx->direction ? NUMBER_OF_ANTICOG_MEASUREMENTS - ctx->sample_index - 1 : ctx->sample_index;
    Debug_SendTextResponse(
        "Anti-cogging failed: %s (code %u). stage=%u; direction=%s; revolution=%u/%u; bin=%d; progress=%u.%u%%.\n"
        "Timing: total=%lums/%lums; point=%lums/%ums.\n"
        "At failure: target=%smrad; wrapped_position=%smrad; unwrapped_position=%smrad; "
        "wrapped_error=%surad; PID_error=%surad; speed=%surad/s; filtered_Iq=%smA; Iq_command=%smA; hold=%u/%u.\n"
        "Settling requires |wrapped_error|<%ldurad and |speed|<%ldurad/s for %u consecutive checks. "
        "active_faults=0x%08lX; latched_faults=0x%08lX.\n",
        reason, (unsigned)ctx->runtime.error, (unsigned)ctx->failed_stage,
        ctx->direction ? "reverse" : "forward", (unsigned)ctx->revolution + 1,
        (unsigned)ctx->revolutions, bin, (unsigned)ctx->progress / 10, (unsigned)ctx->progress % 10,
        (unsigned long)elapsed, (unsigned long)overall_limit,
        (unsigned long)(ctx->failed_stage == FOC_ANTICOG_COLLECT ? now - ctx->point_started_ms : 0),
        (unsigned)ANTICOG_POINT_TIMEOUT_MS,
        values[0], values[1], values[2], values[3], values[4], values[5], values[6], values[7],
        (unsigned)ctx->hold_count, (unsigned)ANTICOG_HOLD_CHECKS,
        (long)(ANTICOG_SETTLE_ERROR * 1000000), (long)(ANTICOG_SETTLE_SPEED * 1000000), (unsigned)ANTICOG_HOLD_CHECKS,
        (unsigned long)hfoc->active_errors, (unsigned long)hfoc->latched_errors);
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
    hfoc->anticogging.revolutions = hfoc->calibration_revolutions[1] ? hfoc->calibration_revolutions[1] : 3;
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
    float failed_target = hfoc->angle_setpoint;
    float failed_q_command = hfoc->dq_current_setpoint.q;
    if(ctx->gains_saved){
        hfoc->flash_data.controller.PID_gains_position = ctx->saved_position_gains;
        FOC_SetControlMode(hfoc, CONTROL_MODE_OPENLOOP);
        PID_Reset(&hfoc->pid_position);
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
    hfoc->flash_data.controller.anticogging_FF_enabled = ctx->saved_ff;
    if(result == FOC_LOOP_COMPLETED) ctx->progress = 1000;
    if(result == FOC_LOOP_ERROR) ctx->failed_stage = ctx->stage;
    ctx->runtime.active = 0;
    ctx->runtime.status = result;
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
    if(hfoc->active_errors || hfoc->latched_errors || !isfinite(ctx->filtered_current) ||
       !isfinite(hfoc->flash_data.limits.max_dq_current) || hfoc->flash_data.limits.max_dq_current <= 0 ||
       fabsf(ctx->filtered_current) > hfoc->flash_data.limits.max_dq_current || !isfinite(hfoc->encoder_speed_mechanical) ||
       !isfinite(hfoc->encoder_angle_mechanical_wrapped)){
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
                PID_Reset(&hfoc->pid_speed);
                PID_Reset(&hfoc->pid_current_d);
                PID_Reset(&hfoc->pid_current_q);
                hfoc->dq_current_setpoint = (DQCurrentsTypeDef){0};
                FOC_SetPhaseVoltages(hfoc, (PhaseVoltagesTypeDef){0});
                DRV8323_ExitHighImpedance(&hfoc->hdrv8323);
                hfoc->flash_data.controller.PID_gains_position = (PIDValuesTypeDef){.Kp = 10, .Ki = 20, .Kd = 0.2f};
                ctx->stage = FOC_ANTICOG_POSITION;
                ctx->wait_ms = 0;
                break;
            case FOC_ANTICOG_POSITION:
                hfoc->angle_setpoint = ctx->direction ? (NUMBER_OF_ANTICOG_MEASUREMENTS - 1) * ANTICOG_ANGLE_STEP : 0;
                ctx->stage = FOC_ANTICOG_COLLECT;
                ctx->wait_ms = 100;
                ctx->point_started_ms = now;
                break;
            case FOC_ANTICOG_COLLECT:
                if(ctx->sample_index < NUMBER_OF_ANTICOG_MEASUREMENTS){
                    if(FOC_RoutineElapsed(now, ctx->point_started_ms, ANTICOG_POINT_TIMEOUT_MS)){
                        ctx->runtime.error = FOC_ROUTINE_ERROR_TIMEOUT;
                        return FOC_AntiCogging_Finish(hfoc, FOC_LOOP_ERROR);
                    }
                    float error = hfoc->angle_setpoint - hfoc->encoder_angle_mechanical_wrapped;
                    normalize_angle_pm_pi(&error);
                    if(fabsf(hfoc->encoder_speed_mechanical) < ANTICOG_SETTLE_SPEED && fabsf(error) < ANTICOG_SETTLE_ERROR){
                        ctx->hold_count++;
                    }else ctx->hold_count = 0;
                    if(ctx->hold_count >= ANTICOG_HOLD_CHECKS){
                        /* Record the settled target before advancing to the next bin. */
                        uint16_t index = ctx->direction == 0 ? ctx->sample_index :
                                         NUMBER_OF_ANTICOG_MEASUREMENTS - ctx->sample_index - 1;
                        hfoc->flash_data.controller.anticogging_array[ctx->direction][index] += ctx->filtered_current / ctx->revolutions;
                        ctx->sample_index++;
                        ctx->progress = ((ctx->direction * ctx->revolutions + ctx->revolution) * NUMBER_OF_ANTICOG_MEASUREMENTS + ctx->sample_index) * 1000UL /
                            (2UL * ctx->revolutions * NUMBER_OF_ANTICOG_MEASUREMENTS);
                        if(ctx->sample_index < NUMBER_OF_ANTICOG_MEASUREMENTS){
                            unsigned next = ctx->direction ? NUMBER_OF_ANTICOG_MEASUREMENTS - ctx->sample_index - 1 : ctx->sample_index;
                            hfoc->angle_setpoint = next * ANTICOG_ANGLE_STEP;
                        }
                        ctx->hold_count = 0;
                        ctx->point_started_ms = now;
                    }
                    ctx->wait_ms = 2;
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
