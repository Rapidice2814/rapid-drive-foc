#include <math.h>
#include "FOC_Handle.h"
#include "FOC_Loops.h"
#include "FOC_USB_Debug.h"
#include "Cordic.h"
#include "Utils.h"

#define CAL_STEPS 1024U
#define CAL_SAMPLES 8U
#define CAL_SETTLE_MS 10U
#define CAL_TIMEOUT_MS 120000U
#define CAL_DISAGREEMENT_MAX 0.03f

/** @brief Commands a stator field without closing position feedback on the encoder. */
static void calibration_field(FOC_HandleTypeDef *hfoc){
    FOC_EncoderCalibrationContextTypeDef *ctx = &hfoc->encoder_calibration;
    float phase = (float)ctx->step * (M_2PIF / CAL_STEPS) * hfoc->flash_data.motor.pole_pairs;
    normalize_angle_0_2pi(&phase);
    float c, s;
    Cordic_CalculateSinCos(phase, &c, &s);
    FOC_SetPhaseVoltages(hfoc, InvClarke_transform((ABVoltagesTypeDef){ctx->voltage * c, ctx->voltage * s}));
}

/**
 * @brief Starts an unloaded bidirectional encoder calibration with private buffers.
 * @param hfoc Stationary, fault-free FOC handle with known pole count/direction.
 * @note Existing accepted LUT/zero remain untouched during measurement.
 */
void FOC_EncoderCalibration_Start(FOC_HandleTypeDef *hfoc){
    if(hfoc->encoder_calibration.runtime.active) FOC_EncoderCalibration_Finish(hfoc, FOC_LOOP_CANCELLED);
    hfoc->encoder_calibration = (FOC_EncoderCalibrationContextTypeDef){
        .runtime = {.active = 1, .status = FOC_LOOP_IN_PROGRESS, .started_ms = HAL_GetTick(), .stage_started_ms = HAL_GetTick()},
        .stage = FOC_ENCODER_CAL_LOCK,
        .voltage = hfoc->flash_data.encoder.calibration_voltage,
        .source = hfoc->flash_data.encoder.source_id,
        .revolutions = hfoc->calibration_revolutions[0] ? hfoc->calibration_revolutions[0] : 3,
        .pole_pairs = hfoc->flash_data.motor.pole_pairs,
        .motor_direction = hfoc->flash_data.motor.direction,
        .saved_hfi = hfoc->flash_data.hfi.hfi_enabled,
        .saved_anticog_ff = hfoc->flash_data.controller.anticogging_FF_enabled,
        .saved_current_ff = hfoc->flash_data.controller.current_PID_FF_enabled,
        .saved_anticog_valid = hfoc->flash_data.controller.anticogging_data_valid
    };
    FOC_EncoderCalibrationContextTypeDef *ctx = &hfoc->encoder_calibration;
    const FOC_EncoderSourceTypeDef *source = FOC_EncoderSource(ctx->source);
    if(ctx->revolutions > 8 || source == NULL || !source->calibratable || !hfoc->encoder_runtime.healthy ||
       hfoc->active_errors || hfoc->latched_errors || !hfoc->adc_calibrated ||
       !hfoc->flash_data.encoder.offset_valid || hfoc->flash_data.motor.pole_pairs == 0 ||
       !isfinite(ctx->voltage) || ctx->voltage <= 0 ||
       !isfinite(hfoc->flash_data.limits.max_dq_voltage) || ctx->voltage > hfoc->flash_data.limits.max_dq_voltage ||
       !isfinite(hfoc->flash_data.limits.max_dq_current) || hfoc->flash_data.limits.max_dq_current <= 0){
        ctx->runtime.error = FOC_ROUTINE_ERROR_INVALID_INPUT;
        FOC_EncoderCalibration_Finish(hfoc, FOC_LOOP_ERROR);
        return;
    }
    hfoc->encoder_runtime.bypass_correction = 1;
    FOC_EncoderResetHistory(hfoc);
    hfoc->flash_data.hfi.hfi_enabled = 0;
    hfoc->flash_data.controller.anticogging_FF_enabled = 0;
    hfoc->flash_data.controller.current_PID_FF_enabled = 0;
    FOC_SetPhaseVoltages(hfoc, (PhaseVoltagesTypeDef){0});
    DRV8323_ExitHighImpedance(&hfoc->hdrv8323);
    calibration_field(hfoc);
}

/**
 * @brief Removes excitation, restores temporary flags and retains terminal status.
 * @param result Terminal status. SAVE already committed/persisted successful data.
 * @return Retained status; cancellation never overwrites accepted calibration.
 */
FOC_LoopStatusTypeDef FOC_EncoderCalibration_Finish(FOC_HandleTypeDef *hfoc, FOC_LoopStatusTypeDef result){
    FOC_EncoderCalibrationContextTypeDef *ctx = &hfoc->encoder_calibration;
    if(!ctx->runtime.active) return ctx->runtime.status;
    FOC_SetPhaseVoltages(hfoc, (PhaseVoltagesTypeDef){0});
    DRV8323_SetHighImpedance(&hfoc->hdrv8323);
    hfoc->encoder_runtime.bypass_correction = 0;
    hfoc->flash_data.hfi.hfi_enabled = ctx->saved_hfi;
    hfoc->flash_data.controller.anticogging_FF_enabled = ctx->saved_anticog_ff;
    hfoc->flash_data.controller.current_PID_FF_enabled = ctx->saved_current_ff;
    if(result == FOC_LOOP_COMPLETED && !ctx->candidate_valid) result = FOC_LOOP_ERROR;
    if(result != FOC_LOOP_COMPLETED && result != FOC_LOOP_CANCELLED && result != FOC_LOOP_ERROR) result = FOC_LOOP_ERROR;
    if(result == FOC_LOOP_ERROR) ctx->failed_stage = ctx->stage;
    ctx->runtime.status = result;
    ctx->runtime.active = 0;
    ctx->stage = result == FOC_LOOP_COMPLETED ? FOC_ENCODER_CAL_DONE :
        result == FOC_LOOP_CANCELLED ? FOC_ENCODER_CAL_CANCELLED : FOC_ENCODER_CAL_FAILED;
    FOC_EncoderResetHistory(hfoc);
    return result;
}

/** @brief Validates both sweep coverage/repeatability and a continuous monotonic LUT. */
static uint8_t calibration_validate(FOC_HandleTypeDef *hfoc){
    FOC_EncoderCalibrationContextTypeDef *ctx = &hfoc->encoder_calibration;
    float mean = 0;
    for(unsigned i = 0; i < FOC_ENCODER_LUT_SIZE; i++){
        if(!ctx->forward_count[i] || !ctx->reverse_count[i]) return 0;
        float a = ctx->forward[i] / ctx->forward_count[i];
        float b = ctx->reverse[i] / ctx->reverse_count[i];
        float disagreement = fabsf(a - b);
        if(!isfinite(a) || !isfinite(b) || disagreement > CAL_DISAGREEMENT_MAX) return 0;
        if(disagreement > ctx->max_disagreement) ctx->max_disagreement = disagreement;
        ctx->forward[i] = (a + b) * 0.5f;
        mean += ctx->forward[i] / FOC_ENCODER_LUT_SIZE;
    }
    for(unsigned i = 0; i < FOC_ENCODER_LUT_SIZE; i++) ctx->forward[i] -= mean;
    return FOC_EncoderValidateLUT(ctx->forward);
}

/** @brief Swaps the candidate/accepted LUT so RAM can be rolled back on flash failure. */
static void calibration_swap(FOC_HandleTypeDef *hfoc){
    for(unsigned i = 0; i < FOC_ENCODER_LUT_SIZE; i++){
        float old = hfoc->flash_data.encoder.correction[i];
        hfoc->flash_data.encoder.correction[i] = hfoc->encoder_calibration.forward[i];
        hfoc->encoder_calibration.forward[i] = old;
    }
}

/**
 * @brief Advances a lock, settling, one-sample, validation, or persistence stage.
 * @param hfoc FOC handle updated by the encoder provider before this sample tick.
 * @return IN_PROGRESS or terminal result. No flash writes occur during a sweep.
 */
FOC_LoopStatusTypeDef FOC_EncoderCalibration_Update(FOC_HandleTypeDef *hfoc){
    FOC_EncoderCalibrationContextTypeDef *ctx = &hfoc->encoder_calibration;
    if(!ctx->runtime.active){
        if(ctx->stage == FOC_ENCODER_CAL_IDLE){ctx->runtime.status = FOC_LOOP_ERROR; ctx->runtime.error = FOC_ROUTINE_ERROR_NOT_STARTED;}
        return ctx->runtime.status;
    }
    uint32_t now = HAL_GetTick();
    if(FOC_RoutineElapsed(now, ctx->runtime.started_ms, CAL_TIMEOUT_MS * ctx->revolutions)){
        ctx->runtime.error = FOC_ROUTINE_ERROR_TIMEOUT;
        return FOC_EncoderCalibration_Finish(hfoc, FOC_LOOP_ERROR);
    }
    if(!hfoc->encoder_runtime.healthy || ctx->source != hfoc->flash_data.encoder.source_id ||
       ctx->pole_pairs != hfoc->flash_data.motor.pole_pairs || ctx->motor_direction != hfoc->flash_data.motor.direction ||
       !isfinite(hfoc->encoder_runtime.raw_angle) || hfoc->active_errors || hfoc->latched_errors) goto invalid;
    if(!isfinite(hfoc->adc_values.phase_current.a) || !isfinite(hfoc->adc_values.phase_current.b) ||
       !isfinite(hfoc->adc_values.phase_current.c) ||
       fabsf(hfoc->adc_values.phase_current.a) > hfoc->flash_data.limits.max_dq_current ||
       fabsf(hfoc->adc_values.phase_current.b) > hfoc->flash_data.limits.max_dq_current ||
       fabsf(hfoc->adc_values.phase_current.c) > hfoc->flash_data.limits.max_dq_current) goto invalid;
    switch(ctx->stage){
        case FOC_ENCODER_CAL_LOCK:
            if(FOC_RoutineElapsed(now, ctx->runtime.stage_started_ms, 300U)){
                ctx->reference_raw = ctx->last_raw = hfoc->encoder_runtime.raw_angle;
                ctx->raw_relative = 0;
                ctx->stage = FOC_ENCODER_CAL_APPLY;
            }
            break;
        case FOC_ENCODER_CAL_APPLY:
            if(ctx->step > CAL_STEPS * ctx->revolutions || ctx->direction > 1) goto invalid;
            calibration_field(hfoc);
            ctx->samples = 0;
            ctx->runtime.stage_started_ms = now;
            ctx->stage = FOC_ENCODER_CAL_SAMPLE;
            break;
        case FOC_ENCODER_CAL_SAMPLE: {
            float raw = hfoc->encoder_runtime.raw_angle;
            float delta = raw - ctx->last_raw;
            normalize_angle_pm_pi(&delta);
            ctx->raw_relative += delta;
            ctx->last_raw = raw;
            if(!FOC_RoutineElapsed(now, ctx->runtime.stage_started_ms, CAL_SETTLE_MS)) break;
            float expected = (float)ctx->step * (M_2PIF / CAL_STEPS) * (hfoc->flash_data.motor.direction ? -1.0f : 1.0f);
            float error = expected - ctx->raw_relative;
            float tracking_limit = fminf(0.15f, M_PIF / (2.0f * hfoc->flash_data.motor.pole_pairs));
            if(!isfinite(error) || fabsf(error) > tracking_limit) goto invalid;
            unsigned bin = (unsigned)(raw * (FOC_ENCODER_LUT_SIZE / M_2PIF) + 0.5f) % FOC_ENCODER_LUT_SIZE;
            if(ctx->direction == 0){ctx->forward[bin] += error; ctx->forward_count[bin]++;}
            else {ctx->reverse[bin] += error; ctx->reverse_count[bin]++;}
            if(++ctx->samples == CAL_SAMPLES){
                unsigned total = CAL_STEPS * ctx->revolutions;
                ctx->progress = (uint16_t)((ctx->direction ? 2 * total - ctx->step : ctx->step) * 1000UL / (2 * total));
                if(ctx->direction == 0 && ctx->step == total){ctx->direction = 1; ctx->stage = FOC_ENCODER_CAL_APPLY;}
                else if(ctx->direction == 1 && ctx->step == 0){
                    FOC_SetPhaseVoltages(hfoc, (PhaseVoltagesTypeDef){0});
                    DRV8323_SetHighImpedance(&hfoc->hdrv8323);
                    ctx->stage = FOC_ENCODER_CAL_VALIDATE;
                }else{
                    if(ctx->direction) ctx->step--; else ctx->step++;
                    ctx->stage = FOC_ENCODER_CAL_APPLY;
                }
            }
            break;
        }
        case FOC_ENCODER_CAL_VALIDATE:
            if(!calibration_validate(hfoc)) goto invalid;
            ctx->candidate_valid = 1;
            ctx->stage = FOC_ENCODER_CAL_SAVE;
            break;
        case FOC_ENCODER_CAL_SAVE: {
            /* The old table resides in the candidate buffer after the swap. */
            float old_zero = hfoc->flash_data.encoder.raw_zero;
            float old_offset = hfoc->flash_data.encoder.mechanical_offset;
            uint8_t old_valid = hfoc->flash_data.encoder.correction_valid;
            uint8_t old_enabled = hfoc->flash_data.encoder.correction_enabled;
            uint8_t old_source = hfoc->flash_data.encoder.correction_source;
            uint8_t old_offset_valid = hfoc->flash_data.encoder.offset_valid;
            calibration_swap(hfoc);
            hfoc->flash_data.encoder.correction_valid = hfoc->flash_data.encoder.correction_enabled = 1;
            hfoc->flash_data.encoder.correction_source = ctx->source;
            hfoc->flash_data.encoder.raw_zero = ctx->reference_raw;
            hfoc->flash_data.encoder.mechanical_offset = ctx->reference_raw + FOC_EncoderLUT(hfoc->flash_data.encoder.correction, ctx->reference_raw);
            normalize_angle_0_2pi(&hfoc->flash_data.encoder.mechanical_offset);
            hfoc->flash_data.encoder.offset_valid = 1;
            hfoc->flash_data.controller.anticogging_data_valid = 0;
            /* Persist normal settings, not temporary calibration disable flags. */
            hfoc->flash_data.hfi.hfi_enabled = ctx->saved_hfi;
            hfoc->flash_data.controller.anticogging_FF_enabled = ctx->saved_anticog_ff;
            hfoc->flash_data.controller.current_PID_FF_enabled = ctx->saved_current_ff;
            if(FOC_FLASH_WriteData(&hfoc->flash_data) != FLASH_OK){
                calibration_swap(hfoc);
                hfoc->flash_data.encoder.raw_zero = old_zero;
                hfoc->flash_data.encoder.mechanical_offset = old_offset;
                hfoc->flash_data.encoder.offset_valid = old_offset_valid;
                hfoc->flash_data.encoder.correction_valid = old_valid;
                hfoc->flash_data.encoder.correction_enabled = old_enabled;
                hfoc->flash_data.encoder.correction_source = old_source;
                hfoc->flash_data.controller.anticogging_data_valid = ctx->saved_anticog_valid;
                goto invalid;
            }
            ctx->progress = 1000;
            Debug_SendTextResponse("Encoder calibration saved; anti-cogging recalibration required.\n");
            return FOC_EncoderCalibration_Finish(hfoc, FOC_LOOP_COMPLETED);
        }
        default:
            ctx->runtime.error = FOC_ROUTINE_ERROR_INVALID_STAGE;
            return FOC_EncoderCalibration_Finish(hfoc, FOC_LOOP_ERROR);
    }
    return FOC_LOOP_IN_PROGRESS;
invalid:
    ctx->runtime.error = FOC_ROUTINE_ERROR_INVALID_RESULT;
    return FOC_EncoderCalibration_Finish(hfoc, FOC_LOOP_ERROR);
}
