#include <math.h>
#include "FOC_Handle.h"
#include "FOC_Loops.h"
#include "FOC_Diagnostics.h"
#include "FOC_USB_Debug.h"
#include "Cordic.h"
#include "Utils.h"

#define PROBE_TURNS 8.0f
#define PROBE_SWEEP_MS 8000U
#define PROBE_HOLD_MS 500U
#define PROBE_TIMEOUT_MS 25000U

static void probe_field(FOC_HandleTypeDef *h, float phase, float scale){
    float c, s;
    Cordic_CalculateSinCos(remainderf(phase, M_2PIF), &c, &s);
    float voltage = (h->setup_voltage > 0 ? h->setup_voltage : 0.5f) * scale;
    FOC_SetPhaseVoltages(h, InvClarke_transform((ABVoltagesTypeDef){voltage*c, voltage*s}));
}

void FOC_MotorProbe_Start(FOC_HandleTypeDef *h){
    h->motor_probe = (FOC_MotorProbeContext){.runtime = {.active = 1,
        .status = FOC_LOOP_IN_PROGRESS, .started_ms = HAL_GetTick()}, .stage_started_ms = HAL_GetTick()};
    const FOC_EncoderSourceTypeDef *source = FOC_EncoderSource(h->flash_data.encoder.source_id);
    float voltage = h->setup_voltage > 0 ? h->setup_voltage : 0.5f;
    if(!h->adc_calibrated || !h->encoder_runtime.healthy || !source || !source->calibratable ||
       !isfinite(voltage) || voltage <= 0 || !isfinite(h->flash_data.limits.max_dq_voltage) ||
       !isfinite(h->flash_data.limits.max_dq_current) || h->flash_data.limits.max_dq_current <= 0 ||
       voltage > h->flash_data.limits.max_dq_voltage){
        h->motor_probe.runtime.error = FOC_ROUTINE_ERROR_INVALID_INPUT;
        h->motor_probe.runtime.detail = "Set a physical encoder, calibrated ADC and valid setup voltage/current limits";
        FOC_MotorProbe_Finish(h, FOC_LOOP_ERROR); return;
    }
    h->dq_current_setpoint = (DQCurrentsTypeDef){0};
    FOC_SetPhaseVoltages(h, (PhaseVoltagesTypeDef){0});
    if(DRV8323_ExitHighImpedance(&h->hdrv8323) != DRV8323_OK){
        h->motor_probe.runtime.error = FOC_ROUTINE_ERROR_INVALID_INPUT;
        h->motor_probe.runtime.detail = "Could not enable inverter for motor detection";
        FOC_MotorProbe_Finish(h, FOC_LOOP_ERROR);
    }
}

FOC_LoopStatusTypeDef FOC_MotorProbe_Finish(FOC_HandleTypeDef *h, FOC_LoopStatusTypeDef result){
    FOC_MotorProbeContext *c = &h->motor_probe;
    if(!c->runtime.active) return c->runtime.status;
    FOC_SetPhaseVoltages(h, (PhaseVoltagesTypeDef){0});
    DRV8323_SetHighImpedance(&h->hdrv8323);
    h->dq_current_setpoint = (DQCurrentsTypeDef){0};
    if(result == FOC_LOOP_COMPLETED){
        if(!c->candidate_poles){result = FOC_LOOP_ERROR; c->runtime.error = FOC_ROUTINE_ERROR_INVALID_RESULT;}
        else {
            h->flash_data.motor.pole_pairs = c->candidate_poles;
            h->flash_data.motor.direction = c->candidate_direction;
            h->flash_data.encoder.offset_valid = h->flash_data.encoder.correction_valid = h->flash_data.encoder.correction_enabled = 0;
            h->flash_data.controller.anticogging_data_valid = h->flash_data.controller.anticogging_FF_enabled = 0;
            FOC_EncoderResetHistory(h);
            Debug_SendTextResponse("Detected motor: %u pole pairs (%u poles), direction=%u. Encoder alignment required.\n",
                c->candidate_poles, 2u*c->candidate_poles, c->candidate_direction);
        }
    }
    if(result == FOC_LOOP_CANCELLED && (h->active_errors || h->latched_errors))
        c->runtime.detail = "Motor detection interrupted by fault; inspect fault flags";
    c->failed_stage = c->stage;
    c->runtime.active = 0; c->runtime.status = result;
    FOC_ReportRoutineResult(h, "Motor direction/pole detection", &c->runtime, c->stage);
    return result;
}

FOC_LoopStatusTypeDef FOC_MotorProbe_Update(FOC_HandleTypeDef *h){
    FOC_MotorProbeContext *c = &h->motor_probe;
    if(!c->runtime.active) return c->runtime.status;
    uint32_t now = HAL_GetTick(), elapsed = now - c->stage_started_ms;
    if(now - c->runtime.started_ms > PROBE_TIMEOUT_MS){
        c->runtime.error = FOC_ROUTINE_ERROR_TIMEOUT; c->runtime.detail = "Motor direction/pole detection timed out"; goto failed;
    }
    float raw = h->encoder_runtime.raw_angle;
    if(!h->encoder_runtime.healthy || !isfinite(raw) || h->active_errors || h->latched_errors){
        c->runtime.error = FOC_ROUTINE_ERROR_ENCODER; c->runtime.detail = "Encoder unavailable or hardware fault during detection"; goto failed;
    }
    if(c->stage){
        float delta = remainderf(raw - c->last_raw, M_2PIF);
        c->travel += delta;
    }
    c->last_raw = raw;
    if(c->stage == 0){
        probe_field(h, 0, fminf(1, elapsed / 200.0f));
        if(elapsed < PROBE_HOLD_MS) return FOC_LOOP_IN_PROGRESS;
        c->travel = 0; c->stage = 1; c->stage_started_ms = now;
    }else if(c->stage == 1 || c->stage == 3){
        float fraction = fminf(1, elapsed / (float)PROBE_SWEEP_MS);
        c->phase = PROBE_TURNS * M_2PIF * (c->stage == 1 ? fraction : 1 - fraction);
        probe_field(h, c->phase, 1);
        if(elapsed < PROBE_SWEEP_MS) return FOC_LOOP_IN_PROGRESS;
        c->stage++; c->stage_started_ms = now;
    }else if(c->stage == 2){
        probe_field(h, PROBE_TURNS*M_2PIF, 1);
        if(elapsed < PROBE_HOLD_MS) return FOC_LOOP_IN_PROGRESS;
        c->forward_travel = c->travel;
        c->travel = 0; c->stage = 3; c->stage_started_ms = now;
    }else if(c->stage == 4){
        probe_field(h, 0, 1);
        if(elapsed < PROBE_HOLD_MS) return FOC_LOOP_IN_PROGRESS;
        float forward = c->forward_travel, reverse = c->travel;
        if(fabsf(forward) < 0.1f || fabsf(reverse) < 0.1f || forward*reverse >= 0){
            c->runtime.detail = "Insufficient/opposite-direction travel: motor stalled, magnet lost or encoder invalid"; goto invalid;
        }
        float a = PROBE_TURNS*M_2PIF/fabsf(forward), b = PROBE_TURNS*M_2PIF/fabsf(reverse);
        float rounded = floorf((a+b)*0.5f + 0.5f);
        if(rounded < 1 || rounded > 255 || fabsf(a-rounded) > 0.2f || fabsf(b-rounded) > 0.2f || fabsf(a-b) > 0.25f){
            c->runtime.detail = "Pole estimate not close to a consistent integer in both directions; check tracking, field strength and encoder"; goto invalid;
        }
        c->candidate_poles = (uint8_t)rounded; c->candidate_direction = forward < 0;
        return FOC_MotorProbe_Finish(h, FOC_LOOP_COMPLETED);
    }else{c->runtime.detail = "Invalid motor detection stage"; goto invalid;}
    return FOC_LOOP_IN_PROGRESS;
invalid:
    c->runtime.error = FOC_ROUTINE_ERROR_INVALID_RESULT;
failed:
    return FOC_MotorProbe_Finish(h, FOC_LOOP_ERROR);
}
