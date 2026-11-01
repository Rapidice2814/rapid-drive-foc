#ifndef FOC_ROUTINE_H
#define FOC_ROUTINE_H

#include <stdint.h>
#include "PID.h"
#include "FOC_Encoder.h"

typedef enum {
    FOC_LOOP_IN_PROGRESS,
    FOC_LOOP_COMPLETED,
    FOC_LOOP_ERROR,
    FOC_LOOP_CANCELLED
} FOC_LoopStatusTypeDef;

typedef enum {
    FOC_ROUTINE_ERROR_NONE,
    FOC_ROUTINE_ERROR_NOT_STARTED,
    FOC_ROUTINE_ERROR_INVALID_STAGE,
    FOC_ROUTINE_ERROR_INVALID_INPUT,
    FOC_ROUTINE_ERROR_TIMEOUT,
    FOC_ROUTINE_ERROR_ENCODER,
    FOC_ROUTINE_ERROR_CURRENT_SENSE,
    FOC_ROUTINE_ERROR_INVALID_RESULT
} FOC_RoutineErrorTypeDef;

typedef struct {
    uint32_t started_ms;
    uint32_t stage_started_ms;
    FOC_LoopStatusTypeDef status;
    FOC_RoutineErrorTypeDef error;
    const char *detail; /* Static or context-owned reason; never persisted. */
    uint8_t active;
} FOC_RoutineRuntimeTypeDef;

typedef struct {
    uint32_t sequence;
    uint8_t state, status, error;
} FOC_SetupResult;

typedef struct {
    FOC_RoutineRuntimeTypeDef runtime;
    float last_raw, travel, forward_travel, phase;
    uint32_t stage_started_ms;
    uint8_t stage, failed_stage, candidate_poles, candidate_direction;
} FOC_MotorProbeContext;

void FOC_ReportRoutineResult(FOC_HandleTypeDef *hfoc, const char *name,
                            FOC_RoutineRuntimeTypeDef *runtime, unsigned stage);

typedef enum {
    FOC_ALIGNMENT_IDLE,
    FOC_ALIGNMENT_START,
    FOC_ALIGNMENT_SWEEP,
    FOC_ALIGNMENT_SET_ZERO,
    FOC_ALIGNMENT_RELEASE,
    FOC_ALIGNMENT_WAIT_RELEASE,
    FOC_ALIGNMENT_DONE,
    FOC_ALIGNMENT_FAILED,
    FOC_ALIGNMENT_CANCELLED
} FOC_AlignmentStageTypeDef;

typedef struct {
    FOC_RoutineRuntimeTypeDef runtime;
    FOC_AlignmentStageTypeDef stage;
    FOC_AlignmentStageTypeDef failed_stage;
    uint16_t sweep_index;
    float phase;
    float magnitude;
    float candidate_offset;
    float candidate_raw_zero;
    uint8_t candidate_valid;
} FOC_AlignmentContextTypeDef;

typedef enum {
    FOC_CURRENT_CAL_IDLE,
    FOC_CURRENT_CAL_START,
    FOC_CURRENT_CAL_ENABLE,
    FOC_CURRENT_CAL_COLLECT,
    FOC_CURRENT_CAL_REPORT,
    FOC_CURRENT_CAL_WAIT_FINISH,
    FOC_CURRENT_CAL_DONE,
    FOC_CURRENT_CAL_FAILED,
    FOC_CURRENT_CAL_CANCELLED
} FOC_CurrentCalibrationStageTypeDef;

typedef struct {
    FOC_RoutineRuntimeTypeDef runtime;
    FOC_CurrentCalibrationStageTypeDef stage;
    FOC_CurrentCalibrationStageTypeDef failed_stage;
    uint16_t sample_count;
    float sum_a, sum_b, sum_c;
    float candidate_a, candidate_b, candidate_c;
    uint8_t calibration_enabled;
    uint8_t candidate_valid;
} FOC_CurrentCalibrationContextTypeDef;

#define FOC_IDENTIFICATION_SAMPLES 25U

typedef enum {
    FOC_IDENT_IDLE, FOC_IDENT_START, FOC_IDENT_PREPARE,
    FOC_IDENT_REMOVE_VOLTAGE, FOC_IDENT_WAIT_CAPTURE, FOC_IDENT_CAPTURE,
    FOC_IDENT_SELECT_MODEL, FOC_IDENT_EVALUATE_MODEL, FOC_IDENT_NEXT_PHASE,
    FOC_IDENT_VALIDATE, FOC_IDENT_WAIT_FINISH,
    FOC_IDENT_DONE, FOC_IDENT_FAILED, FOC_IDENT_CANCELLED
} FOC_IdentificationStageTypeDef;

typedef enum {
    FOC_IDENT_FIT_BASELINE, FOC_IDENT_FIT_INDUCTANCE,
    FOC_IDENT_FIT_RESISTANCE, FOC_IDENT_FIT_APPLY
} FOC_IdentificationFitStageTypeDef;

typedef struct {
    FOC_RoutineRuntimeTypeDef runtime;
    FOC_IdentificationStageTypeDef stage, failed_stage;
    FOC_IdentificationFitStageTypeDef fit_stage;
    uint32_t wait_ms, iterations;
    uint16_t capture_index, model_index;
    uint8_t selector, attempt, candidate_valid;
    float time[FOC_IDENTIFICATION_SAMPLES];
    float measured[FOC_IDENTIFICATION_SAMPLES];
    float predicted[FOC_IDENTIFICATION_SAMPLES];
    float resistance[3], inductance[3];
    float estimated_r, estimated_l, model_r, model_l;
    float error, baseline_error, inductance_error;
    float candidate_r, candidate_l;
} FOC_IdentificationContextTypeDef;

typedef enum {
    FOC_PID_TUNE_IDLE, FOC_PID_TUNE_START, FOC_PID_TUNE_COMPUTE,
    FOC_PID_TUNE_DONE, FOC_PID_TUNE_FAILED, FOC_PID_TUNE_CANCELLED
} FOC_PIDAutotuneStageTypeDef;

typedef struct {
    FOC_RoutineRuntimeTypeDef runtime;
    FOC_PIDAutotuneStageTypeDef stage, failed_stage;
    float candidate_kp, candidate_ki;
    uint8_t candidate_valid;
} FOC_PIDAutotuneContextTypeDef;

typedef enum {
    FOC_ALIGNMENT_TEST_IDLE, FOC_ALIGNMENT_TEST_START, FOC_ALIGNMENT_TEST_ROTATE,
    FOC_ALIGNMENT_TEST_VALIDATE, FOC_ALIGNMENT_TEST_WAIT_FINISH,
    FOC_ALIGNMENT_TEST_DONE, FOC_ALIGNMENT_TEST_FAILED, FOC_ALIGNMENT_TEST_CANCELLED
} FOC_AlignmentTestStageTypeDef;

typedef struct {
    FOC_RoutineRuntimeTypeDef runtime;
    FOC_AlignmentTestStageTypeDef stage, failed_stage;
    uint32_t wait_ms;
    uint16_t sample_index;
    float magnitude, reference_angle, electrical_angle, error_cw, error_ccw;
    uint8_t direction, dir_swapped, attempt, candidate_direction, candidate_valid;
} FOC_AlignmentTestContextTypeDef;

typedef enum {
    FOC_ANTICOG_IDLE, FOC_ANTICOG_START, FOC_ANTICOG_POSITION,
    FOC_ANTICOG_COLLECT, FOC_ANTICOG_REPORT, FOC_ANTICOG_WAIT_FINISH,
    FOC_ANTICOG_DONE, FOC_ANTICOG_FAILED, FOC_ANTICOG_CANCELLED
} FOC_AntiCoggingStageTypeDef;

typedef struct {
    FOC_RoutineRuntimeTypeDef runtime;
    FOC_AntiCoggingStageTypeDef stage, failed_stage;
    PIDValuesTypeDef saved_position_gains;
    uint32_t wait_ms, point_started_ms;
    uint16_t sample_index, report_index;
    float filtered_current;
    float turn_base, current_mean, current_m2, position_min, position_max, speed_sum;
    float window_error_max, window_command_max;
    float last_error, last_span, last_speed, last_std, last_mean;
    uint32_t window_started_ms;
    uint16_t window_samples;
    uint8_t approaching;
    uint8_t divider, direction, hold_count, report_direction;
    uint8_t gains_saved, candidate_valid, revolution, revolutions, saved_ff;
    uint16_t progress;
} FOC_AntiCoggingContextTypeDef;

typedef struct {
    float electrical_angle;
} FOC_OpenLoopContextTypeDef;

typedef enum {
    FOC_ENCODER_CAL_IDLE, FOC_ENCODER_CAL_LOCK, FOC_ENCODER_CAL_APPLY,
    FOC_ENCODER_CAL_SAMPLE, FOC_ENCODER_CAL_VALIDATE, FOC_ENCODER_CAL_SAVE,
    FOC_ENCODER_CAL_DONE, FOC_ENCODER_CAL_FAILED, FOC_ENCODER_CAL_CANCELLED
} FOC_EncoderCalibrationStageTypeDef;

typedef struct {
    FOC_RoutineRuntimeTypeDef runtime;
    FOC_EncoderCalibrationStageTypeDef stage, failed_stage;
    float forward[FOC_ENCODER_LUT_SIZE], reverse[FOC_ENCODER_LUT_SIZE];
    uint16_t forward_count[FOC_ENCODER_LUT_SIZE], reverse_count[FOC_ENCODER_LUT_SIZE];
    float reference_raw, last_raw, raw_relative, current, max_disagreement;
    char validation_detail[128]; /* Retains measured failure details through cleanup. */
    AngleSimulator saved_simulator;
    uint32_t release_started_ms;
    uint8_t owns_simulator;
    uint16_t step, progress;
    uint8_t direction, samples, saved_anticog_ff, saved_current_ff;
    uint8_t saved_anticog_valid, source, candidate_valid;
    uint8_t pole_pairs, motor_direction, revolutions;
} FOC_EncoderCalibrationContextTypeDef;

/** @brief Sequential table-upload state; encoder staging reuses its idle workspace. */
typedef struct {
    uint32_t crc;
    uint16_t received;
    uint8_t active, kind, source;
} FOC_CalibrationUploadTypeDef;

/**
 * @brief Tests elapsed milliseconds using unsigned, wraparound-safe subtraction.
 * @note Intervals must be shorter than one full uint32_t clock period.
 */
static inline uint8_t FOC_RoutineElapsed(uint32_t now, uint32_t since, uint32_t interval){
    return (uint32_t)(now - since) >= interval;
}

#endif /* FOC_ROUTINE_H */
