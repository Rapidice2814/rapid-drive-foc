#ifndef TEST_FOC_HANDLE_H
#define TEST_FOC_HANDLE_H
/* Suppress the hardware handle when a source includes its adjacent header. */
#define FOC_HANDLE_H
#include "FOC_Loops.h"
#include "FOC_States.h"
#include "Utils.h"
#include "FOC_Config.h"
#include "FOC_Flash.h"
#include "Filters.h"
#include "AS5047P_Driver.h"
typedef struct {float a, b, c;} PhaseCurrentsTypeDef;
typedef PhaseCurrentsTypeDef PhaseVoltagesTypeDef;
typedef struct {float alpha, beta;} ABVoltagesTypeDef;
typedef struct {float d, q;} DQCurrentsTypeDef;
typedef enum {FOC_OK, FOC_ERROR} FOC_StatusTypeDef;
typedef enum {DRV8323_OK, DRV8323_ERROR} DRV8323_StatusTypeDef;
typedef struct {int high_impedance;} DRV8323_HandleTypeDef;
struct FOC_Handle {
    FOC_StateTypeDef state, previous_state, next_state;
    uint32_t active_errors, latched_errors;
    uint8_t adc_calibrated;
    BiquadFilter dq_current_d_filter, dq_current_q_filter;
    FOC_AlignmentContextTypeDef alignment;
    FOC_CurrentCalibrationContextTypeDef current_calibration;
    FOC_EncoderCalibrationContextTypeDef encoder_calibration;
    FOC_EncoderRuntimeTypeDef encoder_runtime;
    char routine_result[512];
    FOC_SetupResult setup_result;
    FOC_MotorProbeContext motor_probe;
    float setup_voltage;
    AS5047P_HandleTypeDef has5047p;
    volatile uint32_t *pencoder_count;
    FOC_IdentificationContextTypeDef identification;
    FOC_PIDAutotuneContextTypeDef pid_autotune;
    FOC_AlignmentTestContextTypeDef alignment_test;
    FOC_AntiCoggingContextTypeDef anticogging;
    uint8_t calibration_revolutions[2];
    FOC_CalibrationUploadTypeDef calibration_upload;
    FOC_OpenLoopContextTypeDef open_loop;
    uint8_t run_loop_divider, checklist_step;
    FLASH_DataTypeDef flash_data;
    struct {PhaseCurrentsTypeDef phase_current; float motor_temp, mosfet_temp, vbus; uint8_t current_sense_saturated;} adc_values;
    DQCurrentsTypeDef dq_current_setpoint;
    DQCurrentsTypeDef dq_current;
    PIDControllerTypeDef pid_current_d, pid_current_q, pid_position, pid_speed, pid_anticogging;
    float speed_setpoint, angle_setpoint, ibus;
    float encoder_angle_electrical, encoder_speed_electrical;
    float encoder_angle_mechanical_wrapped, encoder_speed_mechanical;
    float encoder_angle_mechanical_wrapped_prev, encoder_angle_mechanical_unwrapped;
    int32_t encoder_full_rotations;
    PhaseVoltagesTypeDef phase_voltage;
    DRV8323_HandleTypeDef hdrv8323;
};
void FOC_SetPhaseVoltages(FOC_HandleTypeDef *, PhaseVoltagesTypeDef);
PhaseVoltagesTypeDef InvClarke_transform(ABVoltagesTypeDef);
FOC_StatusTypeDef FOC_SetEncoderZero(FOC_HandleTypeDef *);
DRV8323_StatusTypeDef DRV8323_CSACALStart(DRV8323_HandleTypeDef *);
DRV8323_StatusTypeDef DRV8323_CSACALStop(DRV8323_HandleTypeDef *);
void DRV8323_SetHighImpedance(DRV8323_HandleTypeDef *);
DRV8323_StatusTypeDef DRV8323_ExitHighImpedance(DRV8323_HandleTypeDef *);
int DRV8323_CheckFault(DRV8323_HandleTypeDef *);
#endif
