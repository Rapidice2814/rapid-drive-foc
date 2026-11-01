#include "FOC_Configuration.h"
#include "FOC_Handle.h"
#include "FOC_Config.h"
#include "FOC_USB_Debug.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    const char *name, *type, *unit, *label;
    size_t offset, size;
    float minimum, maximum;
    uint32_t states;
    uint8_t group;
} ConfigDescriptor;
#define CONFIG_DESCRIPTOR(symbol, group, type, member, min, max, unit, states, label) \
    {#member, #type, unit, label, offsetof(FLASH_DataTypeDef, member), sizeof(((FLASH_DataTypeDef*)0)->member), min, max, states, FOC_CONFIG_##group},
static const ConfigDescriptor fields[] = {FOC_CONFIGURATION_FIELDS(CONFIG_DESCRIPTOR)};
#undef CONFIG_DESCRIPTOR
#define CONFIG_GROUP_NAME(symbol, label) label,
static const char *const groups[] = {FOC_CONFIGURATION_GROUPS(CONFIG_GROUP_NAME)};
#undef CONFIG_GROUP_NAME
_Static_assert(FOC_CONFIG_FIELD_COUNT > 0 && FOC_CONFIG_FIELD_COUNT <= 48, "Configuration exceeds bounded transaction capacity");
_Static_assert(FOC_CONFIG_GROUP_COUNT <= 255, "Configuration group IDs exceed wire width");
#define CONFIG_NATIVE_f(member) _Generic(((FLASH_DataTypeDef*)0)->member, float: 1, default: 0)
#define CONFIG_NATIVE_u32(member) _Generic(((FLASH_DataTypeDef*)0)->member, uint8_t: 1, uint16_t: 1, uint32_t: 1, default: 0)
#define CONFIG_NATIVE_bool(member) CONFIG_NATIVE_u32(member)
#define CONFIG_NATIVE_i32(member) _Generic(((FLASH_DataTypeDef*)0)->member, int32_t: 1, default: 0)
#define CONFIG_ASSERT(symbol, group, type, member, min, max, unit, states, label) \
    _Static_assert(CONFIG_NATIVE_##type(member), "Configuration wire/native type mismatch"); \
    _Static_assert(sizeof(((FLASH_DataTypeDef*)0)->member) <= 4, "Configuration is scalar only"); \
    _Static_assert(sizeof(#member) + sizeof(label) + sizeof(unit) < 150, "Configuration metadata row too long");
FOC_CONFIGURATION_FIELDS(CONFIG_ASSERT)
#undef CONFIG_ASSERT

static FOC_ConfigResult ok(void){return (FOC_ConfigResult){0, ""};}
static FOC_ConfigResult reject(uint16_t code, const char *message){return (FOC_ConfigResult){code, message};}

/** @brief Returns a stable semantic key, or NULL for an unknown session ID. */
const char *FOC_ConfigurationName(uint8_t id){return id < FOC_CONFIG_FIELD_COUNT ? fields[id].name : NULL;}

/** @brief Formats one finite bound without floating-point printf support. */
static void format_bound(char out[32], float value){
    /* nano.specs does not include floating-point printf. Emit a canonical
     * nine-significant-digit decimal without pulling it into the firmware. */
    if(value == 0){strcpy(out, "0"); return;}
    double scaled = value;
    int exponent = 0;
    if(scaled < 0) scaled = -scaled;
    while(scaled >= 10.0){scaled /= 10.0; exponent++;}
    while(scaled < 1.0){scaled *= 10.0; exponent--;}
    uint32_t mantissa = (uint32_t)(scaled * 100000000.0 + 0.5);
    snprintf(out, 32, "%s%lue%d", value < 0 ? "-" : "", (unsigned long)mantissa, exponent - 8);
}

/** @brief PID metadata uses the same bounds and states as PID writes. */
int FOC_PIDFormat(char *out, size_t capacity, uint8_t id, const char *name){
    char minimum[32], maximum[32];
    format_bound(minimum, CONFIG_ABS_PID_GAIN_MIN);
    format_bound(maximum, CONFIG_ABS_PID_GAIN_MAX);
    return snprintf(out, capacity, "%u\t%s\t%s\t%s\t%lu\n", id, name,
                    minimum, maximum, (unsigned long)FOC_PID_WRITABLE_STATES);
}

static uint8_t pid_gains_valid(PIDValuesTypeDef gains){
    return isfinite(gains.Kp) && isfinite(gains.Ki) && isfinite(gains.Kd) &&
           gains.Kp >= CONFIG_ABS_PID_GAIN_MIN && gains.Ki >= CONFIG_ABS_PID_GAIN_MIN && gains.Kd >= CONFIG_ABS_PID_GAIN_MIN &&
           gains.Kp <= CONFIG_ABS_PID_GAIN_MAX && gains.Ki <= CONFIG_ABS_PID_GAIN_MAX && gains.Kd <= CONFIG_ABS_PID_GAIN_MAX;
}

/** @brief Serializes bounded text metadata identically for discovery and hashing. */
int FOC_ConfigurationFormat(char *out, size_t capacity, uint8_t id){
    if(id >= FOC_CONFIG_FIELD_COUNT) return -1;
    const ConfigDescriptor *d = &fields[id];
    if(!isfinite(d->minimum) || !isfinite(d->maximum) || d->minimum > d->maximum) return -1;
    char minimum[32], maximum[32];
    format_bound(minimum, d->minimum); format_bound(maximum, d->maximum);
    return snprintf(out, capacity, "%u\t%u\t%s\t%s\t%s\t%s\t%s\t%lu\t%s\t%s\n",
                    id, d->group, groups[d->group], d->type, minimum, maximum,
                    d->unit, (unsigned long)d->states, d->name, d->label);
}

/** @brief Returns group ownership, or 255 for an unknown field. */
uint8_t FOC_ConfigurationGroup(uint8_t id){return id < FOC_CONFIG_FIELD_COUNT ? fields[id].group : 255;}

/** @brief Reads a scalar as four little-endian native wire bytes on STM32. */
uint32_t FOC_ConfigurationRead(const FOC_HandleTypeDef *h, uint8_t id){
    uint32_t value = 0;
    if(id < FOC_CONFIG_FIELD_COUNT) memcpy(&value, (const uint8_t*)&h->flash_data + fields[id].offset, fields[id].size);
    return value;
}

static float number(uint32_t raw, uint8_t id){
    if(!strcmp(fields[id].type, "i32")) return (float)(int32_t)raw;
    if(strcmp(fields[id].type, "f")) return (float)raw;
    float value; memcpy(&value, &raw, 4); return value;
}

/**
 * @brief Validates a complete candidate group before applying any field.
 * @note Called from the serialized FOC main loop by USB/CAN, never from an ISR.
 *       No full flash-record copy or interrupt masking is required. Persistence
 *       remains explicit. Invalid requests leave settings and calibration intact.
 */
FOC_ConfigResult FOC_ConfigurationSet(FOC_HandleTypeDef *h, uint8_t group,
                                    const FOC_ConfigUpdate *updates, uint8_t count){
    if(group >= FOC_CONFIG_GROUP_COUNT || !updates || !count || count > FOC_CONFIG_FIELD_COUNT)
        return reject(DEBUG_ERR_UNKNOWN_ID, "Unknown configuration group or invalid update count.");
    if(h->calibration_upload.active) return reject(DEBUG_ERR_INVALID_STATE, "Abort the unfinished table upload first.");
    if(!isfinite(h->encoder_speed_mechanical) || fabsf(h->encoder_speed_mechanical) > 0.1f)
        return reject(DEBUG_ERR_MOTOR_MOVING, "Configuration requires a stationary motor.");
    if(h->state != FOC_STATE_IDLE && h->state != FOC_STATE_ERROR)
        return reject(DEBUG_ERR_MUST_BE_IDLE, "Configuration requires IDLE; only Limits may also be changed in ERROR.");
    uint32_t candidate[FOC_CONFIG_FIELD_COUNT];
    uint8_t changed[FOC_CONFIG_FIELD_COUNT] = {0}, any_changed = 0;
    for(uint8_t i = 0; i < FOC_CONFIG_FIELD_COUNT; i++) candidate[i] = FOC_ConfigurationRead(h, i);
    for(uint8_t n = 0; n < count; n++){
        uint8_t id = updates[n].id;
        if(id >= FOC_CONFIG_FIELD_COUNT || fields[id].group != group)
            return reject(DEBUG_ERR_UNKNOWN_ID, "Configuration field does not belong to this group.");
        if(changed[id]) return reject(DEBUG_ERR_OUT_OF_RANGE, "Duplicate field in configuration transaction.");
        if(!(fields[id].states & (1u << h->state)))
            return reject(DEBUG_ERR_INVALID_STATE, "This configuration field is not writable in the current state.");
        if(fields[id].size < 4 && updates[n].raw >= (1u << (8u * fields[id].size)))
            return reject(DEBUG_ERR_OUT_OF_RANGE, "Configuration value does not fit native integer storage.");
        changed[id] = 1;
        any_changed |= candidate[id] != updates[n].raw;
        candidate[id] = updates[n].raw;
    }
    for(uint8_t i = 0; i < FOC_CONFIG_FIELD_COUNT; i++){
        if(fields[i].group != group) continue;
        float v = number(candidate[i], i);
        /* Zero explicitly means unset, including in a partially supplied group. */
        if(v == 0) continue;
        if(!isfinite(v) || v < fields[i].minimum || v > fields[i].maximum)
            return reject(DEBUG_ERR_OUT_OF_RANGE, fields[i].name);
    }
#define VALUE(symbol) number(candidate[FOC_CONFIG_##symbol], FOC_CONFIG_##symbol)
    if(group == FOC_CONFIG_LIMITS && VALUE(UNDERVOLTAGE) > 0 && VALUE(OVERVOLTAGE) > 0 && VALUE(UNDERVOLTAGE) >= VALUE(OVERVOLTAGE))
        return reject(DEBUG_ERR_OUT_OF_RANGE, "Bus undervoltage trip must be below overvoltage trip.");
    if(group == FOC_CONFIG_LIMITS && VALUE(UNDERVOLTAGE) > 0 && VALUE(MAX_VOLTAGE) > VALUE(UNDERVOLTAGE) * M_1_SQRT3F)
        return reject(DEBUG_ERR_OUT_OF_RANGE, "Voltage limit must fit undervoltage trip / sqrt(3) across the permitted bus range.");
    if((group == FOC_CONFIG_LIMITS || group == FOC_CONFIG_ENCODER) &&
       VALUE(MAX_CURRENT) > 0 && (!isfinite(VALUE(CAL_CURRENT)) || VALUE(CAL_CURRENT) > VALUE(MAX_CURRENT)))
        return reject(DEBUG_ERR_OUT_OF_RANGE, "Encoder calibration D current must not exceed the configured current limit.");
    if(group == FOC_CONFIG_CONTROLLER && VALUE(CURRENT_FF) &&
       (!isfinite(VALUE(RESISTANCE)) || VALUE(RESISTANCE) <= 0 || !isfinite(VALUE(INDUCTANCE)) || VALUE(INDUCTANCE) <= 0))
        return reject(DEBUG_ERR_OUT_OF_RANGE, "Current feed-forward requires valid positive motor R and L.");
#undef VALUE
    /* Commit is bounded and cannot be observed halfway through by control code. */
    for(uint8_t i = 0; i < FOC_CONFIG_FIELD_COUNT; i++){
        if(changed[i]) memcpy((uint8_t*)&h->flash_data + fields[i].offset, &candidate[i], fields[i].size);
    }
    if(any_changed){
        h->flash_data.controller.anticogging_data_valid = 0;
        h->flash_data.controller.anticogging_FF_enabled = 0;
        if(group == FOC_CONFIG_MOTOR && (changed[FOC_CONFIG_POLE_PAIRS] || changed[FOC_CONFIG_DIRECTION])){
            h->flash_data.encoder.offset_valid = 0;
            h->flash_data.encoder.correction_valid = h->flash_data.encoder.correction_enabled = 0;
            FOC_EncoderResetHistory(h);
        }
        PID_Reset(&h->pid_current_d); PID_Reset(&h->pid_current_q);
        PID_Reset(&h->pid_speed); PID_Reset(&h->pid_position);
    }
    return ok();
}

/** @brief Checks only the settings consumed by the requested motor operation. */
FOC_ConfigResult FOC_ConfigurationReady(const FOC_HandleTypeDef *h, FOC_StateTypeDef operation){
    const struct FLASH_ControllerParameters *controller = &h->flash_data.controller;
    if(!pid_gains_valid(controller->PID_gains_d) || !pid_gains_valid(controller->PID_gains_q) ||
       !pid_gains_valid(controller->PID_gains_speed) || !pid_gains_valid(controller->PID_gains_position) ||
       !pid_gains_valid(h->flash_data.anticogging_position_gains))
        return reject(DEBUG_ERR_OUT_OF_RANGE, "Invalid PID gains.");
    uint8_t required[FOC_CONFIG_FIELD_COUNT] = {0};
    for(uint8_t i = 0; i < FOC_CONFIG_FIELD_COUNT; i++){
        float v = number(FOC_ConfigurationRead(h, i), i);
        if(!isfinite(v) || (v != 0 && (v < fields[i].minimum || v > fields[i].maximum)))
            return reject(DEBUG_ERR_OUT_OF_RANGE, fields[i].name);
        required[i] = fields[i].group == FOC_CONFIG_LIMITS;
    }
    if(operation != FOC_STATE_IDENTIFY && operation != FOC_STATE_MOTOR_PROBE && operation != FOC_STATE_BOOTUP_SOUND)
        required[FOC_CONFIG_POLE_PAIRS] = 1;
    if(operation == FOC_STATE_RUN || operation == FOC_STATE_PID_AUTOTUNE || operation == FOC_STATE_ANTICOGGING)
        required[FOC_CONFIG_RESISTANCE] = required[FOC_CONFIG_INDUCTANCE] = 1;
    if(operation == FOC_STATE_PID_AUTOTUNE || operation == FOC_STATE_CHECKLIST) required[FOC_CONFIG_BANDWIDTH] = 1;
    if(operation == FOC_STATE_ENCODER_CALIBRATION) required[FOC_CONFIG_CAL_CURRENT] = 1;
    for(uint8_t i = 0; i < FOC_CONFIG_FIELD_COUNT; i++){
        if(required[i] && number(FOC_ConfigurationRead(h, i), i) <= 0)
            return reject(DEBUG_ERR_OUT_OF_RANGE, fields[i].name);
    }
    const struct FLASH_Limits *l = &h->flash_data.limits;
    if(l->vbus_undervoltage_trip_level >= l->vbus_overvoltage_trip_level ||
       l->max_dq_voltage > l->vbus_undervoltage_trip_level * M_1_SQRT3F ||
       h->flash_data.encoder.calibration_current > l->max_dq_current)
        return reject(DEBUG_ERR_OUT_OF_RANGE, "Inconsistent operating limits.");
    if(operation == FOC_STATE_RUN || operation == FOC_STATE_ANTICOGGING){
        const struct FLASH_ControllerParameters *c = &h->flash_data.controller;
        if((c->PID_gains_d.Kp <= 0 && c->PID_gains_d.Ki <= 0) ||
           (c->PID_gains_q.Kp <= 0 && c->PID_gains_q.Ki <= 0))
            return reject(DEBUG_ERR_OUT_OF_RANGE, "Configure or tune current PI gains first.");
        if(operation == FOC_STATE_ANTICOGGING && h->flash_data.anticogging_position_gains.Kp <= 0)
            return reject(DEBUG_ERR_OUT_OF_RANGE, "Configure anti-cogging position gains first.");
    }
    return ok();
}

/** @brief Applies validated functional targets; USB and CAN share these rules. */
FOC_ConfigResult FOC_SetValidatedMode(FOC_HandleTypeDef *h, uint8_t mode){
    if((h->state != FOC_STATE_IDLE && h->state != FOC_STATE_RUN) || h->calibration_upload.active)
        return reject(DEBUG_ERR_INVALID_STATE, "Control mode requires IDLE/RUN and no calibration upload.");
    if(h->active_errors || h->latched_errors) return reject(h->active_errors ? DEBUG_ERR_ACTIVE_FAULTS : DEBUG_ERR_LATCHED_FAULTS, "Faults prevent control mode changes.");
    const PIDValuesTypeDef *gains = mode == CONTROL_MODE_SPEED ? &h->flash_data.controller.PID_gains_speed :
                                    mode == CONTROL_MODE_POSITION ? &h->flash_data.controller.PID_gains_position : NULL;
    if(gains && (!isfinite(gains->Kp) || !isfinite(gains->Ki) || !isfinite(gains->Kd) ||
                 gains->Kp < 0 || gains->Ki < 0 || gains->Kd < 0 ||
                 (gains->Kp == 0 && gains->Ki == 0)))
        return reject(DEBUG_ERR_OUT_OF_RANGE, "Configure the selected controller's gains first.");
    if(mode > CONTROL_MODE_SPEED || FOC_SetControlMode(h, (ControlModeTypeDef)mode))
        return reject(DEBUG_ERR_OUT_OF_RANGE, "Unknown control mode.");
    return ok();
}

/** @brief Applies one target without bypassing state, mode or current limits. */
FOC_ConfigResult FOC_SetTarget(FOC_HandleTypeDef *h, uint8_t target, float value){
    if(!isfinite(value)) return reject(DEBUG_ERR_OUT_OF_RANGE, "Target must be finite.");
    if(h->state != FOC_STATE_RUN || h->calibration_upload.active)
        return reject(DEBUG_ERR_INVALID_STATE, "Targets require RUN and no table upload.");
    if(h->active_errors || h->latched_errors) return reject(h->active_errors ? DEBUG_ERR_ACTIVE_FAULTS : DEBUG_ERR_LATCHED_FAULTS, "Faults prevent target changes.");
    if(target > FOC_TARGET_TORQUE) return reject(DEBUG_ERR_UNKNOWN_ID, "Unknown target operation.");
    ControlModeTypeDef mode = h->flash_data.controller.control_mode;
    if((target == FOC_TARGET_SPEED && mode != CONTROL_MODE_SPEED) ||
       (target == FOC_TARGET_POSITION && mode != CONTROL_MODE_POSITION) ||
       ((target == FOC_TARGET_CURRENT_D || target == FOC_TARGET_CURRENT_Q || target == FOC_TARGET_TORQUE) && mode != CONTROL_MODE_OPENLOOP))
        return reject(DEBUG_ERR_INVALID_STATE, "Target does not match the active control mode.");
    if(target == FOC_TARGET_TORQUE){
        float kt = h->flash_data.motor.torque_constant;
        if(!isfinite(kt) || kt <= 0) return reject(DEBUG_ERR_OUT_OF_RANGE, "Torque requires a valid Nm/A torque constant.");
        value /= kt; target = FOC_TARGET_CURRENT_Q;
    }
    if(target <= FOC_TARGET_CURRENT_Q){
        float d = target == FOC_TARGET_CURRENT_D ? value : h->dq_current_setpoint.d;
        float q = target == FOC_TARGET_CURRENT_Q ? value : h->dq_current_setpoint.q;
        float limit = h->flash_data.limits.max_dq_current;
        if(!isfinite(limit) || limit < CONFIG_ABS_CURRENT_MIN || limit > CONFIG_ABS_CURRENT_MAX || !isfinite(d) || !isfinite(q) || hypotf(d, q) > limit)
            return reject(DEBUG_ERR_OUT_OF_RANGE, "Requested d/q current magnitude exceeds the configured limit.");
        h->dq_current_setpoint = (DQCurrentsTypeDef){d, q};
    }else if(target == FOC_TARGET_SPEED){
        if(value < CONFIG_ABS_SPEED_TARGET_MIN || value > CONFIG_ABS_SPEED_TARGET_MAX)
            return reject(DEBUG_ERR_OUT_OF_RANGE, "Speed is outside the configured absolute target range.");
        h->speed_setpoint = value;
    }else{
        if(value < CONFIG_ABS_POSITION_TARGET_MIN || value > CONFIG_ABS_POSITION_TARGET_MAX)
            return reject(DEBUG_ERR_OUT_OF_RANGE, "Position is outside the configured absolute target range.");
        h->angle_setpoint = value;
    }
    return ok();
}

/** @brief Validates a whole PID gain set and invalidates dependent compensation. */
FOC_ConfigResult FOC_SetValidatedPID(FOC_HandleTypeDef *h, PIDControllerTypeDef *pid, PIDValuesTypeDef gains){
    if((unsigned)h->state >= 32 || !(FOC_PID_WRITABLE_STATES & (1u << h->state)) || h->calibration_upload.active)
        return reject(DEBUG_ERR_INVALID_STATE, "PID tuning requires IDLE/RUN and no calibration upload.");
    if(h->active_errors || h->latched_errors) return reject(h->active_errors ? DEBUG_ERR_ACTIVE_FAULTS : DEBUG_ERR_LATCHED_FAULTS, "Clear faults before PID tuning.");
    if(!pid || !pid->K || !pid_gains_valid(gains))
        return reject(DEBUG_ERR_OUT_OF_RANGE, "PID gains must be finite and within the absolute bounds; stability still requires tuning.");
    PID_SetGains(pid, gains);
    h->flash_data.controller.anticogging_data_valid = h->flash_data.controller.anticogging_FF_enabled = 0;
    return ok();
}
