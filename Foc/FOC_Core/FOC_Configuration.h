#ifndef FOC_CONFIGURATION_H
#define FOC_CONFIGURATION_H

#include <stdint.h>
#include <stddef.h>
#include "FOC_States.h"
#include "PID.h"

/* Order supplies session-local IDs. Keys, not IDs, identify saved settings. */
#define FOC_CONFIGURATION_GROUPS(X) \
    X(LIMITS, "Limits") X(MOTOR, "Motor") X(CONTROLLER, "Controller") \
    X(ENCODER, "Encoder")
#define CONFIG_GROUP_ENUM(symbol, label) FOC_CONFIG_##symbol,
typedef enum {FOC_CONFIGURATION_GROUPS(CONFIG_GROUP_ENUM) FOC_CONFIG_GROUP_COUNT} FOC_ConfigGroup;
#undef CONFIG_GROUP_ENUM

/* symbol, group, wire type, member, minimum, maximum, unit, writable states, label */
#define FOC_CONFIGURATION_FIELDS(X) \
 X(MAX_CURRENT,LIMITS,f,limits.max_dq_current,CONFIG_ABS_CURRENT_MIN,CONFIG_ABS_CURRENT_MAX,"A",CONFIG_IDLE_ERROR,"Maximum d/q current") \
 X(MAX_VOLTAGE,LIMITS,f,limits.max_dq_voltage,CONFIG_ABS_VOLTAGE_MIN,CONFIG_ABS_VOLTAGE_MAX,"V",CONFIG_IDLE_ERROR,"Maximum d/q voltage") \
 X(OVERVOLTAGE,LIMITS,f,limits.vbus_overvoltage_trip_level,CONFIG_ABS_BUS_VOLTAGE_MIN,CONFIG_ABS_BUS_VOLTAGE_MAX,"V",CONFIG_IDLE_ERROR,"Bus overvoltage trip") \
 X(UNDERVOLTAGE,LIMITS,f,limits.vbus_undervoltage_trip_level,CONFIG_ABS_BUS_VOLTAGE_MIN,CONFIG_ABS_BUS_VOLTAGE_MAX,"V",CONFIG_IDLE_ERROR,"Bus undervoltage trip") \
 X(BUS_CURRENT,LIMITS,f,limits.ibus_overcurrent_trip_level,CONFIG_ABS_CURRENT_MIN,CONFIG_ABS_CURRENT_MAX,"A",CONFIG_IDLE_ERROR,"Bus overcurrent trip") \
 X(MOTOR_TEMP,LIMITS,f,limits.motor_temp_trip_level,CONFIG_ABS_TEMPERATURE_MIN,CONFIG_ABS_MOTOR_TEMPERATURE_MAX,"degC",CONFIG_IDLE_ERROR,"Motor temperature trip") \
 X(MOSFET_TEMP,LIMITS,f,limits.mosfet_temp_trip_level,CONFIG_ABS_TEMPERATURE_MIN,CONFIG_ABS_MOSFET_TEMPERATURE_MAX,"degC",CONFIG_IDLE_ERROR,"MOSFET temperature trip") \
 X(POLE_PAIRS,MOTOR,u32,motor.pole_pairs,CONFIG_ABS_POLE_PAIRS_MIN,CONFIG_ABS_POLE_PAIRS_MAX,"-",CONFIG_IDLE,"Pole pairs") \
 X(DIRECTION,MOTOR,bool,motor.direction,0,1,"-",CONFIG_IDLE,"Reverse direction") \
 X(RESISTANCE,MOTOR,f,motor.phase_resistance,CONFIG_ABS_RESISTANCE_MIN,CONFIG_ABS_RESISTANCE_MAX,"ohm",CONFIG_IDLE,"Phase resistance") \
 X(INDUCTANCE,MOTOR,f,motor.phase_inductance,CONFIG_ABS_INDUCTANCE_MIN,CONFIG_ABS_INDUCTANCE_MAX,"H",CONFIG_IDLE,"Phase inductance") \
 X(TORQUE_CONSTANT,MOTOR,f,motor.torque_constant,CONFIG_ABS_TORQUE_CONSTANT_MIN,CONFIG_ABS_TORQUE_CONSTANT_MAX,"Nm/A",CONFIG_IDLE,"Torque constant") \
 X(BANDWIDTH,CONTROLLER,f,controller.current_control_bandwidth,CONFIG_ABS_BANDWIDTH_MIN,CONFIG_ABS_BANDWIDTH_MAX,"rad/s",CONFIG_IDLE,"Current-control bandwidth") \
 X(CURRENT_FF,CONTROLLER,bool,controller.current_PID_FF_enabled,0,1,"-",CONFIG_IDLE,"Current feed-forward") \
 X(CAL_CURRENT,ENCODER,f,encoder.calibration_current,CONFIG_ABS_CURRENT_MIN,CONFIG_ABS_CURRENT_MAX,"A",CONFIG_IDLE,"Calibration D current")

#define CONFIG_IDLE (1u << FOC_STATE_IDLE)
#define FOC_PID_WRITABLE_STATES (CONFIG_IDLE | (1u << FOC_STATE_RUN))
#define CONFIG_IDLE_ERROR (CONFIG_IDLE | (1u << FOC_STATE_ERROR))
#define CONFIG_FIELD_ENUM(symbol, group, type, member, min, max, unit, states, label) FOC_CONFIG_##symbol,
typedef enum {FOC_CONFIGURATION_FIELDS(CONFIG_FIELD_ENUM) FOC_CONFIG_FIELD_COUNT} FOC_ConfigField;
#undef CONFIG_FIELD_ENUM

typedef struct {uint8_t id; uint32_t raw;} FOC_ConfigUpdate;
typedef struct {uint16_t code; const char *message;} FOC_ConfigResult;
typedef enum {FOC_TARGET_CURRENT_D, FOC_TARGET_CURRENT_Q, FOC_TARGET_SPEED,
              FOC_TARGET_POSITION, FOC_TARGET_TORQUE} FOC_Target;

int FOC_PIDFormat(char *out, size_t capacity, uint8_t id, const char *name);
FOC_ConfigResult FOC_ConfigurationReady(const FOC_HandleTypeDef *hfoc, FOC_StateTypeDef operation);
const char *FOC_ConfigurationName(uint8_t id);
int FOC_ConfigurationFormat(char *out, size_t capacity, uint8_t id);
uint8_t FOC_ConfigurationGroup(uint8_t id);
uint32_t FOC_ConfigurationRead(const FOC_HandleTypeDef *hfoc, uint8_t id);
FOC_ConfigResult FOC_ConfigurationSet(FOC_HandleTypeDef *hfoc, uint8_t group,
                                    const FOC_ConfigUpdate *updates, uint8_t count);
FOC_ConfigResult FOC_SetTarget(FOC_HandleTypeDef *hfoc, uint8_t target, float value);
FOC_ConfigResult FOC_SetValidatedMode(FOC_HandleTypeDef *hfoc, uint8_t mode);
FOC_ConfigResult FOC_SetValidatedPID(FOC_HandleTypeDef *hfoc, PIDControllerTypeDef *pid, PIDValuesTypeDef gains);
#endif
