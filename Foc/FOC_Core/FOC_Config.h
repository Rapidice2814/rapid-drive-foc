#ifndef FOC_CONFIG_H
#define FOC_CONFIG_H

#include "main.h"

#define FOC_VERSION_MAJOR 0
#define FOC_VERSION_MINOR 3
#define FOC_VERSION_PATCH 0

/* Debugging */
/* Raw debug writes and text commands are development-only, never a safety API. */
#ifndef FOC_ENABLE_DEBUG_WRITES
#ifdef DEBUG
#define FOC_ENABLE_DEBUG_WRITES 1
#else
#define FOC_ENABLE_DEBUG_WRITES 0
#endif
#endif
#define DEBUG_SOF1_BIN 0xAA
#define DEBUG_SOF2_BIN 0x55

#define MAX_LOGDATA_SAMPLE_COUNT 30
#define MAX_LOGDATA_SIGNAL_COUNT 8

/*Anticogging*/
#define NUMBER_OF_ANTICOG_MEASUREMENTS 1000 //The amount of steps in the full rotation
#define ANTICOG_ANGLE_STEP (2.0f * M_PI / NUMBER_OF_ANTICOG_MEASUREMENTS)


/*PCB parameters*/
#define MAXIMUM_BUS_VOLTAGE 30.0f // [V]
#define MINIMUM_BUS_VOLTAGE 8.0f // [V]
#define MAXIMUM_PHASE_CURRENT 30.0f // [A], 30A when no cooling on the mosfets, this can be increased with proper cooling
#define CURRENT_SENSE_RESISTANCE 0.01f // [Ohm], current sense resistor value to measure the phase currents
#define VBUS_VOLTAGE_DIVIDER_RATIO (1.0f/(1.0f + 10.0f)) // R2/(R1+R2) voltage divider ratio for the bus voltage measurement, R1 = 100k, R2 = 10k

/*Filters*/
/* Mechanical speed PLL natural frequency. Lower = quieter, slower acceleration tracking. */
#define ENCODER_SPEED_PLL_FREQUENCY_HZ 50.0f

/* Anti-cogging acceptance windows (mechanical radians, seconds and amperes).
   Calibration uses the separate user-configured pid_anticogging gains. */
#define ANTICOG_WINDOW_MS 100U
#define ANTICOG_MIN_SAMPLES 32U
#define ANTICOG_MOVE_SETTLE_MS 50U
#define ANTICOG_POSITION_TOLERANCE 0.005f
#define ANTICOG_POSITION_SPAN 0.004f
#define ANTICOG_MEAN_ABS_SPEED 0.05f
#define ANTICOG_CURRENT_STD_ABS 0.02f
#define ANTICOG_CURRENT_STD_REL 0.05f


/*Temperature*/
#define MOTOR_MAX_TEMP 60.0f // [Celsius], maximum motor temperature
#define MOSFET_MAX_TEMP 80.0f // [Celsius], maximum mosfet temperature


/*NTC Parameters*/
#define NTC_TEMP_MIN -10       // Minimum temperature in °C
#define NTC_TEMP_MAX 100      // Maximum temperature in °C
#define NTC_T0 298.15f        // Reference temperature in Kelvin (25°C)
#define NTC_B 3950.0f         // Beta constant
#define NTC_R0 10e3f      // Reference resistance at T0 (Ohms)
#define NTC_SERIES_RESISTOR 10e3f //10k resistor in series with the NTC for the voltage divider





/* Motor-specific settings and operating limits start unset (zero). */

/* Absolute production-API bounds. These are board/firmware capabilities, not
 * editable runtime limits. Widening them requires hardware review/rebuild.
 * Runtime settings must fit these ranges and satisfy cross-field validators. */
#define CONFIG_ABS_CURRENT_MIN 0.001f                 // A
#define CONFIG_ABS_CURRENT_MAX MAXIMUM_PHASE_CURRENT  // A
#define CONFIG_ABS_VOLTAGE_MIN 0.001f                 // V
#define CONFIG_ABS_VOLTAGE_MAX (MAXIMUM_BUS_VOLTAGE / 1.732050808f) // V, modulation envelope
#define CONFIG_ABS_BUS_VOLTAGE_MIN MINIMUM_BUS_VOLTAGE // V
#define CONFIG_ABS_BUS_VOLTAGE_MAX MAXIMUM_BUS_VOLTAGE // V
#define CONFIG_ABS_TEMPERATURE_MIN 0.0f               // degC
#define CONFIG_ABS_MOTOR_TEMPERATURE_MAX MOTOR_MAX_TEMP // degC
#define CONFIG_ABS_MOSFET_TEMPERATURE_MAX MOSFET_MAX_TEMP // degC
#define CONFIG_ABS_POLE_PAIRS_MIN 1u
#define CONFIG_ABS_POLE_PAIRS_MAX 255u                 // uint8_t storage
#define CONFIG_ABS_RESISTANCE_MIN 0.000001f            // ohm
#define CONFIG_ABS_RESISTANCE_MAX 1000.0f              // ohm, supported numerical range
#define CONFIG_ABS_INDUCTANCE_MIN 0.000000001f         // H
#define CONFIG_ABS_INDUCTANCE_MAX 10.0f                // H, supported numerical range
#define CONFIG_ABS_TORQUE_CONSTANT_MIN 0.000001f       // Nm/A
#define CONFIG_ABS_TORQUE_CONSTANT_MAX 1000.0f         // Nm/A
#define CONFIG_ABS_BANDWIDTH_MIN 0.001f                // rad/s
#define CONFIG_ABS_BANDWIDTH_MAX 5000.0f               // rad/s
#define CONFIG_ABS_PID_GAIN_MIN 0.0f                  // units depend on controller/gain
#define CONFIG_ABS_PID_GAIN_MAX 1000000.0f             // numerical bound, not a stability guarantee
#define CONFIG_ABS_SPEED_TARGET_MIN (-10000.0f)       // rad/s
#define CONFIG_ABS_SPEED_TARGET_MAX 10000.0f           // rad/s
#define CONFIG_ABS_POSITION_TARGET_MIN (-1000000.0f)  // rad, unwrapped
#define CONFIG_ABS_POSITION_TARGET_MAX 1000000.0f      // rad, unwrapped

#endif // FOC_CONFIG_H
