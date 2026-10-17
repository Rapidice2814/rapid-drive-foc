#ifndef FOC_CONFIG_H
#define FOC_CONFIG_H

#include "main.h"

#define FOC_VERSION_MAJOR 0
#define FOC_VERSION_MINOR 1
#define FOC_VERSION_PATCH 1

/* Debugging */
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

/*Current loop*/
#define CURRENT_LOOP_OUTPUT_DELAY 1.15f // [current loop cycles], time from sampling the angle to the voltage being applied (PWM update), the output angle is advanced by this. Measured on the test motor: 1.0-1.25

/*Filters*/
#define SPEED_MEASUREMENT_ALPHA 0.3f


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





/*Motor parameters*/
#define MOTOR_POLE_PAIRS 11 //Motor pole pairs
#define MOTOR_STATOR_RESISTANCE 0.2f // [Ohm]
#define MOTOR_STATOR_INDUCTANCE 2.5e-05f // [H]
#define MOTOR_TORQUE_CONSTANT (8.27 / 380) // [Nm/A]

/*limits*/
#define VOLTAGE_LIMIT 8.0f //[V], max motor voltage
#define MAX_DQ_CURRENT 5.0f //[A]


/* HFI Parameters */
#define HFI_INJECTION_OMEGA (1000.0f * M_2PIF) // [rad/s], must stay well below CURRENT_LOOP_FREQUENCY / 2
#define HFI_INJECTION_AMPLITUDE 0.2f // [V]

#define HFI_BANDPASS_Q 2.0f // [-], quality factor of the band-pass filter that separates the carrier current from the fundamental current
#define HFI_CURRENT_LPF_CUTOFF_FREQ 200.0f // [Hz], cutoff frequency of the low-pass filter for the demodulated HFI signals

#define HFI_PLL_KP 1000.0f // [rad/s per unit error], the error is roughly (Lq-Ld)/Lq * angle error, so these gains assume ~10% saliency
#define HFI_PLL_KI 25000.0f
#define HFI_MAX_ELECTRICAL_SPEED (200.0f * M_2PIF) // [rad/s], HFI is a low speed method, the PLL output is clamped to this value

#define HFI_POLARITY_CURRENT 2.0f // [A], average current of the polarity voltage pulses (the voltage follows from R and L). 0 disables the polarity detection
#define HFI_BIAS_CURRENT 1.5f // [A], DC d-axis current while HFI runs. Keeps the phase currents away from zero, where dead time distorts the carrier
#define HFI_CONVERGE_TIME 0.2f // [s], time the PLL gets to lock before the polarity detection starts
#define HFI_POLARITY_PULSE_PAIRS 8 // [-], number of +/- voltage pulse pairs, averaged for the polarity decision
#define HFI_POLARITY_PULSE_SPACING 8 // [cycles], current loop cycles from one pulse to the next, lets the current decay
#define HFI_POLARITY_DEFAULT_VOLTAGE 1.0f // [V], pulse voltage if the motor R and L are not identified
#define HFI_SETTLE_TIME 0.05f // [s], time after the polarity detection with the alternating bias, so the PLL catches up before the steady bias pulls the rotor
#define HFI_BIAS_ALTERNATE_TIME 0.01f // [s], while the polarity is unknown the bias flips sign at this interval, so it can't pull the rotor in one direction

#define HFI_MIN_CARRIER_POWER 1e-6f // [A^2], below this the carrier current is considered absent and the PLL is not updated


#endif // FOC_CONFIG_H