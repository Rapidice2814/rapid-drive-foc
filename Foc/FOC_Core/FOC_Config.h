#ifndef FOC_CONFIG_H
#define FOC_CONFIG_H

#include "main.h"


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
#define SPEED_MEASUREMENT_ALPHA 0.3f


/*Temperature*/
#define MOTOR_MAX_TEMP 60.0f // [Celsius], maximum motor temperature
#define MOSFET_MAX_TEMP 60.0f // [Celsius], maximum mosfet temperature


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
#define MAX_DQ_CURRENT 17.2f //[A]



#endif // FOC_CONFIG_H