#ifndef FOC_CONFIG_H
#define FOC_CONFIG_H

#include "main.h"


/*Anticogging*/
#define NUMBER_OF_ANTICOG_MEASUREMENTS 1000 //The amount of steps in the full rotation
#define ANTICOG_ANGLE_STEP (2.0f * M_PI / NUMBER_OF_ANTICOG_MEASUREMENTS)


/*PCB parameters*/
#define MAXIMUM_BUS_VOLTAGE 30.0f // [V]
#define MAXIMUM_PHASE_CURRENT 30.0f // [A], 30A when no cooling on the mosfets, this can be increased with proper cooling
#define CURRENT_SENSE_RESISTANCE 0.02f // [Ohm], current sense resistor value to measure the phase currents
#define VBUS_VOLTAGE_DIVIDER_RATIO (1.0f/(1.0f + 10.0f)) // R2/(R1+R2) voltage divider ratio for the bus voltage measurement, R1 = 100k, R2 = 10k

/*ADC measurement filters*/
#define ADC_LOOP_ALPHA (2.0f/(CURRENT_LOOP_CLOCK_DIVIDER+1))
#define TEMP_LOOP_ALPHA (2.0f/(1000))





/*Motor parameters*/
#define MOTOR_POLE_PAIRS 11 //Motor pole pairs
#define MOTOR_STATOR_RESISTANCE 0.2f // [Ohm]
#define MOTOR_STATOR_INDUCTANCE 2.5e-05f // [H]
#define MOTOR_TORQUE_CONSTANT (8.27 / 380) // [Nm/A]

/*Voltages*/
#define INPUT_VOLTAGE 24.0f //[V]
#define VOLTAGE_LIMIT 5.0f //[V], max bus voltage


/*PID limits*/
#define CURRENT_PID_LIMIT (VOLTAGE_LIMIT * M_1_SQRT3F)//[V]
#define CURRENT_PID_INT_LIMIT (CURRENT_PID_LIMIT * 0.8f)//[V]

#define SPEED_PID_LIMIT 15.0f //[A]
#define SPEED_PID_INT_LIMIT (SPEED_PID_LIMIT * 0.8f) //[A]



#endif // FOC_CONFIG_H