#ifndef FOC_CONFIG_H
#define FOC_CONFIG_H


/*Anticogging*/
#define NUMBER_OF_ANTICOG_MEASUREMENTS 1000 //The amount of steps in the full rotation
#define ANTICOG_ANGLE_STEP (2.0f * M_PI / NUMBER_OF_ANTICOG_MEASUREMENTS)

/*Moror parameters*/
#define MOTOR_POLE_PAIRS 11 //Motor pole pairs
#define MOTOR_STATOR_RESISTANCE 0.2f // [Ohm]
#define MOTOR_STATOR_INDUCTANCE 2.5e-05f // [H]
#define MOTOR_TORQUE_CONSTANT (8.23 / 380) // [Nm/A]

/*Voltages*/
#define INPUT_VOLTAGE 24.0f //[V]
#define VOLTAGE_LIMIT 4.0f //[V], max voltage output by the inverter, limits max speed


/*PID limits*/
#define CURRENT_PID_LIMIT (VOLTAGE_LIMIT * M_1_SQRT3F)//[V]
#define CURRENT_PID_INT_LIMIT (CURRENT_PID_LIMIT * 0.8f)//[V]

#define SPEED_PID_LIMIT 15.0f //[A]
#define SPEED_PID_INT_LIMIT (SPEED_PID_LIMIT * 0.8f) //[A]



#endif // FOC_CONFIG_H