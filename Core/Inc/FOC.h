#ifndef FOC_H
#define FOC_H

#include "main.h"

typedef enum {
    FOC_STATE_INIT,
    FOC_CURRENT_SENSOR_CALIBRATION,
    FOC_CALIBRATION,
    FOC_STATE_ALIGNMENT,
    FOC_STATE_RUN
} FOC_State;

/*Motor parameters for the Maxon motor*/
#define MOTOR_POLE_PAIRS 1
#define MOTOR_STATOR_RESISTANCE 0.535f // ohms
#define MOTOR_STATOR_INDUCTANCE 0.4025e-3f // henries
#define MOTOR_MAGNET_FLUX_LINKAGE 5.47e-3f // webers

#define OUTER_LOOP_FREQUENCY 2000

#define INPUT_VOLTAGE 12.0f
#define VOLTAGE_LIMIT 12.0f
#define PID_LIMIT (VOLTAGE_LIMIT * M_1_SQRT3F)
#define PID_INT_LIMIT (PID_LIMIT * 0.8f)



// Function prototypes
void FOC_Setup();
void FOC_Loop();

#endif // FOC_H