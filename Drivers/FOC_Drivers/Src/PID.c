#include "PID.h"
#include <stdint.h>
#include "FOC_Utils.h"


void PID_Init(PIDControllerTypeDef *pid, float T, float tau, float limMin, float limMax, float limMinInt, float limMaxInt, PIDValuesTypeDef *K, uint8_t useAngleNormalization) {

    pid->K = K;
	/* Clear controller variables */
	pid->integrator = 0.0f;
	pid->prevError  = 0.0f;

	pid->differentiator  = 0.0f;
	pid->prevMeasurement = 0.0f;

	pid->out = 0.0f;

    pid->T = T;
    pid->tau = tau;

    pid->limMin = limMin;
    pid->limMax = limMax;
    pid->limMinInt = limMinInt;
    pid->limMaxInt = limMaxInt;

    pid->useAngleNormalization = useAngleNormalization;
}

float PID_Update(PIDControllerTypeDef *pid, float setpoint, float measurement) {

    float error = setpoint - measurement;

    if(pid->useAngleNormalization) {
        normalize_angle2(&error); // Normalize the angle error to [-pi, pi]
    }

    float proportional = pid->K->Kp * error;

    pid->integrator = pid->integrator + 0.5f * pid->K->Ki * pid->T * (error + pid->prevError);

	/* Anti-wind-up via integrator clamping */
    if (pid->integrator > pid->limMaxInt) {

        pid->integrator = pid->limMaxInt;

    } else if (pid->integrator < pid->limMinInt) {

        pid->integrator = pid->limMinInt;

    }
    float difference = measurement - pid->prevMeasurement;
    if (pid->useAngleNormalization) {
        normalize_angle2(&difference); // Normalize the angle difference to [-pi, pi]
    }
		
    pid->differentiator = -(2.0f * pid->K->Kd * (difference)	/* Note: derivative on measurement, therefore minus sign in front of equation! */
                        + (2.0f * pid->tau - pid->T) * pid->differentiator)
                        / (2.0f * pid->tau + pid->T);


    pid->out = proportional + pid->integrator + pid->differentiator;

    if (pid->out > pid->limMax) {

        pid->out = pid->limMax;

    } else if (pid->out < pid->limMin) {

        pid->out = pid->limMin;

    }

	/* Store error and measurement for later use */
    pid->prevError       = error;
    pid->prevMeasurement = measurement;

	/* Return controller output */
    return pid->out;

}