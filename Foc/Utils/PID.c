#include "PID.h"
#include "FOC_Utils.h"
#include <math.h>

/**
  * @brief Initializes the PID controller
  * @param pid Pointer to the PID controller structure
  * @param T Sample time in seconds
  * @param tau Derivative low-pass filter time
  * @param max_out Pointer to the maximum output value
  * @param K Pointer to the PID gains structure
  * @param useAngleNormalization Flag to enable angle normalization
  * @note The angle normalization will limit the angle error and difference to the range [-pi, pi].
  */
void PID_Init(PIDControllerTypeDef *pid, float T, float tau, float *max_out, PIDValuesTypeDef *K, uint8_t useAngleNormalization) {

    pid->K = K;
	/* Clear controller variables */
	pid->integrator = 0.0f;
	pid->prevError  = 0.0f;
	pid->differentiator  = 0.0f;
	pid->prevMeasurement = 0.0f;
	pid->out = 0.0f;
    pid->T = T;
    pid->tau = tau;

    pid->max_out = max_out;

    pid->useAngleNormalization = useAngleNormalization;
}

/**
  * @brief Updates the PID controller. This function needs to be called at regular intervals (every T seconds).
  * @param pid Pointer to the PID controller structure
  * @param setpoint Desired setpoint value
  * @param measurement Current measurement value
  * @return The output of the PID controller
  * @note If useAngleNormalization is set, the angle error and difference will be normalized to [-pi, pi].
  */
float PID_Update(PIDControllerTypeDef *pid, float setpoint, float measurement) {

    float error = setpoint - measurement;
    if(pid->useAngleNormalization) {
        normalize_angle2(&error); // Normalize the angle error to [-pi, pi]
    }

    float proportional = pid->K->Kp * error;

    pid->integrator = pid->integrator + 0.5f * pid->K->Ki * pid->T * (error + pid->prevError);

    float max_int_out = *pid->max_out * 0.8f; // Limit the integrator output to 80% of the max output

    if (fabsf(pid->integrator) > max_int_out) { // clamp the integrator
        pid->integrator = (pid->integrator > 0.0f ? 1.0f : -1.0f) * (max_int_out);
    }

    float difference = measurement - pid->prevMeasurement;
    if (pid->useAngleNormalization) {
        normalize_angle2(&difference); // Normalize the angle difference to [-pi, pi]
    }
		
    pid->differentiator = -(2.0f * pid->K->Kd * (difference)	// Note: derivative on measurement, therefore minus sign in front of equation! 
                        + (2.0f * pid->tau - pid->T) * pid->differentiator)
                        / (2.0f * pid->tau + pid->T);


    pid->out = proportional + pid->integrator + pid->differentiator;

    if (fabsf(pid->out) > *pid->max_out) { //clamp the output
        pid->out = (pid->out > 0.0f ? 1.0f : -1.0f) * (*pid->max_out);
    }

    pid->prevError       = error;
    pid->prevMeasurement = measurement;

    return pid->out;
}