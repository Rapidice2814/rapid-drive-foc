#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
	/* Controller gains */
	float Kp;
	float Ki;
	float Kd;
}PIDValuesTypeDef;

typedef struct {

	PIDValuesTypeDef *K;

	/* Output limits */
	float *max_out;

	/* Derivative low-pass filter time constant */
	float tau;


	/* Sample time (in seconds) */
	float T;

	/* Controller "memory" */
	float integrator;
	float prevError;			/* Required for integrator */
	float differentiator;
	float prevMeasurement;		/* Required for differentiator */

	/* Controller output */
	float out;

	/* Angle normalization flag */
	uint8_t useAngleNormalization; //when set to 1, the controller will normalize the angle error and difference to [-pi, pi]

} PIDControllerTypeDef;


void PID_Init(PIDControllerTypeDef *pid, float T, float tau, float *max_out, PIDValuesTypeDef *K, uint8_t useAngleNormalization);
float PID_Update(PIDControllerTypeDef *pid, float setpoint, float measurement);


#endif // PID_H