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

	/* Derivative low-pass filter time constant */
	float tau;

	/* Output limits */
	float limMin;
	float limMax;
	
	/* Integrator limits */
	float limMinInt;
	float limMaxInt;

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
	uint8_t useAngleNormalization;

} PIDControllerTypeDef;


void PID_Init(PIDControllerTypeDef *pid, float T, float tau, float limMin, float limMax, float limMinInt, float limMaxInt, PIDValuesTypeDef *K, uint8_t useAngleNormalization);
float PID_Update(PIDControllerTypeDef *pid, float setpoint, float measurement);


#endif // PID_H