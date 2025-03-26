#ifndef PID_H
#define PID_H

typedef struct {

	/* Controller gains */
	float Kp;
	float Ki;
	float Kd;

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

} PIDController;

void PID_SetK(PIDController *pid, float Kp, float Ki, float Kd);
void PID_Init(PIDController *pid, float T, float tau, float limMin, float limMax, float limMinInt, float limMaxInt);
float PID_Update(PIDController *pid, float setpoint, float measurement);


#endif // PID_H