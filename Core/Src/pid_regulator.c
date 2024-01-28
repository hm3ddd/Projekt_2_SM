/*
 * pid_regulator.c
 *
 *  Created on: Jan 28, 2024
 *      Author: huber
 */
#include "pid_regulator.h"

float32_t calculate_discrete_pid(pid_t2* pid, float32_t setpoint, float32_t measured){
	float32_t u=0, P, I, D, error, integral, derivative;

	error = setpoint-measured;

	//Moduł proporcjonalny
	P = pid->param.Kp * error;

	//Moduł całki
	integral = pid->previous_integral + (error+pid->previous_error) ; //całka numeryczna bez anti-windup
	pid->previous_integral = integral;
	I = pid->param.Ki*integral*(pid->param.dt/2.0);

	//Różniczka
	derivative = (error - pid->previous_error)/pid->param.dt; //różniczka numeryczna bez without filter
	pid->previous_error = error;
	D = pid->param.Kd*derivative;

	//sum of all parts
	u = P  + I + D; //without saturation

	if (u > pid->max_output) {
	    u = pid->max_output;
	} else if (u < pid->min_output) {
	    u = pid->min_output;
	}




	return u;
}

