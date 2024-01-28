/*
 * pid_regulator.h
 *
 *  Created on: Jan 28, 2024
 *      Author: huber
 */
#ifndef SRC_PID_REGULATOR_H_
#define SRC_PID_REGULATOR_H_
typedef float float32_t;

typedef struct{
	float32_t Kp;
	float32_t Ki;
	float32_t Kd;
	float32_t dt;
}parametry_PID;


typedef struct{
	parametry_PID param;
	float32_t previous_error, previous_integral;
	float32_t min_output;
	float32_t max_output;
}pid_t2;


float32_t calculate_discrete_pid(pid_t2* pid, float32_t setpoint, float32_t measured);



#endif /* SRC_PID_REGULATOR_H_ */
