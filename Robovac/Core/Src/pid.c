/*
 * pid.c
 *
 *  Created on: Mar 9, 2020
 *      Author: linux
 */

#include "pid.h"
#include "config.h"
#include <stdlib.h>

PID* pid_init(double kp, double ki, double kd, double cv_min, double cv_max)
{
	PID *pid = (PID*) malloc(sizeof(PID));

	pid->integral = 0;
	pid->derivate = 0;
	pid->last_error = 0;

	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->cv_min = cv_min;
	pid->cv_max = cv_max;

	pid->dt = 0.6;

	return pid;
}

double pid_calc(PID *pid, double process_variable, double set_point)
{
	double error = set_point - process_variable;
	double out;

	pid->integral += pid->ki * error * pid->dt;

	out = pid->kp * error; //proportional
	out += ((error - pid->last_error) / pid->dt) * pid->kd; //derivative
	out += pid->integral;
	/*if(pid->integral > PID_INTEGRAL_MAX) {pid->integral = PID_INTEGRAL_MAX;}
	if(pid->integral < PID_INTEGRAL_MIN) {pid->integral = PID_INTEGRAL_MIN;}
	if(pid->derivate > PID_DERIVATE_MAX) {pid->derivate = PID_DERIVATE_MAX;}
	if(pid->derivate < PID_DERIVATE_MIN) {pid->derivate = PID_DERIVATE_MIN;}*/

	pid->last_error = error;
	//out =  + pid->integral + pid->derivate;

	if(out > pid->cv_max)
	{
		out = pid->cv_max;
	}

	if(out < pid->cv_min)
	{
		out = pid->cv_min;
	}


	return out;
}
