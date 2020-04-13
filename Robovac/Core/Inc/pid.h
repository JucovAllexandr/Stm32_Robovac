/*
 * pid.h
 *
 *  Created on: Mar 9, 2020
 *      Author: linux
 */

#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct{
	double kp, ki, kd;
	double integral;
	double derivate;
	double last_error;
	double dt;
	//float cv, pv, sp;
	double cv_min, cv_max;
}PID;

PID* pid_init(double kp, double ki, double kd, double cv_min, double cv_max);
double pid_calc(PID *pid, double process_variable, double set_point);
#endif /* INC_PID_H_ */
