/*
 * kalman_filter.h
 *
 *  Created on: Mar 11, 2020
 *      Author: linux
 */

#ifndef INC_KALMAN_FILTER_H_
#define INC_KALMAN_FILTER_H_

typedef struct{
	double q; // measurement noise
	double r; // environment noise
	double x0; // predicted state
	double p0; // predicted covariance
	double state, covariance;
}KalmanFilter;

void kalman_filter_init(KalmanFilter *f, double q, double r, double state, double covariance);
double kalman_filter_calc(KalmanFilter *f, double val);

#endif /* INC_KALMAN_FILTER_H_ */
