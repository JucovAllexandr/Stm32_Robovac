/*
 * kalman_filter.c
 *
 *  Created on: Mar 11, 2020
 *      Author: linux
 */

#include "kalman_filter.h"

void kalman_filter_init(KalmanFilter *f, double q, double r, double state, double covariance)
{
	f->q = q;
	f->r = r;
	f->state = state;
	f->covariance = covariance;
	f->x0 = 0;
	f->p0 =0;
}

double kalman_filter_calc(KalmanFilter *f, double val)
{
	f->x0 = f->state;
	f->p0 = f->covariance + f->q;

	double k = f->p0 / (f->p0 + f->r);
	f->state = f->x0 + k * (val - f->x0);
	f->covariance = (1 - k) * f->p0;
	return f->state;
}
