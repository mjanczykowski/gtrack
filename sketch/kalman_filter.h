#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <math.h>

#define LARGE_NUMBER 10000000.0

class KalmanFilter{
	private:
		float Q_angle, Q_gyroBias, R_angle;
		float P[2][2];
		float K[2];
		float y, S, rate, x, bias;
	public:
		KalmanFilter(float Q_angle, float Q_gyroBias, float R_angle, float x);
		~KalmanFilter();
		float iterate(float measurement, float newRate, float dt);
		void setX(float x);
};

#endif
