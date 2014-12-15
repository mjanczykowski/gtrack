#include "kalman_filter.h"

KalmanFilter::KalmanFilter(float Q_angle, float Q_gyroBias, float R_angle, float x){
	this -> Q_angle = Q_angle;
	this -> Q_gyroBias = Q_gyroBias;
	this -> R_angle = R_angle;
	this -> x = x;
	P[1][1] = P[0][0] = LARGE_NUMBER;
	P[0][1] = P[1][0] = 0.0;
	bias = 0.0;
	K[0] = K[1] = 0.0;
}

KalmanFilter::~KalmanFilter(){}

float KalmanFilter::iterate(float measurement, float newRate, float dt){
	rate = newRate - bias;
	x += dt * rate;

	P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
	P[0][1] -= dt * P[1][1];
	P[1][0] -= dt * P[1][1];
	P[1][1] += Q_gyroBias * dt;

	S = P[0][0] + R_angle;

	K[0] = P[0][0] / S;
	K[1] = P[1][0] / S;

	y = measurement - x;

	x += K[0] * y;
	bias += K[1] * y;

	float P00_temp = P[0][0];
	float P01_temp = P[0][1];
	P[0][0] -= K[0] * P00_temp;
	P[0][1] -= K[0] * P01_temp;
	P[1][0] -= K[1] * P00_temp;
	P[1][1] -= K[1] * P01_temp;

	return x;
}

void KalmanFilter::setX(float x){
	this -> x = x;
}
