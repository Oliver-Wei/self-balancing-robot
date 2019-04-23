/*
  Adeept Kalman Filter library V1.0
  2015 Copyright (c) Adeept Technology Inc.  All right reserved.
  Author: TOM
*/

#ifndef KALMANFILTER_H_
#define KALMANFILTER_H_

#if defined(ARDUINO) && (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif


class KalmanFilter
{
public:
	void firstOrderFilter(float angle_m, float gyro_m,float dt,float K1);
	void kalmanFilter(double angle_m, double gyro_m,float dt,float Q_angle,float Q_gyro,float R_angle,float C_0);
	void angleTest(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,float dt,float Q_angle,float Q_gyro,
									float R_angle,float C_0,float K1);
  float Gyro_x,Gyro_y,Gyro_z;
  float accelz = 0;
  float angle;
  float angle6;
private:
	float angle_err,q_bias;
	float Pdot[4] = { 0, 0, 0, 0};
	float P[2][2] = {{ 1, 0 }, { 0, 1 }};
	float PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;                              
	
};
#endif

//
// END OF FILE
//