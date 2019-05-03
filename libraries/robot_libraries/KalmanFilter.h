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
	// void firstOrderFilter(float angle_m, float gyro_m,float dt,float K);
	void getAngle(double newAngle, double newRate,float dt,float Q_angle,float Q_gyroBias,float R_measure);
	void angleTest(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,float dt,float Q_angle,float Q_gyroBias,
									float R_measure,float K);
	float Gyro_x,Gyro_y,Gyro_z;
	// float accelz = 0;
	float angle = 0;
	// float angle6 = 0;
private:
	float bias;
	float P[2][2] = {{ 1, 0 }, { 0, 1 }};                           
};
#endif

//
// END OF FILE
//