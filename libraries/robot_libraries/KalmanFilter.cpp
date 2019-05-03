#include "KalmanFilter.h"

//////////////////////////firstorderfilter////////////////////
// void KalmanFilter::firstOrderFilter(float angle_m, float gyro_m,float dt,float K)
// {
//   angle6 = K * angle_m + (1 - K) * (angle6 + gyro_m * dt);  // roll   focus more on the gyro
// }

////////////////////////kalman/////////////////////////
void KalmanFilter::getAngle(double newAngle, double newRate,float dt,float Q_angle,float Q_gyroBias,float R_measure)
{
  // Q_angle is measured by the accelerometer
  // Q_bias is measured by the gyroscope

  // Discrete Kalman filter time update equations - Time Update ("Predict")
  // Update xhat - Project the state ahead
  /* Step 1 */
  angle += (newRate - bias) * dt;

  // Update estimation error covariance - Project the error covariance ahead
  /* Step 2 */
  P[0][0] += (P[1][1] * dt - P[0][1] - P[1][0] + Q_angle) * dt;
  P[0][1] -= P[1][1] * dt;
  P[1][0] -= P[1][1] * dt;
  P[1][1] += Q_gyroBias * dt;

  // Calculate angle and bias - Update estimate with measurement zk (newAngle)
  /* Step 3 */
  float y = newAngle - angle; // Angle difference

  // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
  // Calculate Kalman gain - Compute the Kalman gain
  /* Step 4 */
  float S = P[0][0] + R_measure; // Estimate error
  /* Step 5 */ 
  float K[2]; // Kalman gain - This is a 2x1 vector
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  /* Step 6 */
  angle += K[0] * y;
  bias += K[1] * y;

  // Calculate estimation error covariance - Update the error covariance
  /* Step 7 */
  float P00_temp = P[0][0];
  float P01_temp = P[0][1];

  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;
}
///////////////////////////// Angle test/////////////////////////////////
void KalmanFilter::angleTest(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz,float dt,float Q_angle,float Q_gyroBias,
									float R_measure,float K)
{
  // angle angle z z
  //Balance parameters
  float Angle = atan2(ay , az) * 57.3; //Angle calculation formula, Angle: first-order complementary filter to calculate the car's final tilt angle
  Gyro_x = (gx - 128.1) / 131.0;         //Angle conversion   Gyro_x = (gx - 128.1) / 131.0; 

  // kalmanFilter(Angle, Gyro_x, dt, Q_angle, Q_gyro,R_angle,C_0);//Kalman filter
  getAngle(Angle, Gyro_x, dt, Q_angle,Q_gyroBias, R_measure);//Kalman filter  get the angle

  //Rotation angle Z axis parameter
  if (gz > 32768) gz -= 65536;         //Forced conversion of 2g 1g
  Gyro_z = -gz / 131.0;                  //Z axis parameter conversion
  // accelz = az / 16.4;       // Acceleration along the X axis = (Accelerometer X axis raw data/16384) g.

  // float angleAy = atan2(ax, az) * 57.3;     //Calculate the angle with x axis
  // Gyro_y = -gy / 131.00;                    //Calculate angular velocity

  // firstOrderFilter(angleAy, Gyro_y, dt, K); //First order filtering  // get angle 6  
  // angleAx is accelerometer

}
