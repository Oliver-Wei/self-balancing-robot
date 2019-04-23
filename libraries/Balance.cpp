/*
  Adeept Balance 2WD library V1.0
  2015 Copyright (c) Adeept Technology Inc.  All right reserved.
  Author: TOM
*/

#include "Balance.h"

double Balance::speed_PI_control(double kps,double kis,double kds,int f,int b,double p0)
{
  float speeds = (pulseleft + pulseright) * 1.0;         //Speed pulse value
  pulseright = pulseleft = 0;
  speeds_filterold *= 0.7;                               //First order complementary filtering
  float speeds_filter = speeds_filterold + speeds * 0.3;
  speeds_filterold = speeds_filter;
  positions += speeds_filter;
  positions += f;    //Full control volume integration
  positions += b;    //Full control volume integration
  positions = constrain(positions, -4550,4550);          //Anti-integral saturation
  double output = kis * (p0 - positions) + kps * (p0 - speeds_filter); //Speed loop control // PI control
  if(flag1==1)
  {
  positions=0;
  }
  
  return output;
}

float Balance::direction_PD_control(int turnleftflag,int turnrightflag,double kpturn,double kdturn,float Gyroz)
{
  int spinonce = 0;
  float turnspeed = 0;
	float rotationratio = 0;
  float turnout_put = 0;
	
  if (turnleftflag == 1 || turnrightflag == 1)
  {
    if (spinonce == 0)       //Before the rotation to determine the current speed, enhance the car adaptability.
    {
      turnspeed = ( pulseright + pulseleft);       //The current speed of the car (Pulse representation)
	    spinonce++;
    }

    if (turnspeed < 0)       //The current speed of the car
    {
      turnspeed = -turnspeed;
    }
    if(turnleftflag==1||turnrightflag==1)
    {
     turnmax=5;
     turnmin=-5;
    }
    rotationratio = 55 / turnspeed;       //According to the car speed set value
    if (rotationratio < 0.5)rotationratio = 0.5;
    if (rotationratio > 5)rotationratio = 5;
  }
  else
  {
    rotationratio = 0.5;
    spinonce = 0;
    turnspeed = 0;
  }
  if (turnleftflag == 1)       //According to the direction parameters superimposed
  {
    turnout += rotationratio;
  }
  else if (turnrightflag == 1)//According to the direction parameters superimposed
  {
    turnout -= rotationratio;
  }
  else turnout = 0;
  if (turnout > turnmax) turnout = turnmax;        //Amplitude maximum setting
  if (turnout < turnmin) turnout = turnmin;        //Amplitude minimum setting

  turnout_put = -turnout * kpturn - Gyroz * kdturn; //Rotation PD algorithm control, fusion speed and Z axis rotation positioning.
	return turnout_put;
}

void Balance::pwma(double speedoutput,float rotationoutput,float angle,float angle6,int turnleftflag,int turnrightflag,
	int f,int b,float accelz,int Pin1,int Pin2,int Pin3,int Pin4,int PinPWMA,int PinPWMB)
{
  // angleoutput balance
  // speedoutput forward/backward
  // rotationoutput turn left/right
  pwm1 = -angleoutput - speedoutput - rotationoutput; //Left motor PWM output value
  pwm2 = -angleoutput - speedoutput + rotationoutput; //Right motor PWM output value

  //Amplitude limit
  if (pwm1 > 255) pwm1 = 255;
  if (pwm1 < -255) pwm1 = -255;
  if (pwm2 > 255) pwm2 = 255;
  if (pwm2 < -255) pwm2 = -255;
  //The angle is too large to stop the motor.
  if (angle > 40 || angle < -40){
    pwm1 = 0;
    pwm2 = 0;
  }

   if (angle6 > 3 || angle6 < -3 &&turnleftflag == 0 && turnrightflag == 0 && f == 0 && b == 0){
    if(stopl + stopr > 1500||stopl + stopr <- 1500){
    pwm1 = 0;
    pwm2 = 0;
	  flag1=1;
	}
  }else {
     stopl=stopr=0;
     flag1=0;
  }

   //Positive and negative output judgment of motor (left motor judgment)
  if (pwm1 >= 0) {
    // Move forward
    digitalWrite(Pin2, 0);
    digitalWrite(Pin1, 1);
    analogWrite(PinPWMA, pwm1);
  } else {
    // Move backward
      digitalWrite(Pin2, 1);
      digitalWrite(Pin1, 0);
      analogWrite(PinPWMA, -pwm1); //--=+
  }
   //Positive and negative output judgment of motor (right motor judgment)
  if (pwm2 >= 0) {
    // Move forward
    digitalWrite(Pin4, 0);
    digitalWrite(Pin3, 1);
    analogWrite(PinPWMB, pwm2);
  } else {
    // Move backward
      digitalWrite(Pin4, 1);
      digitalWrite(Pin3, 0);
      analogWrite(PinPWMB, -pwm2);  //--=+
  }
}
