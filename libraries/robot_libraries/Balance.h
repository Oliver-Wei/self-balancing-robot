#ifndef BALANCE_H_
#define BALANCE_H_

#if defined(ARDUINO) && (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif


class Balance
{
public:
  double speed_PI_control(double kps,double kis,double kds,int f,int b,double p0);
  float direction_PD_control(int turnleftflag,int turnrightflag,double kpturn,double kdturn,float Gyroz);
  void angleout();
  void pwma(double speedoutput,float rotationoutput,float angle,int turnleftflag,int turnrightflag,
			int f,int b, int Pin1,int Pin2,int Pin3,int Pin4,int PinPWMA,int PinPWMB);
	int pulseright = 0;  //
	int pulseleft = 0;   //
	// int posture=0;
	double angleoutput=0,pwm1 = 0, pwm2 = 0;
private:
	float speeds_filterold;//Speed filtering
	float positions;       
	int turnmax = 0;       //Rotate the output amplitude
	int turnmin = 0;       //Rotate the output amplitude
	float turnout = 0;
	int flag1 = 0;
};
#endif
//
// END OF FILE
//