/*
  Adeept Distance module library V1.0
  2015 Copyright (c) Adeept Technology Inc.  All right reserved.
  Author: TOM
*/

#include <Arduino.h>
#include <Distance.h>

// Constructor
Distance::Distance()
{
}
// Begin function to set default pins
void Distance::begin()
{
	begin (2,3);
}
// Begin variables
// - int trigPin: pin used to activate the sensor
// - int echoPin: pin used to read the reflection
void Distance::begin(int echoPin, int trigPin)
{
	_trigPin=trigPin;
	_echoPin=echoPin;
	pinMode(_trigPin, OUTPUT);
	pinMode(_echoPin, INPUT);
	setAveraging(1);		      //1: all samples passed to higher level
}
// setAveraging(int avg): Sets how many samples have to be averaged in getDistanceCentimeter, default value is 100.
void Distance::setAveraging(int avg)
{
	_average=avg;
}
// getDistanceTime(): Returns the time between transmission and echo receive
int Distance::getDistanceTime()
{
	long sum = 0;
	
	for (int i=0;i<_average;i++)
	{
		digitalWrite(_trigPin, LOW);
		delayMicroseconds(2);
		digitalWrite(_trigPin, HIGH);
		delayMicroseconds(10);
		digitalWrite(_trigPin, LOW);
		_duration = pulseIn(_echoPin, HIGH);
		sum=sum+_duration;
	}
	return(int(sum/_average));
}
// getDistanceCentimeter(): Returns the distance in centimeters
int Distance::getDistanceCentimeter()
{
	return (getDistanceTime()/29/2);
}
// getDistanceInch(): Returns the distance in inches
int Distance::getDistanceInch()
{
	return (getDistanceTime()/74/2);
}
// isCloser: check whether the distance to the detected object is smaller than a given threshold
boolean Distance::isCloser(int threshold)
{
	if (threshold>getDistanceCentimeter())
	{
		return (true);
	}
	else
	{
		return (false);
	}
}
// isFarther: check whether the distance to the detected object is larger than a given threshold
boolean Distance::isFarther(int threshold)
{
	if (threshold<getDistanceCentimeter())
	{
		return true;
	}
	else
	{
		return false;
	}
}


