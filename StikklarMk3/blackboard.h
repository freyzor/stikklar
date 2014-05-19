#ifndef BLACKBOARD_H
#define BLACKBOARD_H

#include <Arduino.h>
#include "config.h"

class BlackBoard {
public:
	// IR sensor intensity
	unsigned char topIrIntensity;
	unsigned char centerIrIntensity;
	unsigned char bottomIrIntensity;

	// ax voltage
	unsigned char voltage;

	// ax status
	int axSpeed[AX_SERVO_COUNT];
	int axPosition[AX_SERVO_COUNT];
	int axLoad[AX_SERVO_COUNT];
	unsigned char axTemperature[AX_SERVO_COUNT];
	unsigned char axError[AX_SERVO_COUNT];

	// wheel command
	int steering;
	int drivingSpeed;

	// IMU
	float yaw;
	float pitch;
	float roll;
};

extern volatile BlackBoard blackboard;

#endif