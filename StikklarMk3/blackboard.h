#ifndef BLACKBOARD_H
#define BLACKBOARD_H

#include <Arduino.h>

#define NUM_DYNAMIXEL_SERVOS 18

class BlackBoard {
public:
	// IR sensors
	unsigned char topIrDistance;
	unsigned char centerIrDistance;
	unsigned char bottomIrDistance;

	// ax voltage
	unsigned char voltage;

	// ax status
	int axSpeed[NUM_DYNAMIXEL_SERVOS];
	int axPosition[NUM_DYNAMIXEL_SERVOS];
	int axLoad[NUM_DYNAMIXEL_SERVOS];
	unsigned char axTemperature[NUM_DYNAMIXEL_SERVOS];
	unsigned char axError[NUM_DYNAMIXEL_SERVOS];

	// wheel command
	int steering;
	int drivingSpeed;
};

extern volatile BlackBoard blackboard;

#endif