#ifndef TURRET_ENGINE_H
#define TURRET_ENGINE_H

#include <BioloidController.h>

#define MIN_PAN 0
#define MAX_PAN 1023

#define MIN_TILT 204
#define MAX_TILT 819

#define PAN_CENTER	512
#define TILT_CENTER	512

#define TURRET_TRANSITION_TIME	98

class TurretEngine {
public:
	TurretEngine();
	void setupContoller();
	void setBioloidController(BioloidController* bioloidController);
	void update();
	void updateServos();
	int pan;
	int tilt;
private:
	BioloidController* controller;
};

#endif