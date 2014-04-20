#ifndef _GAIT_ENGINE_H
#define _GAIT_ENGINE_H

#include <ax12.h>
#include <BioloidController.h>
#include <Arduino.h>
#include "config.h"
#include "inverse_kinematic.h"

#define GaitSetup (*this.*gaitSetup)
#define GaitGen (*this.*gaitGen)

#define maxValue(n) maxs[n-1]
#define minValue(n) mins[n-1]
#define neutral(n) neutrals[n-1]

#define DEFAULT_ENDPOINT_X 70  // front/back
#define DEFAULT_ENDPOINT_Y 70  // right/left
#define DEFAULT_ENDPOINT_Z 160 // down/up
 
// ripple gate move one leg at the time
#define RIPPLE                  0
#define RIPPLE_SMOOTH           1
// amble gaits move two alternate legs at a time
#define AMBLE                   2
#define AMBLE_SMOOTH            3
// special ripple gait to switch endpoins
#define RIPPLE_STEP_TO 			5

// Standard Transition time should be of the form (k*BIOLOID_FRAME_LENGTH)-1
// for maximal accuracy. BIOLOID_FRAME_LENGTH = 33ms, so good options include:
// 32, 65, 98, etc...
#define STD_TRANSITION          98

class GaitEngine {
public:
	GaitEngine();
    void update();
    void readPose();
    void slowStart(long msec);
    void doPose(const unsigned int * addr, long msec);
    void setupIK();
	void gaitSelect(int GaitType);
	void setStepToTarget(int x, int y, int z, long msec);
	bool isSteppingTo();

    //Parameters for manipulating body position 
	float bodyRotX;    // body roll
	float bodyRotY;    // body pitch
	float bodyRotZ;    // body rotation
	int bodyPosX;
	int bodyPosY;

	// Parameters for gait manipulation */
	int Xspeed;
	int Yspeed;
	float Rspeed;
	ik_req_t centerOfGravityOffset;
private:
	BioloidController _controller;
	// offset from leg joint relative to body
	ik_req_t endpoints[LEG_COUNT];
	// offset from leg joint relative to body
	ik_req_t currentEndpoints[LEG_COUNT];
	int tranTime;
	float cycleTime;
	int stepsInCycle;
	int liftHeight;
	int step;
	int currentGait;
	int gaitLegNo[LEG_COUNT];   // order to move legs in
	// relative to leg quadrant
	ik_req_t gaits[LEG_COUNT];  // gait position
	int pushSteps;
	ik_req_t nextEndPoint;
	int stepToStepCounter;
	ik_req_t stepToVector;
	long stepToMSec;

	// Gait methods
	// implemented gait types methods
	void DefaultGaitSetup() {};
	ik_req_t DefaultGaitGen(int leg);
	ik_req_t SmoothGaitGen(int leg);
	ik_req_t StepToGaitGen(int leg);
	void setupStepToGait();

	void doLegIK(int legId, int coxaId, int femurId, int tibiaId);
	void doIK();
	// setup the starting positions of the legs.
	void setEndpoints(int x, int y, int z);

	// helpers
	void printServoError(int legId, int jointId, int servoValue);
	void setJointValue(int legId, int jointId, int rawValue);
	bool isMoving();
	bool isAtEndpoints();

	// gait function pointers
	ik_req_t (GaitEngine::*gaitGen)(int leg);
	void (GaitEngine::*gaitSetup)();
};

#endif