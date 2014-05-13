#ifndef _GAIT_ENGINE_H
#define _GAIT_ENGINE_H

#include <BioloidController.h>
#include <vmath.h>
#include "inverse_kinematic.h"

#define GaitSetup (*this.*gaitSetup)
#define GaitGen (*this.*gaitGen)

#define maxValue(n) maxs[n-1]
#define minValue(n) mins[n-1]
#define neutral(n) neutrals[n-1]
 
// ripple gate move one leg at the time
#define RIPPLE                  0
#define RIPPLE_SMOOTH           1
// amble gaits move two alternate legs at a time
#define AMBLE                   2
#define AMBLE_SMOOTH            3
// special ripple gait to switch endpoins
#define RIPPLE_STEP_TO 			5
// geo stable ripple gait
#define RIPPLE_GEO				6

// Standard Transition time should be of the form (k*BIOLOID_FRAME_LENGTH)-1
// for maximal accuracy. BIOLOID_FRAME_LENGTH = 33ms, so good options include:
// 32, 65, 98, etc...
#define STD_TRANSITION          98

class GaitEngine {
public:
	GaitEngine();
	void setupContoller();
	void setBioloidController(BioloidController* bioloidController);
    void update();
    void readPose();
    void slowStart(long msec);
    void doPose(const unsigned int* addr, long msec);
    void setupIK();
	void gaitSelect(int GaitType);
	void setStepToTarget(int x, int y, int z, long msec);
	bool isSteppingTo();
	bool isContiouslySteppingTo();
	void cacheGaits();
	void restoreCachedGaits();

    //Parameters for manipulating body position 
	vec3 bodyRot;
	vec2 bodyPos;

	// Parameters for gait manipulation */
	int Xspeed;
	int Yspeed;
	float Rspeed;
	// bodyPos already does x,y of this it seems
	ik_req_t centerOfGravityOffset;
private:
	BioloidController* controller;
	// offset from leg joint relative to body
	ik_req_t defaultFootPositions[LEG_COUNT];
	// offset from leg joint relative to body
	ik_req_t currentFootPositions[LEG_COUNT];
	// temporary gait variables, used by step to as a reference point
	ik_req_t tempGaits[LEG_COUNT];  // cached gait positions

	char  legJoints[4][3];

	unsigned long gaitStartTime;	// the milli time when gait was started
	float gaitCycleSignal;			// the position with in the step cycle
	float deltaCycleSignal;   		// the delta time between current step and the last

	int tranTime;
	int cycleTimeMillis;
	float cycleTime;
	int stepsInCycle;
	int liftHeight;
	int step;
	int currentGait;
	int gaitLegNo[LEG_COUNT];   // order to move legs in

	float gaitLegOffset[LEG_COUNT];
	// relative to leg quadrant
	ik_req_t gaits[LEG_COUNT];  // gait positions
	ik_req_t cachedGaits[LEG_COUNT];  // cached gait positions

	int pushSteps;
	ik_req_t nextEndPoint;
	int stepToStepCounter;
	ik_req_t stepToVector;
	vec2 stepToVectors[LEG_COUNT];
	long stepToMSec;
	// COG - center of gravity
	bool isCogCompensationEnabled;
	float cogAmplitudeLR;
	float cogAmplitudeFB;
	float cogDampeningFactor;

	// Gait methods
	// implemented gait types methods
	void DefaultGaitSetup() {};
	ik_req_t DefaultGaitGen(char leg);
	ik_req_t SmoothGaitGen(char leg);
	ik_req_t StepToGaitGen(char leg);
	void setupStepToGait();
	
	ik_req_t ContinuousGaitGen(char leg);
	void ContinuousGaitSetup();

	ik_req_t ContinuousStepToGaitGen(char legId);
	void setupContinousStepToGait();

	void doIK();
	// setup the starting positions of the legs.
	void setDefaultFootPosition(int x, int y, int z);

	// helpers
	void printServoError(char legId, char jointId, int servoValue);
	void setJointValue(char legId, char jointId, int rawValue);
	bool isMoving();
	bool isAtEndpoints();

	// gait function pointers
	ik_req_t (GaitEngine::*gaitGen)(char leg);
	void (GaitEngine::*gaitSetup)();

	vec2 calculateDesiredCOG(float t);
	vec2 calculateCogVector(vec2 cog, vec2 leg_a, vec2 leg_b);
	float updateCogDampening();

	void setupGeoRippleGait();

	// gait update steps
	void updateGaitAndFootPositions();
	void adjustFootPositionsByBodyFrame();
	void solveAndUpdateLegJoints();
};

#endif