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
#define STD_TRANSITION          65

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
	bool isContiouslySteppingTo();
	void cacheGaits();
	void restoreCachedGaits();
	void setPeriodMillis(int millis);

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

	// cycle period time variables
	unsigned long lastFrameMillis;	// the last frame start time in milliseconds
	unsigned long gaitStartMillis;	// the start time of the gait
	float normalizeSignalTime;		// the normalized position [0, 1) within the cycle period
	float frameTime;   				// the delta time between current frame and the last
	int periodMillis;

	int liftHeight;
	int currentGait;
	float gaitLegOffset[LEG_COUNT];
	ik_req_t gaits[LEG_COUNT];  // gait positions relative to leg quadrant
	ik_req_t cachedGaits[LEG_COUNT];  // cached gait positions

	// step to variables
	ik_req_t nextEndPoint;
	vec2 stepToVectors[LEG_COUNT];
	long stepToMSec;
	// COG - center of gravity
	bool isCogCompensationEnabled;
	float cogAmplitudeLR;
	float cogAmplitudeFB;
	float cogDampeningFactor;

	// Gait methods
	// implemented gait types methods
	ik_req_t ContinuousGaitGen(char leg);
	void ContinuousGaitSetup();
	// special step to gait
	ik_req_t ContinuousStepToGaitGen(char legId);
	void setupContinousStepToGait();
	void setupGeoRippleGait();

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

	// gait update steps
	void updateGaitAndFootPositions();
	void adjustFootPositionsByBodyFrame();
	void solveAndUpdateLegJoints();

	void updateFrameTime();
};

#endif