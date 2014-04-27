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
// geo stable ripple gait
#define RIPPLE_GEO				6

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

	// float bodyRotX;    // body roll
	// float bodyRotY;    // body pitch
	// float bodyRotZ;    // body rotation
	// int bodyPosX;
	// int bodyPosY;

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
	BioloidController _controller;
	// offset from leg joint relative to body
	ik_req_t defaultFootPositions[LEG_COUNT];
	// offset from leg joint relative to body
	ik_req_t currentFootPositions[LEG_COUNT];

	char  legJoints[4][3];

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
	ik_req_t DefaultGaitGen(char leg);
	ik_req_t SmoothGaitGen(char leg);
	ik_req_t StepToGaitGen(char leg);
	void setupStepToGait();

	// void doLegIK(int legId, int coxaId, int femurId, int tibiaId);
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


	// COG - center of gravity
	bool isCogCompensationEnabled;
	float amp_LeftRight;
	float amp_FrontBack;
	vec2 calculateDesiredCOG(float t);
	vec2 calculateCogVector(vec2 cog, vec2 leg_a, vec2 leg_b);

	void setupGeoRippleGait();

	// gait update steps
	void updateGaitAndFootPositions();
	void updateCOG();
	void adjustFootPositionsByBodyFrame();
	void solveAndUpdateLegJoints();
};

#endif