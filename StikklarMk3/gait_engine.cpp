#include "gait_engine.h"
#include <math.h>
#include "config.h"
#include <Arduino.h>
#include <ax12.h>

#define M_2_PI M_PI*2
#define M_4_PI M_PI*4
#define M_PI_2 M_PI/2

const char* LEG_NAMES[] = {
	"Right Front", 
	"Left Front", 
	"Left Rear",
	"Right Rear" 
};

const int mins[] = {
	60, 162, 95, 0, 
	60, 162, 95, 0, 
	60, 162, 95, 0, 
	60, 162, 95, 0};

const int maxs[] = {
	960, 850, 1005, 0, 
	960, 850, 1005, 0, 
	960, 850, 1005, 0, 
	960, 850, 1005, 0};

const int neutrals[] = {
	364, 512, 742, 0, 
	669, 512, 742, 0, 
	669, 512, 742, 0, 
	364, 512, 742, 0};

const bool signs[] = {
	true, true, false, false, 
	false, true, false, false, 
	false, true, false, true, 
	true, true, false, true};

int signedValue(int n, int val) {
	if (signs[n-1]) {
		return val;
	} else {
		return -val;
	}
}

GaitEngine::GaitEngine(): currentGait(-1), periodMillis(2000) {
	gaitGen = &GaitEngine::ContinuousGaitGen;
	gaitSetup = &GaitEngine::ContinuousGaitSetup;
}

void GaitEngine::setPeriodMillis(int millis) {
	if (millis < 250) return;
	periodMillis = millis;
}

void GaitEngine::setBioloidController(BioloidController* bioloidController) {
	controller = bioloidController;
}

void GaitEngine::setupContoller() {
	// configuring the bioloid interpolation controller
	controller->poseSize = 12;
	controller->setId(0, RF_COXA);		// 1
	controller->setId(1, RF_FEMUR);		// 2
	controller->setId(2, RF_TIBIA);		// 3
	controller->setId(3, RR_COXA);		// 5
	controller->setId(4, RR_FEMUR);		// 6
	controller->setId(5, RR_TIBIA);		// 7
	controller->setId(6, LF_COXA);		// 9
	controller->setId(7, LF_FEMUR);		// 10
	controller->setId(8, LF_TIBIA);		// 11
	controller->setId(9, LR_COXA);		// 13
	controller->setId(10, LR_FEMUR);	// 14
	controller->setId(11, LR_TIBIA);	// 15

	// TODO: this can be setup as a static [4][3] array
	legJoints[RIGHT_FRONT][0] = RF_COXA;
	legJoints[RIGHT_FRONT][1] = RF_FEMUR;
	legJoints[RIGHT_FRONT][2] = RF_TIBIA;
	legJoints[LEFT_FRONT][0] = LF_COXA;
	legJoints[LEFT_FRONT][1] = LF_FEMUR;
	legJoints[LEFT_FRONT][2] = LF_TIBIA;
	legJoints[LEFT_REAR][0] = LR_COXA;
	legJoints[LEFT_REAR][1] = LR_FEMUR;
	legJoints[LEFT_REAR][2] = LR_TIBIA;
	legJoints[RIGHT_REAR][0] = RR_COXA;
	legJoints[RIGHT_REAR][1] = RR_FEMUR;
	legJoints[RIGHT_REAR][2] = RR_TIBIA;

	// reset any body offset
	bodyPos.x = 0;
	bodyPos.y = 0;
}

void GaitEngine::setDefaultFootPosition(int x, int y, int z){
    defaultFootPositions[RIGHT_FRONT].x = x;
    defaultFootPositions[RIGHT_FRONT].y = y;
    defaultFootPositions[RIGHT_FRONT].z = z;

    defaultFootPositions[RIGHT_REAR].x = -x;
    defaultFootPositions[RIGHT_REAR].y = y;
    defaultFootPositions[RIGHT_REAR].z = z;

    defaultFootPositions[LEFT_FRONT].x = x;
    defaultFootPositions[LEFT_FRONT].y = -y;
    defaultFootPositions[LEFT_FRONT].z = z;

    defaultFootPositions[LEFT_REAR].x = -x;
    defaultFootPositions[LEFT_REAR].y = -y;
    defaultFootPositions[LEFT_REAR].z = z;
}

void GaitEngine::readPose() {
	controller->readPose();
}

void GaitEngine::update() {
	doIK();
}

void GaitEngine::slowStart(long msec) {
    doIK();
    controller->interpolateSetup(msec);
    while(controller->interpolating > 0) {
        controller->interpolateStep();
        delay(3);
    }	
}

void GaitEngine::doPose(const unsigned int * addr, long msec) {
	// TODO: this needs to be non blocking 
    controller->loadPose(addr);
    controller->interpolateSetup(msec);
    while(controller->interpolating > 0) {
        controller->interpolateStep();
        delay(3);
    }	
}

// Setup the starting positions of the legs.
void GaitEngine::setupIK(){
    // this is used to move the center of gravity 
    centerOfGravityOffset.x = 0;
    centerOfGravityOffset.y = 0;
    centerOfGravityOffset.z = 0;

    setDefaultFootPosition(
    	DEFAULT_ENDPOINT_X, 
    	DEFAULT_ENDPOINT_Y, 
    	DEFAULT_ENDPOINT_Z
    );

    // default 33
    liftHeight = LEG_LIFT_HIGHT; 
}

void GaitEngine::setJointValue(char legId, char jointId, int rawValue){
    int servo = neutral(jointId) + signedValue(jointId, rawValue);
    if(servo < maxValue(jointId) && servo > minValue(jointId))
        controller->setNextPose(jointId, servo);
    else
        printServoError(legId, jointId, servo);
}

void GaitEngine::updateGaitAndFootPositions() {
	ik_req_t gait;
	for(char legId=0; legId < LEG_COUNT; legId++) {
		gait = GaitGen(legId);
	    // add the gate offset to the default endpoint
	    currentFootPositions[legId] = addPoints(defaultFootPositions[legId], gait);
	}
}

void GaitEngine::adjustFootPositionsByBodyFrame() {
	ik_req_t footPosition;
	vec2 offset;
	vec2 baseOffset(X_COXA, Y_COXA);
	for(char legId=0; legId < LEG_COUNT; legId++) {
		footPosition = currentFootPositions[legId];
		// adjust for the body orientation
		offset = getForQuadrant(legId, baseOffset);
		ik_req_t correction = bodyIK(bodyRot, bodyPos, footPosition, offset);
	    // add rotation correction to the foot pos to get the new positon
	    currentFootPositions[legId] = addPoints(footPosition, correction);
	}
}

void GaitEngine::solveAndUpdateLegJoints() {
	ik_sol_t legSolution;
	ik_req_t footPosition;
	for(char legId=0; legId < LEG_COUNT; legId++) {
		footPosition = currentFootPositions[legId];
		// Translate endpoint to leg space for for IK solving
	    adjustEndpointForLeg(legId, footPosition);

	    // solve the IK
	    legSolution = legIK(footPosition);

	    setJointValue(legId, legJoints[legId][0], legSolution.coxa);
	    setJointValue(legId, legJoints[legId][1], legSolution.femur);
	    setJointValue(legId, legJoints[legId][2], legSolution.tibia);
	}
}

void GaitEngine::doIK() {
	updateFrameTime();

    GaitSetup();

    updateGaitAndFootPositions();
    if (isCogCompensationEnabled) {
	    // we simply override the wanted body pos and let the bodyIK figure it out
	    bodyPos = calculateDesiredCOG(normalizeSignalTime);
	}
    adjustFootPositionsByBodyFrame();
    solveAndUpdateLegJoints();
}

void GaitEngine::printServoError(char legId, char jointId, int servoValue){
    Serial.print(LEG_NAMES[legId]);
    Serial.print(": servo #");
    Serial.print(int(jointId));
    Serial.print(" IK exceded limits: ");
    Serial.println(servoValue);
}

// Select a new gait
void GaitEngine::gaitSelect(int GaitType){
	if(GaitType == currentGait) return;

	log("gaitSelect:"); logln(GaitType);

	currentGait = GaitType;
	gaitStartMillis = 0;
	periodMillis = 3000;
	isCogCompensationEnabled = false;

	// reset the endpoints in case somebody messed with them
	setDefaultFootPosition(
    	DEFAULT_ENDPOINT_X, 
    	DEFAULT_ENDPOINT_Y, 
    	DEFAULT_ENDPOINT_Z
    );
	if(GaitType == RIPPLE_STEP_TO){
		setupContinousStepToGait();
	} else if(GaitType == RIPPLE_GEO){
		setupGeoRippleGait();
	}
}

void GaitEngine::setupGeoRippleGait() {
	gaitGen = &GaitEngine::ContinuousGaitGen;
	gaitSetup = &GaitEngine::ContinuousGaitSetup;
	gaitLegOffset[LEFT_FRONT] = 0.0;
	gaitLegOffset[RIGHT_REAR] = 0.25;
	gaitLegOffset[RIGHT_FRONT] = 0.5;
	gaitLegOffset[LEFT_REAR] = 0.75;

	periodMillis = 2000;

	isCogCompensationEnabled = true;

	cogAmplitudeLR = 0.30;
	cogAmplitudeFB = 0.15;
}

bool GaitEngine::isMoving() {
	return (
		(Xspeed > 5 || Xspeed < -5) || 
		(Yspeed > 5 || Yspeed < -5) || 
		(Rspeed > 0.05 || Rspeed < -0.05));
}

float GaitEngine::updateCogDampening() {
	if ( isMoving() || isContiouslySteppingTo() ) {
		cogDampeningFactor +=  (frameTime / COG_DAMP_INCREASE_RATIO);
		if (cogDampeningFactor > 1.0) {
			cogDampeningFactor = 1.0;
		};
	} else {
		cogDampeningFactor -=  (frameTime / COG_DAMP_DECREASE_RATIO);
		if (cogDampeningFactor < 0.0) {
			cogDampeningFactor = 0.0;
		};
	}
}

vec2 GaitEngine::calculateDesiredCOG(float t) {
	// logval("t", t);
	// t is a value from 0 to 1 for where we are in the walk cycle
	// compute leg positins relative to robot center
	vec2 legpos[4];
	for (int legId=0; legId < 4; legId++){
		legpos[legId].x = currentFootPositions[legId].x + getXdisp(legId);
		legpos[legId].y = currentFootPositions[legId].y + getYdisp(legId);
	}
	// compute center by dividing 
	vec2 cog;
	lineIntersection(
		legpos[RIGHT_FRONT],
		legpos[LEFT_REAR],
		legpos[LEFT_FRONT],
		legpos[RIGHT_REAR],
		cog
	);

	// X_cog = X_c + a_lr*A_lr*sin(wt+(pi/2))+a_fb*A_fb*sin(2wt)
	// w = 2*pi*f , f is not neede as it is aleady factored into t
	// Bisector vectors
	vec2 A_lr = calculateCogVector(cog, legpos[LEFT_FRONT], legpos[LEFT_REAR]);
	vec2 A_fb = calculateCogVector(cog, legpos[LEFT_FRONT], legpos[RIGHT_FRONT]);

	// compute cyclic bisector vector scale factors
	// apply a dampening factor to smothe oscillation when not moving and taking off
	float m_lr = cogAmplitudeLR * cogDampeningFactor * sin((M_2_PI*t) - M_PI_2);
	float m_fb = cogAmplitudeFB * cogDampeningFactor * sin(2*M_2_PI*t);

	vec2 out = -cog - ((A_lr*m_lr) - (A_fb*m_fb));

	// dlogstart("cog");
	// dlog(t);
	// dlogvec2(cog);
	// dlogvec2(out);
	// dlogvec2(A_lr);
	// dlogvec2(A_fb);
	// for (int legId=0; legId < 4; legId++){
	// 	dlogvec2(legpos[legId]);
	// 	dlog(currentFootPositions[legId].z);
	// }
	// dlogend();
	return out;
}

vec2 GaitEngine::calculateCogVector(vec2 cog, vec2 leg_a, vec2 leg_b) {
	// find angle bisector vector
	vec2 ca = leg_a - cog;
	vec2 cb = leg_b - cog;
	vec2 bisec = ca*cb.len() + cb*ca.len();
	
	lineIntersection(cog, cog+bisec, leg_a, leg_b, bisec);
	bisec = bisec - cog;
	return bisec;
}

// Continouse gait generation
ik_req_t GaitEngine::ContinuousGaitGen(char legId) {
	if( isMoving() ) {
		float t = normalizeSignalTime - gaitLegOffset[legId];
		// normalize to range [0, 1)
		if (t < 0.0) { t += 1.0; }

		if(t < 0.25)
		{
			float t4pi = t*M_4_PI;
			// a sinodal s curve from 0 to 1
			float distance = (1 + cos(M_PI + t4pi)) * 0.5;
			// relocate leg
			gaits[legId].x = Xspeed * distance;
			gaits[legId].y = Yspeed * distance;
			gaits[legId].z = -liftHeight * sin(t4pi);
			gaits[legId].r = Rspeed * distance;
		} else {
			// move body forward
			// to interpolate over entire range 75% duty cycle
			float distance = 1.0 - (t - 0.25) / 0.75;
			gaits[legId].x = Xspeed * distance;
			gaits[legId].y = Yspeed * distance;
			gaits[legId].z = 0;
			gaits[legId].r = Rspeed * distance;
		}
	} else { // stopped
		gaits[legId].z = 0;
	}
	return gaits[legId];
}

void GaitEngine::updateFrameTime() {
	// period is the duration of the cycle
	// frame is the intercal between two samples
	// time referes to the t in 
	unsigned long currentMillis = millis();
	unsigned long frameMillis = currentMillis - lastFrameMillis;
	lastFrameMillis = currentMillis;
	frameTime =  float(frameMillis) / (periodMillis);
	normalizeSignalTime += frameTime;

	// clamp the signal time
	while (normalizeSignalTime > 1.0) { normalizeSignalTime -= 1.0; }

	updateCogDampening();
}

void GaitEngine::ContinuousGaitSetup() {
	if (gaitStartMillis == 0) {
		gaitStartMillis = millis();
	}
}

// this would immediately be followd by gaitSelect
void GaitEngine::setStepToTarget(int x, int y, int z, long msec) {
	nextEndPoint.x = x;
	nextEndPoint.y = y;
	nextEndPoint.z = z;
	stepToMSec = msec;
	// reset the gait in case we retrigger the same, which is fine with new params
	currentGait = -1;
}

bool GaitEngine::isContiouslySteppingTo() {
	// this can never be valid if the wrong gait is selected
	if (currentGait != RIPPLE_STEP_TO) return false;

	// first cycle where work takes place is still running
	if ((millis() - gaitStartMillis) < stepToMSec) return true;

	// make sure all legs have landed before giving the GO
	for (char legId=0; legId < LEG_COUNT; legId++) {
		if (gaits[legId].z != 0) return true;
	}

	return false;
}

void GaitEngine::setupContinousStepToGait() {
	// set up a single cycle to get us to where we want to be
	gaitGen = &GaitEngine::ContinuousStepToGaitGen;
	gaitSetup = &GaitEngine::ContinuousGaitSetup;

	gaitLegOffset[LEFT_FRONT] = 0.0;
	gaitLegOffset[RIGHT_REAR] = 0.25;
	gaitLegOffset[RIGHT_FRONT] = 0.5;
	gaitLegOffset[LEFT_REAR] = 0.75;

	periodMillis = stepToMSec;
	gaitStartMillis = millis();

	isCogCompensationEnabled = true;

	cogAmplitudeLR = 0.15;
	cogAmplitudeFB = 0.15;

	for (char legId=0; legId < LEG_COUNT; legId++) {
		stepToVectors[legId].x = xForLeg(legId, nextEndPoint.x) - currentFootPositions[legId].x;
		stepToVectors[legId].y = yForLeg(legId, nextEndPoint.y) - currentFootPositions[legId].y;
		tempGaits[legId] = gaits[legId];
	}
}

// Continouse gait generation
ik_req_t GaitEngine::ContinuousStepToGaitGen(char legId) {
	if( (millis() - gaitStartMillis) < stepToMSec ) {
		// we will only enter here the first cycle
		float t = normalizeSignalTime - gaitLegOffset[legId];

		if (t < 0.0) {
			gaits[legId].z = 0;
		} else if (t < 0.25) {
			float t4pi = t*M_4_PI;
			// a sinodal s curve from 1 to 0
			float distance = (1 + cos(M_PI + t4pi)) * 0.5;
			// relocate leg
			gaits[legId].x = tempGaits[legId].x + stepToVectors[legId].x * distance;
			gaits[legId].y = tempGaits[legId].y + stepToVectors[legId].y * distance;
			gaits[legId].z = -liftHeight * sin(t4pi);
			gaits[legId].r = 0;
		} else {
			// after the leg relcation we just need to keep the leg down
			gaits[legId].x = tempGaits[legId].x + stepToVectors[legId].x;
			gaits[legId].y = tempGaits[legId].y + stepToVectors[legId].y;
			gaits[legId].z = 0;
		}
	} else { // stopped
		gaits[legId].z = 0;
	}
	return gaits[legId];
}

void GaitEngine::cacheGaits() {
	for (char legId=0; legId < LEG_COUNT; legId++) {
		cachedGaits[legId] = gaits[legId];
	}
}

void GaitEngine::restoreCachedGaits() {
	for (char legId=0; legId < LEG_COUNT; legId++) {
		gaits[legId] = cachedGaits[legId];
	}
}