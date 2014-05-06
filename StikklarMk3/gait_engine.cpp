#include "gait_engine.h"
#include <math.h>
#include "config.h"
#include <Arduino.h>
#include <ax12.h>

#define M_2_PI M_PI*2
#define M_PI_2 M_PI/2

const char* LEG_NAMES[] = {
	"Right Front", 
	"Left Front", 
	"Left Rear",
	"Right Rear" 
};

int mins[] = {
	60, 162, 95, 0, 
	60, 161, 95, 0, 
	60, 162, 95, 0, 
	60, 162, 95, 0};

int maxs[] = {
	960, 850, 1005, 0, 
	960, 852, 1005, 0, 
	960, 864, 1005, 0, 
	960, 840, 1005, 0};

int neutrals[] = {
	364, 516, 754, 0, 
	669, 494, 731, 0, 
	669, 498, 748, 0, 
	364, 506, 745, 0};

bool signs[] = {
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

GaitEngine::GaitEngine() {
	// configuring the bioloid interpolation controller
	_controller.setup(12);
	_controller.setId(0, RF_COXA);		// 1
	_controller.setId(1, RF_FEMUR);		// 2
	_controller.setId(2, RF_TIBIA);		// 3
	_controller.setId(3, RR_COXA);		// 5
	_controller.setId(4, RR_FEMUR);		// 6
	_controller.setId(5, RR_TIBIA);		// 7
	_controller.setId(6, LF_COXA);		// 9
	_controller.setId(7, LF_FEMUR);		// 10
	_controller.setId(8, LF_TIBIA);		// 11
	_controller.setId(9, LR_COXA);		// 13
	_controller.setId(10, LR_FEMUR);	// 14
	_controller.setId(11, LR_TIBIA);	// 15

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

	gaitGen = &GaitEngine::DefaultGaitGen;
	gaitSetup = &GaitEngine::DefaultGaitSetup;

	bodyPos.x = 0;
	bodyPos.y = 0;
	currentGait = -1;
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
	_controller.readPose();
}

void GaitEngine::update() {
	// if our previous interpolation is complete, recompute the IK
	if(_controller.interpolating == 0){
		doIK();
		_controller.interpolateSetup(tranTime);
	}

	// update joints
	_controller.interpolateStep();	
}

void GaitEngine::slowStart(long msec) {
    doIK();
    _controller.interpolateSetup(msec);
    while(_controller.interpolating > 0) {
        _controller.interpolateStep();
        delay(3);
    }	
}

void GaitEngine::doPose(const unsigned int * addr, long msec) {
    _controller.loadPose(addr);
    _controller.interpolateSetup(msec);
    while(_controller.interpolating > 0) {
        _controller.interpolateStep();
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
    liftHeight = 50; 
    stepsInCycle = 1;
    step = 0;
}

void GaitEngine::setJointValue(char legId, char jointId, int rawValue){
    int servo = neutral(jointId) + signedValue(jointId, rawValue);
    if(servo < maxValue(jointId) && servo > minValue(jointId))
        _controller.setNextPose(jointId, servo);
    else
        printServoError(legId, jointId, servo);
}

void GaitEngine::updateGaitAndFootPositions() {
	ik_req_t gait;
	for(char legId=0; legId < LEG_COUNT; legId++) {
		gait = GaitGen(legId);
	    // add the gate offset to the default endpoint
	    currentFootPositions[legId] = addPoints(defaultFootPositions[legId], gait);
	    //log(int(legId)); logvec(" gait", currentFootPositions[legId]);
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
	    //log(int(legId)); logvec(" body", currentFootPositions[legId]);
	}
}

void GaitEngine::solveAndUpdateLegJoints() {
	ik_sol_t legSolution;
	ik_req_t footPosition;
	for(char legId=0; legId < LEG_COUNT; legId++) {
		footPosition = currentFootPositions[legId];
		// Translate endpoint to leg space for for IK solving
	    adjustEndpointForLeg(legId, footPosition);
	    //log(int(legId)); logvec(" ik", footPosition);
	    // solve the IK
	    legSolution = legIK(footPosition);

	    setJointValue(legId, legJoints[legId][0], legSolution.coxa);
	    setJointValue(legId, legJoints[legId][1], legSolution.femur);
	    setJointValue(legId, legJoints[legId][2], legSolution.tibia);
	}
}

void GaitEngine::doIK() {
    GaitSetup();

    updateGaitAndFootPositions();
    if (isCogCompensationEnabled) {
	    // we simply override the wanted body pos and let the bodyIK figure it out
	    bodyPos = calculateDesiredCOG(currentCycleOffset);
	}
    adjustFootPositionsByBodyFrame();
    solveAndUpdateLegJoints();

    step = (step+1) % stepsInCycle;
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
	tranTime = STD_TRANSITION;
	cycleTime = 0;
	isCogCompensationEnabled = false;
	// reset the endpoints in case somebody messed with them
	setDefaultFootPosition(
    	DEFAULT_ENDPOINT_X, 
    	DEFAULT_ENDPOINT_Y, 
    	DEFAULT_ENDPOINT_Z
    );
	// simple ripple, 8 steps
	if(GaitType == RIPPLE){
		gaitGen = &GaitEngine::DefaultGaitGen;
		gaitSetup = &GaitEngine::DefaultGaitSetup;
		gaitLegNo[RIGHT_FRONT] = 0;
		gaitLegNo[LEFT_REAR] = 2;
		gaitLegNo[LEFT_FRONT] = 4;
		gaitLegNo[RIGHT_REAR] = 6;
		pushSteps = 6;
		stepsInCycle = 8;
	// smoother ripple with twice as many steps (but half as fast)
	}else if(GaitType == RIPPLE_SMOOTH){
		gaitGen = &GaitEngine::SmoothGaitGen;
		gaitSetup = &GaitEngine::DefaultGaitSetup;
		gaitLegNo[RIGHT_FRONT] = 0;
		gaitLegNo[LEFT_REAR] = 4;
		gaitLegNo[LEFT_FRONT] = 8;
		gaitLegNo[RIGHT_REAR] = 12;
		pushSteps = 12;
		stepsInCycle = 16;
	// an amble moves opposing pairs of legs at once
	}else if(GaitType == AMBLE){
		gaitGen = &GaitEngine::DefaultGaitGen;
		gaitSetup = &GaitEngine::DefaultGaitSetup;
		gaitLegNo[RIGHT_FRONT] = 0;
		gaitLegNo[LEFT_REAR] = 0;
		gaitLegNo[LEFT_FRONT] = 2;
		gaitLegNo[RIGHT_REAR] = 2;
		pushSteps = 2;
		stepsInCycle = 4;
	// smoother amble with twice as many steps (but half as fast)
	}else if(GaitType == AMBLE_SMOOTH){
		gaitGen = &GaitEngine::SmoothGaitGen;
		gaitSetup = &GaitEngine::DefaultGaitSetup;
		gaitLegNo[RIGHT_FRONT] = 0;
		gaitLegNo[LEFT_REAR] = 0;
		gaitLegNo[LEFT_FRONT] = 4;
		gaitLegNo[RIGHT_REAR] = 4;
		pushSteps = 4;
		stepsInCycle = 8;
		tranTime = 65;
	} else if(GaitType == RIPPLE_STEP_TO){
		setupStepToGait();
	} else if(GaitType == RIPPLE_GEO){
		setupGeoRippleGait();
	}

	if(cycleTimeMillis == 0) {
		cycleTimeMillis = stepsInCycle*tranTime;
		cycleTime = cycleTimeMillis / 1000.0;
	}
	step = 0;
}

void GaitEngine::setupGeoRippleGait() {
	gaitGen = &GaitEngine::ContinuousGaitGen;
	gaitSetup = &GaitEngine::ContinuousGaitSetup;
	gaitLegOffset[LEFT_FRONT] = 0.0;
	gaitLegOffset[RIGHT_REAR] = 0.25;
	gaitLegOffset[RIGHT_FRONT] = 0.5;
	gaitLegOffset[LEFT_REAR] = 0.75;

	cycleTime = 2.0;
	cycleTimeMillis = 2000;
	gaitStartTime = 0;

	isCogCompensationEnabled = true;
	tranTime = 65;

	amp_LeftRight = 0.25;
	amp_FrontBack = 0.2;
}

// this would immediately be followd by gaitSelect
void GaitEngine::setupStepToGait() {
	gaitGen = &GaitEngine::StepToGaitGen;
	gaitSetup = &GaitEngine::DefaultGaitSetup;
	gaitLegNo[RIGHT_FRONT] = 0;
	gaitLegNo[LEFT_REAR] = 4;
	gaitLegNo[LEFT_FRONT] = 8;
	gaitLegNo[RIGHT_REAR] = 12;
	pushSteps = 12;
	stepsInCycle = 16;

	int cyclesToComplete = int((stepToMSec / float(STD_TRANSITION*stepsInCycle)) + 0.5);
	stepToStepCounter = cyclesToComplete * stepsInCycle * LEG_COUNT;
	stepToVector.x = (nextEndPoint.x - currentFootPositions[RIGHT_FRONT].x)/float(cyclesToComplete) + 0.5;
	stepToVector.y = (nextEndPoint.y - currentFootPositions[RIGHT_FRONT].y)/float(cyclesToComplete) + 0.5;

	for (char legId=0; legId < LEG_COUNT; legId++) {
		// set gait offset as the the vector to from new target to current position
		gaits[legId].x = currentFootPositions[legId].x - xForLeg(legId, nextEndPoint.x);
		gaits[legId].y = currentFootPositions[legId].y - yForLeg(legId, nextEndPoint.y);
		gaits[legId].z = 0;
	}
	setDefaultFootPosition(nextEndPoint.x, nextEndPoint.y, nextEndPoint.z);
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

bool GaitEngine::isMoving() {
	return ((Xspeed > 5 || Xspeed < -5) || (Yspeed > 5 || Yspeed < -5) || (Rspeed > 0.05 || Rspeed < -0.05));
}

// Simple calculations at the beginning of a cycle.
void DefaultGaitSetup() {
    // nothing!
}

// Simple, fast, and rough gait. StepsInCycle == leg count.
// Legs will make a fast triangular stroke. 
ik_req_t GaitEngine::DefaultGaitGen(char leg){
 	if ( isMoving() ) {
	    // are we moving?
	    if(step == gaitLegNo[leg]){
			// leg up, middle position
			gaits[leg].x = 0;
			gaits[leg].y = 0;
			gaits[leg].z = -liftHeight;
			gaits[leg].r = 0;
	    }else if(((step == gaitLegNo[leg]+1) || (step == gaitLegNo[leg]-(stepsInCycle-1))) && (gaits[leg].z < 0)){
			// leg down position                                           NOTE: dutyFactor = pushSteps/StepsInCycle
			gaits[leg].x = (Xspeed*cycleTime*pushSteps)/(2*stepsInCycle);     // travel/Cycle = speed*cycleTime
			gaits[leg].y = (Yspeed*cycleTime*pushSteps)/(2*stepsInCycle);     // Stride = travel/Cycle * dutyFactor
			gaits[leg].z = 0;                                                 //   = speed*cycleTime*pushSteps/stepsInCycle
			gaits[leg].r = (Rspeed*cycleTime*pushSteps)/(2*stepsInCycle);     //   we move Stride/2 here
	    }else{
			// move body forward
			gaits[leg].x = gaits[leg].x - (Xspeed*cycleTime)/stepsInCycle;    // note calculations for Stride above
			gaits[leg].y = gaits[leg].y - (Yspeed*cycleTime)/stepsInCycle;    // we have to move Stride/pushSteps here
			gaits[leg].z = 0;                                                 //   = speed*cycleTime*pushSteps/stepsInCycle*pushSteps
			gaits[leg].r = gaits[leg].r - (Rspeed*cycleTime)/stepsInCycle;    //   = speed*cycleTime/stepsInCycle
		}
	}else{ // stopped
		gaits[leg].z = 0;
	}
	return gaits[leg];
}

// Smoother, slower gait. Legs will make a arc stroke.
ik_req_t GaitEngine::SmoothGaitGen(char leg) {
	if( isMoving() ){
		// are we moving?
		if(step == gaitLegNo[leg]){
			// leg up, halfway to middle
			gaits[leg].x = gaits[leg].x/2;
			gaits[leg].y = gaits[leg].y/2;
			gaits[leg].z = -liftHeight/2;
			gaits[leg].r = gaits[leg].r/2;
		}else if((step == gaitLegNo[leg]+1) && (gaits[leg].z < 0)){
			// leg up position
			gaits[leg].x = 0;
			gaits[leg].y = 0;
			gaits[leg].z = -liftHeight;
			gaits[leg].r = 0;
		}else if((step == gaitLegNo[leg] + 2) && (gaits[leg].z < 0)){
			// leg halfway down
			gaits[leg].x = (Xspeed*cycleTime*pushSteps)/(4*stepsInCycle);
			gaits[leg].y = (Yspeed*cycleTime*pushSteps)/(4*stepsInCycle);
			gaits[leg].z = -liftHeight/2;
			gaits[leg].r = (Rspeed*cycleTime*pushSteps)/(4*stepsInCycle);
		}else if((step == gaitLegNo[leg]+3) && (gaits[leg].z < 0)){
			// leg down position                                           NOTE: dutyFactor = pushSteps/StepsInCycle
			gaits[leg].x = (Xspeed*cycleTime*pushSteps)/(2*stepsInCycle);     // travel/Cycle = speed*cycleTime
			gaits[leg].y = (Yspeed*cycleTime*pushSteps)/(2*stepsInCycle);     // Stride = travel/Cycle * dutyFactor
			gaits[leg].z = 0;                                                 //   = speed*cycleTime*pushSteps/stepsInCycle
			gaits[leg].r = (Rspeed*cycleTime*pushSteps)/(2*stepsInCycle);     //   we move Stride/2 here
		}else{
			// move body forward
			gaits[leg].x = gaits[leg].x - (Xspeed*cycleTime)/stepsInCycle;    // note calculations for Stride above
			gaits[leg].y = gaits[leg].y - (Yspeed*cycleTime)/stepsInCycle;    // we have to move Stride/pushSteps here
			gaits[leg].z = 0;                                                 //   = speed*cycleTime*pushSteps/stepsInCycle*pushSteps
			gaits[leg].r = gaits[leg].r - (Rspeed*cycleTime)/stepsInCycle;    //   = speed*cycleTime/stepsInCycle
		}
	}else{ // stopped
		gaits[leg].z = 0;
	}
	return gaits[leg];
}

// this will only work for symetric endpoints
ik_req_t GaitEngine::StepToGaitGen(char legId){
	if( stepToStepCounter > 0 ){
	    // are we moving?
	    if(step == gaitLegNo[legId]){
			// leg up, middle position
			gaits[legId].z = -liftHeight;
			gaits[legId].r = 0;
	    }else if((step == gaitLegNo[legId]+1) && (gaits[legId].z < 0)){
			// leg down position     
			gaits[legId].x += xForLeg(legId, stepToVector.x);
			gaits[legId].y += yForLeg(legId, stepToVector.y);
			gaits[legId].z = 0;                                               
			gaits[legId].r = 0;
			// log(legId); logvec(" leg", gaits[legId]);
	    }else{
	    	// stay put
			gaits[legId].z = 0;
		}
		stepToStepCounter--;
	}else{ // stopped
		gaits[legId].x = 0;
		gaits[legId].y = 0;
		gaits[legId].z = 0;                                               
		gaits[legId].r = 0;
	}
	return gaits[legId];
}

bool GaitEngine::isSteppingTo() {
	return (currentGait == RIPPLE_STEP_TO && stepToStepCounter > 0);
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
	vec2 A_lr = calculateCogVector(cog, legpos[LEFT_FRONT], legpos[LEFT_REAR]);
	vec2 A_fb = calculateCogVector(cog, legpos[LEFT_FRONT], legpos[RIGHT_FRONT]);

	// precompute these factors
	float m_lr = sin((M_2_PI*t) - M_PI_2) * amp_LeftRight;
	float m_fb = sin(2*M_2_PI*t) * amp_FrontBack;

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
		float t = currentCycleOffset - gaitLegOffset[legId];
		// normalize to range [0, 1)
		if (t < 0.0) { t += 1.0; }

		if(t < 0.25)
		{
			// an sinodal s curve from 0 to 1
			float distance = (1 + cos(M_PI + t*M_PI*4)) * 0.5;
			// relocate leg
			gaits[legId].x = Xspeed * distance;
			gaits[legId].y = Yspeed * distance;
			gaits[legId].z = -liftHeight * sin(t*M_PI*4);
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

void GaitEngine::ContinuousGaitSetup() {
	if (gaitStartTime == 0) {
		gaitStartTime = millis();
	}
	int millisIntoCycle = (millis() - gaitStartTime) % cycleTimeMillis;
	currentCycleOffset = millisIntoCycle / float(cycleTimeMillis);
}