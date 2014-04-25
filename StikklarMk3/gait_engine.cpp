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
	161, 162, 95, 0, 
	70, 161, 95, 0, 
	193, 162, 95, 0, 
	41, 162, 95, 0};

int maxs[] = {
	828, 850, 1005, 0, 
	980, 852, 1005, 0, 
	861, 864, 1005, 0, 
	940, 840, 1005, 0};

int neutrals[] = {
	508, 516, 754, 0, 
	364, 494, 731, 0, 
	510, 498, 748, 0, 
	669, 506, 745, 0};

bool signs[] = {
	false, true, false, false, 
	true, true, false, false, 
	true, true, false, true, 
	false, true, false, true};

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
    liftHeight = 60; 
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

void GaitEngine::updateCOG() {
	// compute a t in [0, 1) from splace in duty cycle
    // expectes lf, rr, rf, lr leg order
    float t = step / float(stepsInCycle);
    // we simply override the wanted body pos and let the bodyIK figure it out
    bodyPos = calculateDesiredCOG(t);
}

void GaitEngine::doIK() {
    GaitSetup();

    updateGaitAndFootPositions();
    if (isCogCompensationEnabled) {
    	updateCOG();
	}
    adjustFootPositionsByBodyFrame();
    solveAndUpdateLegJoints();

    step = (step+1) % stepsInCycle;

    //delay(3000);
}


void GaitEngine::printServoError(char legId, char jointId, int servoValue){
    Serial.print(LEG_NAMES[legId]);
    Serial.print(": servo #");
    Serial.print(jointId);
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

	if(cycleTime == 0) {
		cycleTime = (stepsInCycle*tranTime)/1000.0;
	}
	step = 0;
}

void GaitEngine::setupGeoRippleGait() {
	gaitGen = &GaitEngine::SmoothGaitGen;
	gaitSetup = &GaitEngine::DefaultGaitSetup;
	gaitLegNo[RIGHT_FRONT] = 0;
	gaitLegNo[LEFT_REAR] = 4;
	gaitLegNo[LEFT_FRONT] = 8;
	gaitLegNo[RIGHT_REAR] = 12;
	pushSteps = 12;
	stepsInCycle = 16;
	isCogCompensationEnabled = true;
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
	setDefaultFootPosition(
		nextEndPoint.x,
		nextEndPoint.y,
		nextEndPoint.z
	);
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
	// t is a value from 0 to 1 for where we are in the walk cycle
	// compute leg positins relative to robot center
	vec2 legpos[4];
	for (int legId=0; legId < 4; legId++){
		legpos[legId].x = currentFootPositions[legId].x + getXdisp(legId);
		legpos[legId].y = currentFootPositions[legId].y + getYdisp(legId);
	}
	// compute center by dividing 
	vec2 cog = calculateIntersection(
		legpos[RIGHT_FRONT],
		legpos[LEFT_FRONT],
		legpos[LEFT_REAR],
		legpos[RIGHT_REAR]
	);
	// X_cog = X_c + a_lr*A_lr*sin(wt+(pi/2))+a_fb*A_fb*sin(2wt)
	// w = 2*pi*f 
	// f is the frequency of the gate
	vec2 A_lr = calculateCogVector(cog, legpos[LEFT_FRONT], legpos[LEFT_REAR]);
	vec2 A_fb = calculateCogVector(cog, legpos[LEFT_FRONT], legpos[RIGHT_FRONT]);
	// TODO: move into a setup func
	amp_LeftRight = 0.5;
	amp_FrontBack = 0.1;
	float f = 1.0 / cycleTime;
	float w = M_2_PI * f;

	// precompute these factors
	float m_lr = sin(w*t + M_PI_2) * amp_LeftRight;
	float m_fb = sin(2*w*t) * amp_FrontBack;

	cog = cog + (A_lr*m_lr) + (A_fb*m_fb);
	return cog;
}

vec2 GaitEngine::calculateCogVector(vec2 cog, vec2 leg_a, vec2 leg_b) {
	// make relative to center
	leg_a.sub(cog);
	leg_b.sub(cog);
	// distance to a
	float dist_a_sq = leg_a.lenSq();
	float dist_a = sqrt(dist_a_sq);
	// distance to b
	float dist_b_sq = leg_b.lenSq();
	float dist_b = sqrt(dist_b_sq);
	// distance between a and b = c
	vec2 c = leg_b;
	c.sub(leg_a);
	float dist_c_sq = c.lenSq();
	float dist_c = sqrt(dist_c_sq);
	// law of cosines
	float ang_C = acos((dist_a_sq + dist_a_sq - dist_c_sq) / 2*dist_a*dist_b);
	float ang_A = acos((dist_b_sq + dist_c_sq - dist_a_sq) / 2*dist_b*dist_c);
	ang_C = ang_C * 0.5;
	float ang_B = M_PI - ang_A - ang_B;
	// vector length
	float dist_cc = dist_a * sin(ang_C) / sin(ang_B);
	// adjust length
	c.scale(dist_cc/dist_c);
	// adding vector to a will give us the vector we seek
	c.add(leg_a);
	return c;
}

vec2 GaitEngine::calculateIntersection(vec2 rf, vec2 lf, vec2 lr, vec2 rr) {
	// line rf lr
	float sloap_a = (rf.x-lr.x) / (rf.y - lr.y);
	// line lf rr
	float sloap_b = (lf.x-rr.x) / (lf.y - rr.y);

	vec2 intersect;
	intersect.x = ((lf.y-sloap_b*lf.x) - (rf.y - sloap_a*rf.x)) / (sloap_b - sloap_a);
	intersect.y = sloap_a*intersect.x + (rf.y - sloap_a*rf.x);
	return intersect;
}
