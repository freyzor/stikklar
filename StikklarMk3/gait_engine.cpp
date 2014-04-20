#include "gait_engine.h"



// int signedValue(int n, int val) {
//   if (signs[n-1]){
//     return val;
//   } else {
//     return -val;
//   }
// }
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

	gaitGen = &GaitEngine::DefaultGaitGen;
	gaitSetup = &GaitEngine::DefaultGaitSetup;

	currentGait = -1;
}

void GaitEngine::setEndpoints(int x, int y, int z){
    endpoints[RIGHT_FRONT].x = x;
    endpoints[RIGHT_FRONT].y = y;
    endpoints[RIGHT_FRONT].z = z;

    endpoints[RIGHT_REAR].x = -x;
    endpoints[RIGHT_REAR].y = y;
    endpoints[RIGHT_REAR].z = z;

    endpoints[LEFT_FRONT].x = x;
    endpoints[LEFT_FRONT].y = -y;
    endpoints[LEFT_FRONT].z = z;

    endpoints[LEFT_REAR].x = -x;
    endpoints[LEFT_REAR].y = -y;
    endpoints[LEFT_REAR].z = z;
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

    setEndpoints(
    	DEFAULT_ENDPOINT_X, 
    	DEFAULT_ENDPOINT_Y, 
    	DEFAULT_ENDPOINT_Z
    );

    // default 33
    liftHeight = 60; 
    stepsInCycle = 1;
    step = 0;
}

void GaitEngine::setJointValue(int legId, int jointId, int rawValue){
    int servo = neutral(jointId) + signedValue(jointId, rawValue);
    if(servo < maxValue(jointId) && servo > minValue(jointId))
        _controller.setNextPose(jointId, servo);
    else
        printServoError(legId, jointId, servo);
}

void GaitEngine::doLegIK(int legId, int coxaId, int femurId, int tibiaId){
    ik_req_t gait = GaitGen(legId);
    // add the gate offset to the default endpoint
    ik_req_t endpoint = addPoints(endpoints[legId], gait);
    // transpose center of gravity
    endpoint = addPoints(endpoint, centerOfGravityOffset);
    // adjust for the body orientation
    ik_req_t req = bodyIK(
    	bodyRotX, bodyRotY, bodyRotZ,
        bodyPosY, bodyPosX,
    	endpoint.x, endpoint.y, endpoint.z, 
    	getXdisp(legId), getYdisp(legId), gait.r);

    // combine and adjust the result for the leg position
    endpoint = addPoints(endpoint, req);
    // store the endpoint currently assigned
    currentEndpoints[legId] = endpoint;
    // Translate endpoint to leg space for for IK solving
    adjustEndpointForLeg(legId, endpoint);
    // solve the IK
 //    if(currentGait == RIPPLE_STEP_TO) {
 //    	log(legId); logvec("EP", endpoint);
	// }
    ik_sol_t sol = legIK(endpoint.x, endpoint.y, endpoint.z);

    setJointValue(legId, coxaId, sol.coxa);
    setJointValue(legId, femurId, sol.femur);
    setJointValue(legId, tibiaId, sol.tibia);
}

void GaitEngine::doIK() {
    GaitSetup();

    doLegIK(RIGHT_FRONT, RF_COXA, RF_FEMUR, RF_TIBIA);
    // -X_COXA, -endpoint.x
    doLegIK(RIGHT_REAR, RR_COXA, RR_FEMUR, RR_TIBIA);
    // -Y_COXA, -endpoint.y
    doLegIK(LEFT_FRONT, LF_COXA, LF_FEMUR, LF_TIBIA);
    // -X_COXA, -endpoint.x, -Y_COXA, -endpoint.y
    doLegIK(LEFT_REAR, LR_COXA, LR_FEMUR, LR_TIBIA);

    step = (step+1) % stepsInCycle;

}

void GaitEngine::printServoError(int legId, int jointId, int servoValue){
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
	// reset the endpoints in case somebody messed with them
	setEndpoints(
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
	}else if(GaitType == RIPPLE_STEP_TO){
		setupStepToGait();
	}

	if(cycleTime == 0) {
		cycleTime = (stepsInCycle*tranTime)/1000.0;
	}
	step = 0;
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
	stepToVector.x = (nextEndPoint.x - currentEndpoints[RIGHT_FRONT].x)/float(cyclesToComplete) + 0.5;
	stepToVector.y = (nextEndPoint.y - currentEndpoints[RIGHT_FRONT].y)/float(cyclesToComplete) + 0.5;

	// log("Step to setup: counter: ");
	// log(stepToStepCounter);
	// log(" vec("); log(stepToVector.x); log(", "); log(stepToVector.y); logln(")"); 

	for (int legId=0; legId < LEG_COUNT; legId++) {
		// set gait offset as the the vector to from new target to current position
		gaits[legId].x = currentEndpoints[legId].x - xForLeg(legId, nextEndPoint.x);
		gaits[legId].y = currentEndpoints[legId].y - yForLeg(legId, nextEndPoint.y);
		gaits[legId].z = 0;
		// log(legId); logvec(" currEp", currentEndpoints[legId]);
		// log(legId); logvec(" gait", gaits[legId]);
		// delay(50);
	}
	setEndpoints(
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
ik_req_t GaitEngine::DefaultGaitGen(int leg){
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
ik_req_t GaitEngine::SmoothGaitGen(int leg) {
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
ik_req_t GaitEngine::StepToGaitGen(int legId){
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