#include "turret_engine.h"
#include "config.h"

#define DEGREES_2_DX	3.41333

TurretEngine::TurretEngine():
	pan(0), 
	tilt(0) {};

void TurretEngine::setBioloidController(BioloidController* bioloidController) {
	controller = bioloidController;
}

void TurretEngine::setupContoller() {
	// setup the bioloid, we add the turre to the current setup
	// this is meant to add them to the back of the active main engine
	// the turret update will happens as a biproduct of other engine update
	int previousPoseSize = controller->poseSize;
	controller->poseSize = previousPoseSize + 2;
	controller->setId(previousPoseSize,     PAN_SERVO_ID);
	controller->setId(previousPoseSize + 1, TILT_SERVO_ID);
	ax12SetRegister2(PAN_SERVO_ID, AX_CW_ANGLE_LIMIT_L, 0);
    ax12SetRegister2(PAN_SERVO_ID, AX_CCW_ANGLE_LIMIT_L, 1023);
    ax12SetRegister2(TILT_SERVO_ID, AX_CW_ANGLE_LIMIT_L, 0);
    ax12SetRegister2(TILT_SERVO_ID, AX_CCW_ANGLE_LIMIT_L, 1023);
}

void TurretEngine::updateServos() {
	int panValue = PAN_CENTER - int((DEGREES_2_DX * float(pan)) + 0.5);
	if (panValue < MIN_PAN) { panValue = MIN_PAN; }
	else if (panValue > MAX_PAN) { panValue = MAX_PAN; }

	int tiltValue = TILT_CENTER + int((DEGREES_2_DX * float(tilt)) + 0.5);
	if (tiltValue < MIN_TILT) { tiltValue = MIN_TILT; }
	else if (tiltValue > MAX_TILT) { tiltValue = MAX_TILT; }

	controller->setNextPose(PAN_SERVO_ID, panValue);
	controller->setNextPose(TILT_SERVO_ID, tiltValue);
}

void TurretEngine::update() {
	// if our previous interpolation is complete, recompute the IK
	if(controller->interpolating == 0){
		updateServos();
		controller->interpolateSetup(TURRET_TRANSITION_TIME);
	}

	// update joints
	controller->interpolateStep();
}