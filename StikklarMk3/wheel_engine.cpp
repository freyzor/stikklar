#include "wheel_engine.h"

WheelEngine::WheelEngine() {
	// configure the bioloid interpolating servos
	_controller.setup(4);

	setupServoSet(WE_RF_TURN, WE_RF_WHEEL, RF_COXA, RF_WHEEL, RF_TURN_SIGN, RF_WHEEL_SIGN, RF_COXA_WHEEL_NEUTRAL);
	setupServoSet(WE_LF_TURN, WE_LF_WHEEL, LF_COXA, LF_WHEEL, LF_TURN_SIGN, LF_WHEEL_SIGN, LF_COXA_WHEEL_NEUTRAL);
	setupServoSet(WE_LR_TURN, WE_LR_WHEEL, LR_COXA, LR_WHEEL, LR_TURN_SIGN, LR_WHEEL_SIGN, LR_COXA_WHEEL_NEUTRAL);
	setupServoSet(WE_RR_TURN, WE_RR_WHEEL, RR_COXA, RR_WHEEL, RR_TURN_SIGN, RR_WHEEL_SIGN, RR_COXA_WHEEL_NEUTRAL);
}

void WheelEngine::setupServoSet(
		int turnIndex, int wheelIndex, 
		int turnId, int wheelId, 
		char turnSign, char wheelSign,
		int neutralValue)
{
	_servos[turnIndex] = turnId;
	_servos[wheelIndex] = wheelId;
	_signs[turnIndex] = turnSign;
	_signs[wheelIndex] = wheelSign;
	_neutral[turnIndex] = neutralValue;
	_controller.setId(turnIndex, turnId);
}

void WheelEngine::doUpdate() {
	if (steering == 0) {
		updateServos(0, 0, speed, speed, speed, speed);
		return;
	}
	// solve outward rear wheel
	float steeringRadian = MIN_ANGLE_RAD*abs(steering);
	// front radius
	float out_front_radius = WHEEL_X_LENGTH / tan(steeringRadian);
	float in_front_radius = out_front_radius - WHEEL_Y_LENGTH;

	float in_rear_radian = atan(WHEEL_X_LENGTH / in_front_radius);
	// rear radius
	float out_rear_radius = WHEEL_X_LENGTH / sin(steeringRadian);
	float in_rear_radius = WHEEL_X_LENGTH / sin(in_rear_radian);

	int in_rear_steering = (in_rear_radian / MIN_ANGLE_RAD) + 0.5;

	// calculate speed for each wheel
	// scale based on the outer rear wheel, always the fastest moving wheel.
	int out_rear_speed = speed;
	int out_front_speed = (speed * (out_front_radius/out_rear_radius) + 0.5);
	int in_rear_speed = (speed * (in_rear_radius/out_rear_radius) + 0.5);
	int in_front_speed = (speed * (in_front_radius/out_rear_radius) + 0.5);

	if (steering > 0) {
		updateServos(in_rear_steering, steering, in_front_speed, out_front_speed, out_rear_speed, in_rear_speed);
	} else {
		updateServos(steering, -in_rear_steering, out_front_speed, in_front_speed, in_rear_speed, out_rear_speed);
	} 
}

void WheelEngine::update() {
	//if our previous interpolation is complete, recompute the IK
	if(_controller.interpolating == 0){
		doUpdate();
		_controller.interpolateSetup(98);
	}

	// update joints
	_controller.interpolateStep();	
}

void WheelEngine::updateServos(
		int r_angle, int l_angle, 
		int rf_speed, int lf_speed, int lr_speed, int rr_speed) 
{
	_controller.setNextPose(_servos[WE_RF_TURN], _neutral[WE_RF_TURN]);
	_controller.setNextPose(_servos[WE_LF_TURN], _neutral[WE_LF_TURN]);
	// steering wheel angles set
	_controller.setNextPose(_servos[WE_LR_TURN], _neutral[WE_LR_TURN] + (_signs[WE_LR_TURN] * l_angle));
	_controller.setNextPose(_servos[WE_RR_TURN], _neutral[WE_RR_TURN] + (_signs[WE_RR_TURN] * r_angle));
	// set the motor speeds
	this->writeWheelSpeed(rf_speed, lf_speed, lr_speed, rr_speed);
}

unsigned int WheelEngine::convertSpeedToAX(int speed) {
	if (speed < 0) {
		// negative speed is 1024 + speed
		return -speed + 1024;
	} else {
		// positive speed is 0 - 1023
		return speed;
	}
}

void WheelEngine::writeWheelSpeed(int rf_speed, int lf_speed, int lr_speed, int rr_speed) {
	unsigned int temp;
	// convert from signed to a compliant unsigned range
	unsigned int speed[4] = {
		convertSpeedToAX(rf_speed * _signs[WE_RF_WHEEL]),
		convertSpeedToAX(lf_speed * _signs[WE_LF_WHEEL]),
		convertSpeedToAX(lr_speed * _signs[WE_LR_WHEEL]),
		convertSpeedToAX(rr_speed * _signs[WE_RR_WHEEL])
	};
	int length = 16; // 4 + 4 * 3 (id + pos(2byte))
    int checksum = 254 + length + AX_SYNC_WRITE + 2 + AX_GOAL_SPEED_L;

    setTXall();
    ax12write(0xFF);
    ax12write(0xFF);
    ax12write(0xFE);
    ax12write(length); 
    ax12write(AX_SYNC_WRITE);
    ax12write(AX_GOAL_SPEED_L);
    ax12write(2);
    for(int i=0; i<4; i++)
    {
        temp = speed[i];
        checksum += (temp&0xff) + (temp>>8) + _servos[i+4];
        ax12write(_servos[i+4]);
        ax12write(temp&0xff);
        ax12write(temp>>8);
    }
    ax12write(0xff - (checksum % 256));
    setRX(0);
}

void WheelEngine::writeWheelMode() {
    for (int i=4; i < 8; i++) {
    	ax12SetRegister2(_servos[i], AX_CW_ANGLE_LIMIT_L, 0);
    	ax12SetRegister2(_servos[i], AX_CCW_ANGLE_LIMIT_L, 0);
    }
}

void WheelEngine::readPose() {
	_controller.readPose();
}