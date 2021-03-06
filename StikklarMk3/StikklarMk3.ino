// create by Freyr Magnusson
// 2014-04-19
// derived from autogenerated code from NUKE/PyPose

// Included Arduino libraries
#include <Wire.h>

// 3d party Arduino libraries
#include <FiniteStateMachine.h>
#include <L3G.h>
#include <LSM303.h>
#include <ProfileTimer.h>
#include <looper.h>

// Arbotix libraries
#include <ax12.h>
#include <BioloidController.h>
#include <Commander.h>

// freybotlib
#include <vmath.h>

// stikklar
#include "gait_engine.h"
#include "wheel_engine.h"
#include "turret_engine.h"
#include "poses.h"
#include "blackboard.h"
#include "sensors.h"
#include "minimu_ahrs.h"
#include "tones.h"

#define MIN_VOLTAGE 10.0

Commander command;
BioloidController bioloidController;
WheelEngine wheelEngine;
GaitEngine gaitEngine;
TurretEngine turretEngine;
MinIMU_AHRS imu;
looper scheduler;

void setupWalkMode() {
    // setup the bioloid dynamixel config 
    gaitEngine.setupContoller();
    // extend the setup with the turret controllers
    turretEngine.setupContoller();
    // initialize the current servo positions
    bioloidController.readPose();
}

void setupWheelMode() {
    // need to setup the controller every time we enter the driving mode
    wheelEngine.setupContoller();
    // extend the setup with the turret controllers
    turretEngine.setupContoller();
    // initialize the current servo positions
    bioloidController.readPose();
}

void updateGaitEngine() {
    // update joints
    bioloidController.interpolateStep();

    // if our previous interpolation is complete, recompute the IK
    if(bioloidController.interpolating == 0) {
        gaitEngine.update();
        turretEngine.updateServos();
        bioloidController.interpolateSetup(STD_TRANSITION);
    }
}

void updateWheelEngine() {
    // update joints
    bioloidController.interpolateStep();

    // if our previous interpolation is complete, recompute the IK
    if(bioloidController.interpolating == 0) {
        wheelEngine.update();
        turretEngine.updateServos();
        bioloidController.interpolateSetup(STD_TRANSITION);
    }    
}

// *********** Finite State Machine ************
// for transisions between Walking and Driving
State DrivingState = State(enterDriving, noop, exitDriving);
State WalkingState = State(enterWalking, noop, exitWalking);
State GotoWalkingState = State(enterGotoWalkingState, updateGotoWalkingState, noop);
State GotoDrivingState = State(enterGotoDrivingState, updateGotoDrivingState, noop);

FSM fsm = FSM(WalkingState);

void noop() {}

// ************ Driving Mode *************
void enterDriving() {
    debug_msg("enter Driving State");
    setupWheelMode();
    scheduler.addJob(updateWheelEngine, BIOLOID_FRAME_LENGTH);
}

void exitDriving() {
    debug_msg("exit Driving State");
    scheduler.removeJob(updateWheelEngine);
}

// ********** Walking Mode **************
void enterWalking() {
    debug_msg("enter Walking State");
    setupWalkMode();
    gaitEngine.gaitSelect(RIPPLE_GEO);
    scheduler.addJob(updateGaitEngine, BIOLOID_FRAME_LENGTH);
}

void exitWalking() {
    debug_msg("exit Walking State");
    scheduler.removeJob(updateGaitEngine);
}

// ******* Goto Driving transision state **********
void enterGotoDrivingState() {
    debug_msg("enter Goto Driving State");
    // we pick the point right under the coxa axle, 3sec should give two walk cycles
    gaitEngine.setStepToTarget(30, 30, DEFAULT_ENDPOINT_Z, 4000);
    gaitEngine.gaitSelect(RIPPLE_STEP_TO);
    scheduler.addJob(updateGaitEngine, BIOLOID_FRAME_LENGTH);
    debug_msg("driving goto set");
}

void updateGotoDrivingState() {
    if (gaitEngine.isContiouslySteppingTo()) return;

    scheduler.removeJob(updateGaitEngine);
    debug_msg("Done stepping into drive position");
    gaitEngine.cacheGaits();
    gaitEngine.doPose(WHEEL_MODE_MIDDLE, 2000);
    debug_msg("Driving stance achieved");
    fsm.transitionTo(DrivingState);
}

// ******* Goto Driving transision state **********
void enterGotoWalkingState() {
    debug_msg("enter Goto Walking State");
    setupWalkMode();
    // enter an intermediate pose
    // TODO: we could run a sequence to orchestrate this better
    gaitEngine.doPose(WHEEL_CRAB_MIDDLE, 2000);
    debug_msg("Driving crab stance achieved");

    gaitEngine.restoreCachedGaits();
    // go to the default walking stance, 3sec should give two walk cycles
    gaitEngine.setStepToTarget(DEFAULT_ENDPOINT_X, DEFAULT_ENDPOINT_Y, DEFAULT_ENDPOINT_Z, 4000);
    gaitEngine.gaitSelect(RIPPLE_STEP_TO);
    scheduler.addJob(updateGaitEngine, BIOLOID_FRAME_LENGTH);
    debug_msg("Walking goto set");
}

void updateGotoWalkingState() {
    if (gaitEngine.isContiouslySteppingTo()) return;

    scheduler.removeJob(updateGaitEngine);
    debug_msg("Done stepping into default position");
    fsm.transitionTo(WalkingState);
}

// *************** Commander Input processing ***********
void setBodyRotation(Commander &command){
    gaitEngine.bodyRot.y = (((float)command.lookV))/250.0;
    if((command.buttons&BUT_RT) > 0) {
        gaitEngine.bodyRot.x = ((float)command.lookH)/250.0;
        gaitEngine.bodyRot.z = 0.0;
    } else {
        gaitEngine.bodyRot.z = ((float)command.lookH)/250.0;
        gaitEngine.bodyRot.x = 0.0;
    }
}

void setCenterOfGravityOffset(Commander &command){
    // we invert the joystick values since we ned to shift the legs in the opposite direction to move the body correctly
    // can move upto move 125mm in each direction
    if((command.buttons&BUT_RT) == 0) {
        // move on the XY ground plane
        gaitEngine.bodyPos.x = -(int)command.lookV;
        gaitEngine.bodyPos.y = -(int)command.lookH;
        gaitEngine.centerOfGravityOffset.z = 0;
    } else {
        // move up or down
        gaitEngine.centerOfGravityOffset.x = 0;
        gaitEngine.centerOfGravityOffset.y = 0;
        gaitEngine.centerOfGravityOffset.z = (int)command.lookV;
    }
}

void setWalkMovement(Commander &command){
    // convert to -1,1 range
    float x = float(command.walkV)/125.0;
    float y = float(command.walkH)/125.0;
    float sqrtX = sqrt(x);
    float sqrtY = sqrt(y);
    float cycleScale = max(abs(x), abs(y));
    // 0 - 150 range
    gaitEngine.Xspeed = int(sqrtX*MAX_GAIT_STRIDE);
    if (x < 0.0) { gaitEngine.Xspeed = -gaitEngine.Xspeed; };

    if((command.buttons&BUT_LT) > 0) {
        gaitEngine.Yspeed = int(sqrtY*MAX_GAIT_STRIDE);
        gaitEngine.Rspeed = 0.0;
        if (y < 0.0) { gaitEngine.Yspeed = -gaitEngine.Yspeed; };

    } else {
        gaitEngine.Yspeed = 0;
        // range (0 - 0.5)
        gaitEngine.Rspeed = -y*MAX_GAIT_ROTATION;
    }
    gaitEngine.setPeriodMillis(int((1.0 / (cycleScale*0.9 + 0.1))*1000));
}

void setGaitMode(Commander &command) {
    if(command.buttons&BUT_R3) {
        gaitEngine.gaitSelect( RIPPLE_GEO );
    }
}

void setWheelMovement(Commander &command) {
    // max 1024 => speed 125*8 => 500
    wheelEngine.speed = command.walkV * 8;
    // max steer angle => 154, 125
    wheelEngine.steering = int((command.walkH * 0.88) + 0.5);
}

void setTurretMovement(Commander &command) {
    // pan (-150, 150) degrees
    turretEngine.pan = int((command.lookH * 1.2) + 0.5);
    // tilt (-90, 90) degrees
    turretEngine.tilt = int((command.lookV * 0.72) + 0.5);
}

void processCommands() {
    // take commands
    if(command.ReadMsgs() == 0)
        return;
    // toggle LED
    digitalWrite(0, HIGH-digitalRead(0));

    // set speeds
    if ( fsm.isInState(WalkingState) ) {
        if (command.buttons & BUT_L6) {
            // goto driving mode
            PlayTone(AX_SENSOR, TONE_C2);
            fsm.transitionTo(GotoDrivingState);
        } else {
            setGaitMode(command);
            setWalkMovement(command);
            //setCenterOfGravityOffset(command);
            //setBodyRotation(command);
            setTurretMovement(command);
        }
    } else if ( fsm.isInState(DrivingState) ) {
        if (command.buttons & BUT_L6) {
            // goto walk mode
            PlayTone(AX_SENSOR, TONE_A2);
            fsm.transitionTo(GotoWalkingState);
        } else {
            setWheelMovement(command);
            setTurretMovement(command);
        }
    }
}

// ************* Arduino **************

void setup() {
    // set user LED as output
    pinMode(0, OUTPUT);

    // configure bioloid controller and engines
    // the controller is set to hold all the servos
    // individual engines will then limit and set up the active set of servos needed
    bioloidController.setup(AX_SERVO_COUNT);
    gaitEngine.setBioloidController(&bioloidController);
    wheelEngine.setBioloidController(&bioloidController);
    turretEngine.setBioloidController(&bioloidController);

    // initialize the bioloid dynamixel bus communicationss
    ax12Init(1000000l);
    Wire.begin();
    TWBR = ((F_CPU / 400000) - 16) / 2;//set the I2C speed to 400KHz
    command.begin(38400);
    Serial.println("********* Stikklar *********");
    delay(2000);

    // setup serial for usage with the Commander
    
    float voltage = getVoltage();
    if (voltage < MIN_VOLTAGE) {
        emergencyShutdown(voltage);
        // stop everything
        while(1) {};
    }
    logval("Voltage", voltage);

    // power up beep
    PlayTone(AX_SENSOR, TONE_E2);

    imu.initialize();

    // calibration done beep
    PlayTone(AX_SENSOR, TONE_C2);

    // initialize walker and rise slowly
    setupWalkMode();
    turretEngine.updateServos();
    gaitEngine.setupIK();
    gaitEngine.gaitSelect(RIPPLE_GEO);
    gaitEngine.readPose();
    gaitEngine.slowStart(2000);

    // setup scheduler tasks
    // this should loop over all 18 servos in 1800ms
    //scheduler.addJob(monitorDynamixel, 100);
    scheduler.addJob(updateIMU, 20);
    scheduler.addJob(outputAHRS, 100);
    //scheduler.addJob(monitorVoltage, 1000);
    scheduler.addJob(processCommands, 50);
    scheduler.addJob(updateFSM, 166);
}

// used to halt all processes
bool doUpdates = true;

void loop(){
    if (doUpdates) {
        scheduler.scheduler();
    }
}

// **************** Jobs ***********************

void updateFSM() {
    fsm.update();
}

void updateIMU() {
    imu.update();
    imu.updateEulerAngles();
    blackboard.pitch = imu.pitch;
    blackboard.yaw = imu.yaw;
    blackboard.roll = imu.roll;
}

void monitorVoltage() {
    float voltage = getVoltage();
    if (voltage < MIN_VOLTAGE) {
        emergencyShutdown(voltage);
        doUpdates = false;
    }
}

char nextServoMonitorId = 1;
void monitorDynamixel() {
    if (readAxServoInfo(nextServoMonitorId)) {
        int error = blackboard.axError[nextServoMonitorId-1];
        if (error > 0) writeErrors(nextServoMonitorId, error);
    }
    nextServoMonitorId = nextServoMonitorId % AX_SERVO_COUNT;
    nextServoMonitorId++;
}

// ************* Utility ***********************
void writeErrors(int id, int error) {
    log("Error id="); log(id); 
    // ERR_VOLTAGE                 1
    if (error&ERR_VOLTAGE) log(" voltage");
    // ERR_ANGLE_LIMIT             2
    if (error&ERR_ANGLE_LIMIT) log(" angle");
    // ERR_OVERHEATING             4
    if (error&ERR_OVERHEATING) log(" heat");
    // ERR_RANGE                   8
    if (error&ERR_RANGE) log(" range");
    // ERR_CHECKSUM                16
    if (error&ERR_CHECKSUM) log(" checksum");
    // ERR_OVERLOAD                32
    if (error&ERR_OVERLOAD) log(" overload");
    // ERR_INSTRUCTION             64
    if (error&ERR_INSTRUCTION) log(" instruction");

    logln("");

    PlayTone(AX_SENSOR, TONE_Gb0);
}

float getVoltage() {
    // check the voltage (LiPO safety)
    return ax12GetRegister(AX_SENSOR, AX_PRESENT_VOLTAGE, 1) / 10.0;
}

void emergencyShutdown(float voltage) {
    Serial.println("We should really power down now");

    // power down all AX servo motors
    setTorque(false);
    Serial.print("Voltage: ");
    Serial.print(voltage);
    Serial.println("Warning! Voltage is below minimum safe LiPO values");
    Serial.println("All processes have been halted and servo torque disabled.");

    // warn the user
    PlayTone(AX_SENSOR, TONE_E0);
    delay(500);
    PlayTone(AX_SENSOR, TONE_C0);
}

void setTorque(bool enable) {
    byte torque = 0x00;
    if (enable) { torque = 0x01; };

    int length = 4 + (AX_SERVO_COUNT * 2);   // 3 = id + torque enable
    int checksum = 254 + length + AX_SYNC_WRITE + 2 + AX_TORQUE_ENABLE;
    setTXall();
    ax12write(0xFF);
    ax12write(0xFF);
    ax12write(0xFE);
    ax12write(length);
    ax12write(AX_SYNC_WRITE);
    ax12write(AX_TORQUE_ENABLE);
    ax12write(1);
    for(int id=1; id<=AX_SERVO_COUNT; id++)
    {
        checksum += torque + id;
        ax12write(id);
        ax12write(torque);
    } 
    ax12write(0xff - (checksum % 256));
    setRX(0);
}
