// create by Freyr Magnusson
// 2014-04-19
// derived from autogenerated code from NUKE/PyPose

#include <ax12.h>
#include <BioloidController.h>
#include <Commander.h>
#include <FiniteStateMachine.h>
#include <vmath.h>
#include <Wire.h>

#include "gait_engine.h"
#include "wheel_engine.h"
#include "poses.h"

Commander command;
WheelEngine wheelEngine;
GaitEngine gaitEngine;


State DrivingState = State(enterDriving, updateDriving, noop);
State WalkingState = State(enterWalking, updateWalking, noop);
State GotoWalkingState = State(enterGotoWalkingState, updateGotoWalkingState, noop);
State GotoDrivingState = State(enterGotoDrivingState, updateGotoDrivingState, noop);

FSM fsm = FSM(WalkingState);

void noop() {}

// ************ Driving Mode *************
void enterDriving() {
    debug_msg("enter Driving State");
    wheelEngine.readPose();
    wheelEngine.writeWheelMode();
}

void updateDriving() {
    wheelEngine.update();
}

// ********** Walking Mode **************
void enterWalking() {
    debug_msg("enter Walking State");
    gaitEngine.gaitSelect(RIPPLE_SMOOTH);
}

void updateWalking() {
    gaitEngine.update();
}

void exitWalking() {
    debug_msg("exit Driving State");
}

// ******* Goto Driving transision state **********
void enterGotoDrivingState() {
    debug_msg("enter Goto Driving State");
    // we pick the point right under the coxa axle, 3sec should give two walk cycles
    gaitEngine.setStepToTarget(1, 1, DEFAULT_ENDPOINT_Z, 3000);
    gaitEngine.gaitSelect(RIPPLE_STEP_TO);
    debug_msg("driving goto set");
}

void updateGotoDrivingState() {
    if ( !gaitEngine.isSteppingTo() ) {
        debug_msg("Done stepping into drive position");
        gaitEngine.doPose(WHEEL_MODE_MIDDLE, 2000);
        debug_msg("Driving stance achieved");
        fsm.transitionTo(DrivingState);
    } else {
        gaitEngine.update();
    }
}

// ******* Goto Driving transision state **********
void enterGotoWalkingState() {
    debug_msg("enter Goto Walking State");
    gaitEngine.readPose();
    gaitEngine.doPose(WHEEL_CRAB_MIDDLE, 2000);
    debug_msg("Driving crab stance achieved");
    // go to the default walking stance, 3sec should give two walk cycles
    gaitEngine.setStepToTarget(DEFAULT_ENDPOINT_X, DEFAULT_ENDPOINT_Y, DEFAULT_ENDPOINT_Z, 3000);
    gaitEngine.gaitSelect(RIPPLE_STEP_TO);
    debug_msg("Walking goto set");
}

void updateGotoWalkingState() {
    if ( !gaitEngine.isSteppingTo() ) {
        debug_msg("Done stepping into default position");
        fsm.transitionTo(WalkingState);
    } else {
        gaitEngine.update();
    }
}

// The Commander protocol has values of -100 to 100, x/y/z speed are in mm/s
// To go faster than 100mm/s, we can use this speedMultiplier
float speedMultiplier;

void setup(){
    // set user LED as output
    pinMode(0, OUTPUT);
    
    ax12Init(1000000l);
    // setup IK
    gaitEngine.setupIK();
    gaitEngine.gaitSelect(RIPPLE_SMOOTH);
    // setup serial for usage with the Commander
    command.begin(38400);
  
    // wait, then check the voltage (LiPO safety)
    delay (2000);

    float voltage = (ax12GetRegister (1, AX_PRESENT_VOLTAGE, 1)) / 10.0;
    Serial.println ("== Stikklar Mk3 ==");
    Serial.print ("System Voltage: ");
    Serial.print (voltage);
    Serial.println (" volts.");
    if (voltage < 10.0){
        Serial.println ("WARNING: voltage to low entering eternal sleep!");
        while(1);
    }

    gaitEngine.readPose();
    gaitEngine.slowStart(2000);

    speedMultiplier = 1;
}

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
        // gaitEngine.centerOfGravityOffset.x = -(int)command.lookV;
        gaitEngine.bodyPos.x = -(int)command.lookV;
        // gaitEngine.centerOfGravityOffset.y = -(int)command.lookH;
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
    gaitEngine.Xspeed = speedMultiplier*command.walkV;
    if((command.buttons&BUT_LT) > 0){
        gaitEngine.Yspeed = (speedMultiplier*command.walkH)/2;
        gaitEngine.Rspeed = 0.0;
    } else {
        gaitEngine.Rspeed = -(speedMultiplier*command.walkH)/250.0;
        gaitEngine.Yspeed = 0;
    }
}

void setGaitMode(Commander &command) {
    if(command.buttons&BUT_R1) { 
        gaitEngine.gaitSelect( RIPPLE_SMOOTH );
        speedMultiplier=1;

    } else if(command.buttons&BUT_R2) {
        gaitEngine.gaitSelect( RIPPLE );
        speedMultiplier=1;

    } else if(command.buttons&BUT_R3) {
        gaitEngine.gaitSelect( RIPPLE_GEO );
        speedMultiplier=1;

    } else if(command.buttons&BUT_L4) { 
        gaitEngine.gaitSelect( AMBLE_SMOOTH );
        speedMultiplier=2;

    } else if(command.buttons&BUT_L5) {
        gaitEngine.gaitSelect( AMBLE );
        speedMultiplier=2;
    }
}

void setWheelMovement(Commander &command) {
    // max 1024 => speed 125*8 => 500
    wheelEngine.speed = command.walkV * 8;
    // max steer angle => 154, 125
    wheelEngine.steering = command.lookH;
}

void setSpeechCommands(Commander &command) {
    // if (command.buttons & BUT_R3) {
    //     speech.say("What do you want?");
    // }
}

void processCommands() {
    // take commands
    if(command.ReadMsgs() == 0)
        return;
    // toggle LED
    digitalWrite(0,HIGH-digitalRead(0));
    setSpeechCommands(command);
    // set speeds
    if ( fsm.isInState(WalkingState) ) {
        if (command.buttons & BUT_L6) {
            // goto driving mode
            fsm.transitionTo(GotoDrivingState);
        } else {
            setGaitMode(command);
            setWalkMovement(command);
            setCenterOfGravityOffset(command);
            //setBodyRotation(command);
        }
    } else if ( fsm.isInState(DrivingState) ) {
        if (command.buttons & BUT_L6) {
            // goto walk mode
            fsm.transitionTo(GotoWalkingState);
        } else {
            setWheelMovement(command);
        }
    }
}

void processSensors(){
  // read i2c sensors
  // SRF10 sonar
  // MiniIMU 9 DOF orientation
}

void loop(){
    processCommands();
    //processSensors();

    fsm.update();
}

