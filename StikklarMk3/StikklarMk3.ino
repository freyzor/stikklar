/*
 * Auto-Generated by NUKE!
 *   http://arbotix.googlecode.com
 *
 * See http://code.google.com/p/arbotix/wiki/NukeIntro
 *   for details on usage.
 */

#include <ax12.h>
#include <BioloidController.h>
#include <Commander.h>
#include "nuke.h"

Commander command = Commander();
/* The Commander protocol has values of -100 to 100, x/y/z speed are in mm/s
 * To go faster than 100mm/s, we can use this speedMultiplier
 */
int speedMultiplier;

void setup(){
    // set user LED as output
    pinMode(0,OUTPUT);
    // setup IK
    setupIK();
    gaitSelect(RIPPLE);
    // setup serial for usage with the Commander
    command.begin(38400);

    // wait, then check the voltage (LiPO safety)
    delay (1000);
    float voltage = (ax12GetRegister (1, AX_PRESENT_VOLTAGE, 1)) / 10.0;
    Serial.print ("System Voltage: ");
    Serial.print (voltage);
    Serial.println (" volts.");
    if (voltage < 10.0){
        Serial.println ("WARNING: voltage to low entering eternal sleep!");
        while(1);
    }

    // stand up slowly
    bioloid.poseSize = 18;
    bioloid.readPose();
    doIK();
    bioloid.interpolateSetup(1000);
    while(bioloid.interpolating > 0){
        bioloid.interpolateStep();
        delay(3);
    }
    speedMultiplier = 1;
}

void processCommands(){
  // take commands
  if(command.ReadMsgs() > 0){
    digitalWrite(0,HIGH-digitalRead(0));
    // select gaits
    if(command.buttons&BUT_R1){ gaitSelect(RIPPLE_SMOOTH); speedMultiplier=1;}
    if(command.buttons&BUT_R2){ gaitSelect(RIPPLE); speedMultiplier=1;}
    if(command.buttons&BUT_L4){ gaitSelect(AMBLE_SMOOTH); speedMultiplier=2;}
    if(command.buttons&BUT_L5){ gaitSelect(AMBLE); speedMultiplier=2;}
    // set speeds
    Xspeed = speedMultiplier*command.walkV;
    if((command.buttons&BUT_LT) > 0){
      Yspeed = (speedMultiplier*command.walkH)/2;
      Rspeed = 0.0;
    } else {
      Rspeed = -(speedMultiplier*command.walkH)/250.0;
      Yspeed = 0;
    }
    bodyRotY = (((float)command.lookV))/250.0;
    if((command.buttons&BUT_RT) > 0) {
      bodyRotX = ((float)command.lookH)/250.0;
      bodyRotZ = 0.0;
    } else {
      bodyRotZ = ((float)command.lookH)/250.0;
      bodyRotX = 0.0;
    }
  } 
}

void processMotion(){
  // if our previous interpolation is complete, recompute the IK
  if(bioloid.interpolating == 0){
    doIK();
    bioloid.interpolateSetup(tranTime);
  }

  // update joints
  bioloid.interpolateStep();
}

void processSensors(){
  // read i2c sensors
  // SRF10 sonar
  // MiniIMU 9 DOF orientation
}

void loop(){
  processCommands();
  processSensors();
  processMotion();
}

