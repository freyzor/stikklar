#include <ax12.h>
#include <BioloidController.h>
#include <Arduino.h>
#include <math.h>
#include "nuke.h"

/* min and max positions for each servo */
int mins[] = {161, 162, 95, 0, 70, 161, 95, 0, 193, 162, 95, 0, 41, 162, 95, 0, 221, 162};
int maxs[] = {828, 850, 1005, 0, 980, 852, 1005, 0, 861, 864, 1005, 0, 940, 840, 1005, 0, 795, 858};
int neutrals[] = {508, 516, 754, 0, 364, 494, 731, 0, 510, 498, 748, 0, 669, 506, 745, 0, 218, 186};
int signs[] = {false, true, false, false, true, true, false, false, true, true, false, true, false, true, false, true, true, true};

int signedValue(int n, int val){
  if (signs[n-1]){
    return val;
  } else {
    return -val;
  }
};

/* IK Engine */
BioloidController bioloid = BioloidController(1000000);
ik_req_t endpoints[LEG_COUNT];
float bodyRotX = 0;             // body roll (rad)
float bodyRotY = 0;             // body pitch (rad)
float bodyRotZ = 0;             // body rotation (rad)
int bodyPosX = 0;               // body offset (mm)
int bodyPosY = 0;               // body offset (mm)
int Xspeed;                     // forward speed (mm/s)
int Yspeed;                     // sideward speed (mm/s)
float Rspeed;                   // rotation speed (rad/s)

/* Gait Engine */
int gaitLegNo[LEG_COUNT];       // order to step through legs
ik_req_t gaits[LEG_COUNT];      // gait engine output
int pushSteps;                  // how much of the cycle we are on the ground
int stepsInCycle;               // how many steps in this cycle
int step;                       // current step
int tranTime;
int liftHeight;
float cycleTime;                // cycle time in seconds (adjustment from speed to step-size)

/* Setup the starting positions of the legs. */
void setupIK(){
  // defaults: 68, 150, 124
  // int x = (int)(L_COXA + L_FEMUR) * 0.5;      // front displacement from coxa joint axis center
  // int y = (int)(L_COXA + L_FEMUR) * 0.5;      // right displacement from coxa joint axis center
  // int z = (int)(L_TIBIA * 0.7);               // downward displacement from coxa joint axis center
  int x = L_COXA;      // front displacement from coxa joint axis center
  int y = (int)(L_COXA + L_FEMUR);      // right displacement from coxa joint axis center
  int z = (int)(L_COXA + L_FEMUR) * 0.75;
 
  endpoints[RIGHT_FRONT].x = z; // L_COXA
  endpoints[RIGHT_FRONT].y = y; // L_COXA + L_FEMUR
  endpoints[RIGHT_FRONT].z = z; // L_TIBIA * 0.75

  endpoints[RIGHT_REAR].x = -x;
  endpoints[RIGHT_REAR].y = y;
  endpoints[RIGHT_REAR].z = z;

  endpoints[LEFT_FRONT].x = x;
  endpoints[LEFT_FRONT].y = -y;
  endpoints[LEFT_FRONT].z = z;

  endpoints[LEFT_REAR].x = -x;
  endpoints[LEFT_REAR].y = -y;
  endpoints[LEFT_REAR].z = z;

  // default 33
  liftHeight = (int)(L_TIBIA * 0.4); // L_TIBIA * 0.2
  stepsInCycle = 1;
  step = 0;
}

#include "gaits.h"

/* Convert radians to servo position offset. */
int radToServo(float rads){
  float val = rads * 195.56959407132f;
  return (int) val;
}

/* Body IK solver: compute where legs should be. */
ik_req_t bodyIK(int X, int Y, int Z, int Xdisp, int Ydisp, float Zrot){
    ik_req_t ans;

    float cosB = cos(bodyRotX);
    float sinB = sin(bodyRotX);
    float cosG = cos(bodyRotY);
    float sinG = sin(bodyRotY);
    float cosA = cos(bodyRotZ+Zrot);
    float sinA = sin(bodyRotZ+Zrot);

    int totalX = X + Xdisp + bodyPosX;
    int totalY = Y + Ydisp + bodyPosY;

    ans.x = totalX - int(totalX*cosG*cosA + totalY*sinB*sinG*cosA + Z*cosB*sinG*cosA - totalY*cosB*sinA + Z*sinB*sinA) + bodyPosX;
    ans.y = totalY - int(totalX*cosG*sinA + totalY*sinB*sinG*sinA + Z*cosB*sinG*sinA + totalY*cosB*cosA - Z*sinB*cosA) + bodyPosY;
    ans.z = Z - int(-totalX*sinG + totalY*sinB*cosG + Z*cosB*cosG);

    return ans;
}

/* Simple 3dof leg solver. X,Y,Z are the length from the Coxa rotate to the endpoint. */
ik_sol_t legIK(int X, int Y, int Z){
    ik_sol_t ans;

    // first, make this a 2DOF problem... by solving coxa
    ans.coxa = radToServo(atan2(X,Y));
    long trueX = sqrt(sq((long)X)+sq((long)Y)) - L_COXA;
    long im = sqrt(sq((long)trueX)+sq((long)Z));    // length of imaginary leg

    // get femur angle above horizon...
    float q1 = -atan2(Z,trueX);
    long d1 = sq(L_FEMUR)-sq(L_TIBIA)+sq(im);
    long d2 = 2*L_FEMUR*im;
    float q2 = acos((float)d1/(float)d2);
    ans.femur = radToServo(q1+q2);

    // and tibia angle from femur...
    d1 = sq(L_FEMUR)-sq(im)+sq(L_TIBIA);
    d2 = 2*L_TIBIA*L_FEMUR;
    ans.tibia = radToServo(acos((float)d1/(float)d2)-1.57);

    return ans;
}

ik_req_t addPoints(ik_req_t &a, ik_req_t &b)
{
    ik_req_t c;
    c.x = a.x + b.x;
    c.y = a.y + b.y;
    c.z = a.z + b.z;
    return c;
}

void adjustEndpointForLeg(int legId, ik_req_t &endpoint){
    if (legId == RIGHT_REAR){
        endpoint.x = -endpoint.x;
    } else if (legId == LEFT_FRONT){
        endpoint.y = -endpoint.y;
    } else if (legId == LEFT_REAR) {
        endpoint.x = -endpoint.x;
        endpoint.y = -endpoint.y;
    }
}

int getXdisp(int legId){
    // flip the rear legs
    if (legId == RIGHT_REAR || legId == LEFT_REAR)
        return -X_COXA;
    else
        return X_COXA;
}

int getYdisp(int legId){
    // flip the left legs
    if (legId == LEFT_FRONT || legId == LEFT_REAR)
        return -Y_COXA;
    else
        return Y_COXA;
}

void doLegIK(const char *legName, int legId, int coxaId, int femurId, int tibiaId){
    ik_req_t gait = gaitGen(legId);
    // add the gate offset to the default endpoint
    ik_req_t endpoint = addPoints(endpoints[legId], gait);
    // adjust for the body orientation
    ik_req_t req = bodyIK(endpoint.x, endpoint.y, endpoint.z, getXdisp(legId), getYdisp(legId), gait.r);
    // combine and adjust the result for the leg position
    endpoint = addPoints(endpoint, req);
    adjustEndpointForLeg(legId, endpoint);
    // solve the IK
    ik_sol_t sol = legIK(endpoint.x, endpoint.y, endpoint.z);

    int servo = neutral(coxaId) + signedValue(coxaId, sol.coxa);
    if(servo < maxValue(coxaId) && servo > minValue(coxaId))
        bioloid.setNextPose(coxaId, servo);
    else{
        Serial.print(legName);
        Serial.print(" COXA FAIL: ");
        Serial.println(servo);
    }
    servo = neutral(femurId) + signedValue(femurId, sol.femur);
    if(servo < maxValue(femurId) && servo > minValue(femurId))
        bioloid.setNextPose(femurId, servo);
    else{
        Serial.print(legName);
        Serial.print(" FEMUR FAIL: ");
        Serial.println(servo);
    }
    servo = neutral(tibiaId) + signedValue(tibiaId, sol.tibia);
    if(servo < maxValue(tibiaId) && servo > minValue(tibiaId))
        bioloid.setNextPose(tibiaId, servo);
    else{
        Serial.print(legName);
        Serial.print(" TIBIA FAIL: ");
        Serial.println(servo);
    }
}

void doIK(){
    gaitSetup();

    doLegIK("RIGHT_FRONT", RIGHT_FRONT, RF_COXA, RF_FEMUR, RF_TIBIA);
    // -X_COXA, -endpoint.x
    doLegIK("RIGHT_REAR", RIGHT_REAR, RR_COXA, RR_FEMUR, RR_TIBIA);
    // -Y_COXA, -endpoint.y
    doLegIK("LEFT_FRONT", LEFT_FRONT, LF_COXA, LF_FEMUR, LF_TIBIA);
    // -X_COXA, -endpoint.x, -Y_COXA, -endpoint.y
    doLegIK("LEFT_REAR", LEFT_REAR, LR_COXA, LR_FEMUR, LR_TIBIA);

    step = (step+1)%stepsInCycle;

}

// void doIK(){
//     int servo;
//     ik_req_t req, gait;
//     ik_sol_t sol;

//     gaitSetup();

//     doLegIK("RIGHT_FRONT", RIGHT_FRONT, RF_COXA, RF_FEMUR, RF_TIBIA);

//     // right front leg
//     gait = gaitGen(RIGHT_FRONT);
//     req = bodyIK(endpoints[RIGHT_FRONT].x+gait.x, endpoints[RIGHT_FRONT].y+gait.y, endpoints[RIGHT_FRONT].z+gait.z, X_COXA, Y_COXA, gait.r);
//     sol = legIK(endpoints[RIGHT_FRONT].x+req.x+gait.x,endpoints[RIGHT_FRONT].y+req.y+gait.y,endpoints[RIGHT_FRONT].z+req.z+gait.z);
//     servo = 508 - sol.coxa;
//     if(servo < maxs[RF_COXA-1] && servo > mins[RF_COXA-1])
//         bioloid.setNextPose(RF_COXA, servo);
//     else{
//         Serial.print("RF_COXA FAIL: ");
//         Serial.println(servo);
//     }
//     servo = 516 + sol.femur;
//     if(servo < maxs[RF_FEMUR-1] && servo > mins[RF_FEMUR-1])
//         bioloid.setNextPose(RF_FEMUR, servo);
//     else{
//         Serial.print("RF_FEMUR FAIL: ");
//         Serial.println(servo);
//     }
//     servo = 754 - sol.tibia;
//     if(servo < maxs[RF_TIBIA-1] && servo > mins[RF_TIBIA-1])
//         bioloid.setNextPose(RF_TIBIA, servo);
//     else{
//         Serial.print("RF_TIBIA FAIL: ");
//         Serial.println(servo);
//     }

//     // right rear leg
//     gait = gaitGen(RIGHT_REAR);
//     req = bodyIK(endpoints[RIGHT_REAR].x+gait.x,endpoints[RIGHT_REAR].y+gait.y, endpoints[RIGHT_REAR].z+gait.z, -X_COXA, Y_COXA, gait.r);
//     sol = legIK(-endpoints[RIGHT_REAR].x-req.x-gait.x,endpoints[RIGHT_REAR].y+req.y+gait.y,endpoints[RIGHT_REAR].z+req.z+gait.z);
//     servo = 364 + sol.coxa;
//     if(servo < maxs[RR_COXA-1] && servo > mins[RR_COXA-1])
//         bioloid.setNextPose(RR_COXA, servo);
//     else{
//         Serial.print("RR_COXA FAIL: ");
//         Serial.println(servo);
//     }
//     servo = 494 + sol.femur;
//     if(servo < maxs[RR_FEMUR-1] && servo > mins[RR_FEMUR-1])
//         bioloid.setNextPose(RR_FEMUR, servo);
//     else{
//         Serial.print("RR_FEMUR FAIL: ");
//         Serial.println(servo);
//     }
//     servo = 731 - sol.tibia;
//     if(servo < maxs[RR_TIBIA-1] && servo > mins[RR_TIBIA-1])
//         bioloid.setNextPose(RR_TIBIA, servo);
//     else{
//         Serial.print("RR_TIBIA FAIL: ");
//         Serial.println(servo);
//     }

//     // left front leg
//     gait = gaitGen(LEFT_FRONT);
//     req = bodyIK(endpoints[LEFT_FRONT].x+gait.x,endpoints[LEFT_FRONT].y+gait.y, endpoints[LEFT_FRONT].z+gait.z, X_COXA, -Y_COXA, gait.r);
//     sol = legIK(endpoints[LEFT_FRONT].x+req.x+gait.x,-endpoints[LEFT_FRONT].y-req.y-gait.y,endpoints[LEFT_FRONT].z+req.z+gait.z);
//     servo = 510 + sol.coxa;
//     if(servo < maxs[LF_COXA-1] && servo > mins[LF_COXA-1])
//         bioloid.setNextPose(LF_COXA, servo);
//     else{
//         Serial.print("LF_COXA FAIL: ");
//         Serial.println(servo);
//     }
//     servo = 498 + sol.femur;
//     if(servo < maxs[LF_FEMUR-1] && servo > mins[LF_FEMUR-1])
//         bioloid.setNextPose(LF_FEMUR, servo);
//     else{
//         Serial.print("LF_FEMUR FAIL: ");
//         Serial.println(servo);
//     }
//     servo = 748 - sol.tibia;
//     if(servo < maxs[LF_TIBIA-1] && servo > mins[LF_TIBIA-1])
//         bioloid.setNextPose(LF_TIBIA, servo);
//     else{
//         Serial.print("LF_TIBIA FAIL: ");
//         Serial.println(servo);
//     }

//     // left rear leg
//     gait = gaitGen(LEFT_REAR);
//     req = bodyIK(endpoints[LEFT_REAR].x+gait.x,endpoints[LEFT_REAR].y+gait.y, endpoints[LEFT_REAR].z+gait.z, -X_COXA, -Y_COXA, gait.r);
//     sol = legIK(-endpoints[LEFT_REAR].x-req.x-gait.x,-endpoints[LEFT_REAR].y-req.y-gait.y,endpoints[LEFT_REAR].z+req.z+gait.z);
//     servo = 669 - sol.coxa;
//     if(servo < maxs[LR_COXA-1] && servo > mins[LR_COXA-1])
//         bioloid.setNextPose(LR_COXA, servo);
//     else{
//         Serial.print("LR_COXA FAIL: ");
//         Serial.println(servo);
//     }
//     servo = 506 + sol.femur;
//     if(servo < maxs[LR_FEMUR-1] && servo > mins[LR_FEMUR-1])
//         bioloid.setNextPose(LR_FEMUR, servo);
//     else{
//         Serial.print("LR_FEMUR FAIL: ");
//         Serial.println(servo);
//     }
//     servo = 745 - sol.tibia;
//     if(servo < maxs[LR_TIBIA-1] && servo > mins[LR_TIBIA-1])
//         bioloid.setNextPose(LR_TIBIA, servo);
//     else{
//         Serial.print("LR_TIBIA FAIL: ");
//         Serial.println(servo);
//     }

//     step = (step+1)%stepsInCycle;

// }

