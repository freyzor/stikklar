#ifndef INVERSE_KINEMATIC_H
#define INVERSE_KINEMATIC_H

#include <math.h>
#include "config.h"

#define sq(x) x*x

// A leg position request (output of body calcs, input to simple 3dof solver).
typedef struct{
    int x;
    int y;
    int z;
    float r;
} ik_req_t;

// Servo ouptut values (output of 3dof leg solver).
typedef struct{
    int coxa;
    int femur;
    int tibia;
} ik_sol_t;

// Convert radians to servo position offset
int radToServo(float rads);

// Body IK solver: compute where legs should be.
ik_req_t bodyIK(
	float bodyRotX, float bodyRotY, float bodyRotZ,
	int bodyPosX, int bodyPosY,
	int X, int Y, int Z, 
	int Xdisp, int Ydisp, float Zrot);

// Simple 3dof leg solver. X,Y,Z are the length from the Coxa rotate to the endpoint.
ik_sol_t legIK(int X, int Y, int Z);

ik_req_t addPoints(ik_req_t &a, ik_req_t &b);

void adjustEndpointForLeg(int legId, ik_req_t &endpoint);

int getXdisp(int legId);
int getYdisp(int legId);
int xForLeg(int legId, int value);
int yForLeg(int legId, int value);

#endif