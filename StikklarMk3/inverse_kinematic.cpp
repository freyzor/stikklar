#include "inverse_kinematic.h"

// Convert radians to servo position offset
int radToServo(float rads){
  float val = rads * 195.56959407132f;
  return (int) val;
}

// Body IK solver: compute where legs should be.
ik_req_t bodyIK2(
    	float bodyRotX, float bodyRotY, float bodyRotZ,
    	int bodyPosX, int bodyPosY,
    	int X, int Y, int Z, 
    	int Xdisp, int Ydisp, float Zrot)
{
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

ik_req_t bodyIK(vec3 bodyRot, vec2 bodyPos, ik_req_t footPos, vec2 offset)
{
    ik_req_t ans;

    // TODO: this part can be computed once for all legs
    float cosB = cos(bodyRot.x);
    float sinB = sin(bodyRot.x);
    float cosG = cos(bodyRot.y);
    float sinG = sin(bodyRot.y);
    float cosA = cos(bodyRot.z + footPos.r);
    float sinA = sin(bodyRot.z + footPos.r);

    int totalX = footPos.x + offset.x + bodyPos.x;
    int totalY = footPos.y + offset.y + bodyPos.y;

    float t1 = totalX*cosG;
    float t2 = totalY*sinB*sinG;
    float t3 = footPos.z*cosB*sinG;
    float t4 = totalY*cosB;
    float t5 = footPos.z*sinB;

    ans.x = totalX - int(t1*cosA + t2*cosA + t3*cosA - t4*sinA + t5*sinA) + bodyPos.x;
    ans.y = totalY - int(t1*sinA + t2*sinA + t3*sinA + t4*cosA - t5*cosA) + bodyPos.y;
    ans.z = footPos.z - int(-totalX*sinG + totalY*sinB*cosG + footPos.z*cosB*cosG);

    return ans;
}

ik_sol_t legIK(ik_req_t footPos) {
    return legIK(footPos.x, footPos.y, footPos.z);
}

// Simple 3dof leg solver. X,Y,Z are the length from the Coxa rotate to the endpoint.
ik_sol_t legIK(int X, int Y, int Z) {
    // NOTE: we can solve this in squared space, should be faster
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
    adjustForQuadrant(legId, endpoint.x, endpoint.y);
}

void adjustForQuadrant(int quadId, int &x, int &y) {
    if (quadId == RIGHT_REAR){
        x = -x;
    } else if (quadId == LEFT_FRONT) {
        y = -y;
    } else if (quadId == LEFT_REAR) {
        x = -x;
        y = -y;
    }
}

void adjustForQuadrant(int quadId, float &x, float &y) {
    if (quadId == RIGHT_REAR){
        x = -x;
    } else if (quadId == LEFT_FRONT) {
        y = -y;
    } else if (quadId == LEFT_REAR) {
        x = -x;
        y = -y;
    }
}

vec2 getForQuadrant(int quadId, const vec2 &v) {
    vec2 vv = v;
    adjustForQuadrant(quadId, vv.x, vv.y);
    return vv;
}

int getXdisp(int legId) {
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

int xForLeg(int legId, int value) {
    // flip the rear legs
    if (legId == RIGHT_REAR || legId == LEFT_REAR)
        return -value;
    else
        return value;
}

int yForLeg(int legId, int value){
    // flip the left legs
    if (legId == LEFT_FRONT || legId == LEFT_REAR)
        return -value;
    else
        return value;
}