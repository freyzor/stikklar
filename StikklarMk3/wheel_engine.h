#ifndef _WHEEL_ENGINE
#define _WHEEL_ENGINE

#include <math.h>
#include <BioloidController.h>
#include "config.h"

// leg assignments
#define WE_RF  0
#define WE_LF  1
#define WE_LR  2
#define WE_RR  3
// internal enums for each servo data index in internal arrays
#define WE_RF_TURN  0
#define WE_LF_TURN  1
#define WE_LR_TURN  2
#define WE_RR_TURN  3
#define WE_RF_WHEEL 4
#define WE_LF_WHEEL 5
#define WE_LR_WHEEL 6
#define WE_RR_WHEEL 7

// initialize as DONE as a signal that normal gait can proceed
// The quadrants is always RF, lf, lr, rr going CCW from RF
class WheelEngine {
public:
	WheelEngine() {};
    void setupContoller();
    void setBioloidController(BioloidController* bioloidController);
    void update();
    void readPose();
    void writeWheelMode();
    int steering;
    int speed;
private:
	void doUpdate();
    void updateServos(int r_angle, int l_angle, int rf_speed, int lf_speed, int lr_speed, int rr_speed);
    void writeWheelSpeed(int rf_speed, int lf_speed, int lr_speed, int rr_speed);

    unsigned int convertSpeedToAX(int speed) ;
	BioloidController* controller;
	static const unsigned char servoIds[8];
	static const char servoSigns[8];
	static const int servoNeutral[4]; // we only need the turn wheel neutral values
};


#endif // _WHEEL_ENGINE