#ifndef _CONFIG_H_
#define _CONFIG_H_

#define AX_SENSOR 100

#define AX_SERVO_COUNT	18

// NUKE config
#define LEG_COUNT   4

// the quadrants of the robot
// right front is Q1 where x and y are positive
// proceed CCW from there
#define RIGHT_FRONT    0
#define LEFT_FRONT     1
#define LEFT_REAR      2
#define RIGHT_REAR     3

// Body
// We assume 4 legs are on the corners of a box defined by X_COXA x Y_COXA
//
#define X_COXA      100  // MM between front and back legs /2
#define Y_COXA      100  // MM between front/back legs /2

// Legs
#define L_COXA      68  // MM distance from coxa servo to femur servo
#define L_FEMUR     82 // MM distance from femur servo to tibia servo
#define L_TIBIA     166 // MM distance from tibia servo to foot

#define DEFAULT_ENDPOINT_X 70  // front/back
#define DEFAULT_ENDPOINT_Y 70  // right/left
#define DEFAULT_ENDPOINT_Z 160 // down/up

#define LEG_LIFT_HIGHT 60

#define COG_DAMP_INCREASE_RATIO 0.125
#define COG_DAMP_DECREASE_RATIO 0.25

// Servo IDs
#define RF_COXA 1
#define RF_FEMUR 2
#define RF_TIBIA 3

#define RR_COXA 5
#define RR_FEMUR 6
#define RR_TIBIA 7

#define LF_COXA 9
#define LF_FEMUR 10
#define LF_TIBIA 11

#define LR_COXA 13
#define LR_FEMUR 14
#define LR_TIBIA 15

#define RF_COXA_SIGN 	1
#define RF_FEMUR_SIGN 	1
#define RF_TIBIA_SIGN 	-1

#define RR_COXA_SIGN 	-1
#define RR_FEMUR_SIGN 	1
#define RR_TIBIA_SIGN 	-1

#define LF_COXA_SIGN 	-1
#define LF_FEMUR_SIGN 	1
#define LF_TIBIA_SIGN 	-1

#define LR_COXA_SIGN 	1
#define LR_FEMUR_SIGN 	1
#define LR_TIBIA_SIGN 	-1


// Wheel Engine servo IDs
#define RF_WHEEL 	4
#define LF_WHEEL 	12
#define LR_WHEEL 	16
#define RR_WHEEL 	8

#define RF_WHEEL_SIGN 	-1
#define LF_WHEEL_SIGN 	1
#define LR_WHEEL_SIGN 	-1
#define RR_WHEEL_SIGN 	1

#define RF_TURN_SIGN 	1
#define LF_TURN_SIGN 	1
#define LR_TURN_SIGN 	1
#define RR_TURN_SIGN 	1

#define MIN_ANGLE_DEG		0.29296875
#define MIN_ANGLE_RAD		0.005113269292
#define WHEEL_X_LENGTH 		X_COXA
#define WHEEL_Y_LENGTH 		Y_COXA * 2

#define RF_COXA_WHEEL_NEUTRAL 666	// 1
#define LF_COXA_WHEEL_NEUTRAL 358	// 9
#define LR_COXA_WHEEL_NEUTRAL 666	// 13
#define RR_COXA_WHEEL_NEUTRAL 358	// 5

#define PAN_SERVO_ID	17
#define TILT_SERVO_ID	18

#define MAX_GAIT_STRIDE  150
#define MAX_GAIT_ROTATION 0.5

#define log(x) Serial.print(x)
#define logln(x) Serial.println(x)
#define logvec(n, v) log(n); log(": ("); log(v.x); log(", "); log(v.y); log(", "); log(v.z); logln(")");
#define logvec2(n, v) log(n); log(": ("); log(v.x); log(", "); log(v.y); logln(")");
#define logval(n, v) log(n); log(": "); logln(v);

#define dlogstart(n) log("#"); log(n); log(","); log(millis());
#define dlogend() logln("");
#define dlogvec2(v) log(","); log(v.x); log(","); log(v.y);
#define dlog(d) log(","); log(d);

//#define DEBUG

#ifdef DEBUG
	#define debug_msg(msg) log(millis()); log(": "); logln(msg);
#else
	#define debug_msg(msg)
#endif

#endif