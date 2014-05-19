/*
This is adapted from MinIMU-9-Arduino-AHRS which included the following notice:

MinIMU-9-Arduino-AHRS
Pololu MinIMU-9 + Arduino AHRS (Attitude and Heading Reference System)

Copyright (c) 2011 Pololu Corporation.
http://www.pololu.com/

MinIMU-9-Arduino-AHRS is based on sf9domahrs by Doug Weibel and Jose Julio:
http://code.google.com/p/sf9domahrs/

sf9domahrs is based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose
Julio and Doug Weibel:
http://code.google.com/p/ardu-imu/

MinIMU-9-Arduino-AHRS is free software: you can redistribute it and/or modify it
under the terms of the GNU Lesser General Public License as published by the
Free Software Foundation, either version 3 of the License, or (at your option)
any later version.

MinIMU-9-Arduino-AHRS is distributed in the hope that it will be useful, but
WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for
more details.

You should have received a copy of the GNU Lesser General Public License along
with MinIMU-9-Arduino-AHRS. If not, see <http://www.gnu.org/licenses/>.

*/
#ifndef MINIMU_AHRS_H
#define MINIMU_AHRS_H

#include <Arduino.h>
#include <L3G.h>
#include <LSM303.h>

// LSM303 accelerometer: 8 g sensitivity
// 3.9 mg/digit; 1 g = 256
#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer 

#define ToRad(x) ((x)*0.01745329252)  // *pi/180

// L3G4200D gyro: 2000 dps full scale
// 70 mdps/digit; 1 dps = 0.07
#define Gyro_Gain_X 0.07 //X axis Gyro gain
#define Gyro_Gain_Y 0.07 //Y axis Gyro gain
#define Gyro_Gain_Z 0.07 //Z axis Gyro gain
#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) //Return the scaled ADC raw data of the gyro in radians for second

// LSM303 magnetometer calibration constants; use the Calibrate example from
// the Pololu LSM303 library to find the right values for your board
#define M_X_MIN -601
#define M_Y_MIN -671
#define M_Z_MIN -336
#define M_X_MAX 418
#define M_Y_MAX 344
#define M_Z_MAX 615

#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

#define INTGRATION_TIME	0.02

class MinIMU_AHRS {
public:
	MinIMU_AHRS();
	void initialize();
	void update();
	void updateEulerAngles();
	// Euler angles
	float roll;
	float pitch;
	float yaw;
private:
	void Compass_Heading();
	void Normalize();
	void Drift_correction();
	void Matrix_update();

	void Gyro_Init();
	void Read_Gyro();
	void Accel_Init();
	void Read_Accel();
	void Compass_Init();
	void Read_Compass();

	L3G gyro;
	LSM303 compass;
	float G_Dt;		// Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

	unsigned long timer_old;

	unsigned int counter;
	int gyro_x;
	int gyro_y;
	int gyro_z;
	int accel_x;
	int accel_y;
	int accel_z;
	int magnetom_x;
	int magnetom_y;
	int magnetom_z;
	float c_magnetom_x;
	float c_magnetom_y;
	float c_magnetom_z;
	float MAG_Heading;
	byte gyro_sat;
	int AN[6];						//array that stores the gyro and accelerometer data
	static int AN_OFFSET[6];		//Array that stores the Offset of the sensors
	static float Accel_Vector[3]; 	//Store the acceleration in a vector
	static float Gyro_Vector[3];	//Store the gyros turn rate in a vector
	static float Omega_Vector[3]; 	//Corrected Gyro_Vector data
	static float Omega_P[3];		//Omega Proportional correction
	static float Omega_I[3];		//Omega Integrator
	static float Omega[3];
	static float errorRollPitch[3];
	static float errorYaw[3];
	static float DCM_Matrix[3][3];
	static float Update_Matrix[3][3];		//Gyros here
	static float Temporary_Matrix[3][3];
	static const int SENSOR_SIGN[9];
};

float Vector_Dot_Product(float vector1[3],float vector2[3]);
void Vector_Cross_Product(float vectorOut[3], float v1[3],float v2[3]);
void Vector_Scale(float vectorOut[3],float vectorIn[3], float scale2);
void Vector_Add(float vectorOut[3],float vectorIn1[3], float vectorIn2[3]);
void Matrix_Multiply(float a[3][3], float b[3][3],float mat[3][3]);

#endif