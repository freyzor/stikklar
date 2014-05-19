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
#include "minimu_ahrs.h"
#include "minimu_calibration.h"

// Uncomment the below line to use this axis definition: 
   // X axis pointing forward
   // Y axis pointing to the right 
   // and Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise
//const int MinIMU_AHRS::SENSOR_SIGN[9] = {1,1,1,-1,-1,-1,1,1,1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer

// Uncomment the below line to use this axis definition: 
   // X axis pointing forward
   // Y axis pointing to the left 
   // and Z axis pointing up.
// Positive pitch : nose down
// Positive roll : right wing down
// Positive yaw : counterclockwise
const int MinIMU_AHRS::SENSOR_SIGN[9] = {1,-1,-1,-1,1,1,1,-1,-1};

// array definitions
int MinIMU_AHRS::AN_OFFSET[6]={0,0,0,0,0,0}; //Array that stores the Offset of the sensors
float MinIMU_AHRS::Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float MinIMU_AHRS::Gyro_Vector[3]= {0,0,0};//Store the gyros turn rate in a vector
float MinIMU_AHRS::Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float MinIMU_AHRS::Omega_P[3]= {0,0,0};//Omega Proportional correction
float MinIMU_AHRS::Omega_I[3]= {0,0,0};//Omega Integrator
float MinIMU_AHRS::Omega[3]= {0,0,0};
float MinIMU_AHRS::errorRollPitch[3] = {0,0,0}; 
float MinIMU_AHRS::errorYaw[3] = {0,0,0};
float MinIMU_AHRS::DCM_Matrix[3][3] = {{1,0,0}, {0,1,0}, {0,0,1}}; 
float MinIMU_AHRS::Update_Matrix[3][3] = {{0, 1, 2},{3, 4, 5},{6, 7, 8}}; //Gyros here
float MinIMU_AHRS::Temporary_Matrix[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

MinIMU_AHRS::MinIMU_AHRS():
	counter(0),
	gyro_sat(0)
{}

void MinIMU_AHRS::initialize() {
	Accel_Init();
	Compass_Init();
	Gyro_Init();

	delay(20);

	for(int i=0; i < 32; i++)    // We take some readings...
	{
		Read_Gyro();
		Read_Accel();
		// Cumulate values
		for(int y=0; y < 6; y++) {
		 	AN_OFFSET[y] += AN[y];
			delay(20);
		}
	}

	for(int y=0; y<6; y++) {
		AN_OFFSET[y] = AN_OFFSET[y]/32;
	}

	AN_OFFSET[5]-=GRAVITY*SENSOR_SIGN[5];

	delay(1000);

	Serial.println("MinIMU AHRS initialized");

	timer_old=micros();
}

// called every 20ms or 50Hz
void MinIMU_AHRS::update() {
    unsigned long timer = micros();
    // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    G_Dt = (timer - timer_old)/1000000.0;
    timer_old = timer;
    
    // *** DCM algorithm
    // Data adquisition
    Read_Gyro();   // This read gyro data
    Read_Accel();     // Read I2C accelerometer
    
    if (counter > 5)  // Read compass data at 10Hz... (5 loop runs)
    {
      	counter = 0;
      	Read_Compass();    // Read I2C magnetometer
      	Compass_Heading(); // Calculate magnetic heading  
    }
    
    // Calculations...
    Matrix_update(); 
    Normalize();
    Drift_correction();

    // TODO: add debug output compatible with the AHRS python test app
    counter++;
}

void MinIMU_AHRS::Gyro_Init()
{
	gyro.init();
	gyro.writeReg(L3G_CTRL_REG4, 0x20); // 2000 dps full scale
	gyro.writeReg(L3G_CTRL_REG1, 0x0F); // normal power mode, all axes enabled, 100 Hz
}

void MinIMU_AHRS::Read_Gyro()
{
	gyro.read();

	AN[0] = gyro.g.x;
	AN[1] = gyro.g.y;
	AN[2] = gyro.g.z;
	gyro_x = SENSOR_SIGN[0] * (AN[0] - AN_OFFSET[0]);
	gyro_y = SENSOR_SIGN[1] * (AN[1] - AN_OFFSET[1]);
	gyro_z = SENSOR_SIGN[2] * (AN[2] - AN_OFFSET[2]);
}

void MinIMU_AHRS::Accel_Init()
{
	compass.init();
	compass.enableDefault();
	switch (compass.getDeviceType())
	{
	case LSM303::device_D:
	  	compass.writeReg(LSM303::CTRL2, 0x18); // 8 g full scale: AFS = 011
	  	break;
	case LSM303::device_DLHC:
	  	compass.writeReg(LSM303::CTRL_REG4_A, 0x28); // 8 g full scale: FS = 10; high resolution output mode
	  	break;
	default: // DLM, DLH
	  	compass.writeReg(LSM303::CTRL_REG4_A, 0x30); // 8 g full scale: FS = 11
	}
}

// Reads x,y and z accelerometer registers
void MinIMU_AHRS::Read_Accel()
{
	compass.readAcc();

	AN[3] = compass.a.x >> 4; // shift left 4 bits to use 12-bit representation (1 g = 256)
	AN[4] = compass.a.y >> 4;
	AN[5] = compass.a.z >> 4;
	accel_x = SENSOR_SIGN[3] * (AN[3] - AN_OFFSET[3]);
	accel_y = SENSOR_SIGN[4] * (AN[4] - AN_OFFSET[4]);
	accel_z = SENSOR_SIGN[5] * (AN[5] - AN_OFFSET[5]);
}

void MinIMU_AHRS::Compass_Init()
{
  // doesn't need to do anything because Accel_Init() should have already called compass.enableDefault()
}

void MinIMU_AHRS::Read_Compass()
{
	compass.readMag();

	magnetom_x = SENSOR_SIGN[6] * compass.m.x;
	magnetom_y = SENSOR_SIGN[7] * compass.m.y;
	magnetom_z = SENSOR_SIGN[8] * compass.m.z;
	// Serial.print("mx: "); Serial.print(compass.m.x); 
	// Serial.print(" my: "); Serial.print(compass.m.y); 
	// Serial.print(" mz: "); Serial.println(compass.m.z);
}



void MinIMU_AHRS::Compass_Heading()
{
	float MAG_X;
	float MAG_Y;
	float cos_roll;
	float sin_roll;
	float cos_pitch;
	float sin_pitch;

	cos_roll = cos(roll);
	sin_roll = sin(roll);
	cos_pitch = cos(pitch);
	sin_pitch = sin(pitch);

	// adjust for LSM303 compass axis offsets/sensitivity differences by scaling to +/-0.5 range
	// c_magnetom_x = SENSOR_SIGN[6] * (((compass.m.x - magn_off_x) * magn_scale_x) - 0.5);
	// c_magnetom_y = SENSOR_SIGN[7] * (((compass.m.y - magn_off_y) * magn_scale_y) - 0.5);
	// c_magnetom_z = SENSOR_SIGN[8] * (((compass.m.z - magn_off_z) * magn_scale_z) - 0.5);
	c_magnetom_x = (float)(magnetom_x - SENSOR_SIGN[6]*M_X_MIN) / (M_X_MAX - M_X_MIN) - SENSOR_SIGN[6]*0.5;
	c_magnetom_y = (float)(magnetom_y - SENSOR_SIGN[7]*M_Y_MIN) / (M_Y_MAX - M_Y_MIN) - SENSOR_SIGN[7]*0.5;
	c_magnetom_z = (float)(magnetom_z - SENSOR_SIGN[8]*M_Z_MIN) / (M_Z_MAX - M_Z_MIN) - SENSOR_SIGN[8]*0.5;
	// Serial.print("cx: "); Serial.print(c_magnetom_x);
	// Serial.print(" cy: "); Serial.print(c_magnetom_y);
	// Serial.print(" cz: "); Serial.println(c_magnetom_z);

	// Tilt compensated Magnetic filed X:
	MAG_X = c_magnetom_x*cos_pitch+c_magnetom_y*sin_roll*sin_pitch+c_magnetom_z*cos_roll*sin_pitch;
	// Tilt compensated Magnetic filed Y:
	MAG_Y = c_magnetom_y*cos_roll-c_magnetom_z*sin_roll;
	// Magnetic Heading
	MAG_Heading = atan2(-MAG_Y,MAG_X);
}

void MinIMU_AHRS::Normalize()
{
	float error=0;
	float temporary[3][3];
	float renorm=0;

	error= -Vector_Dot_Product(&DCM_Matrix[0][0],&DCM_Matrix[1][0])*.5; //eq.19

	Vector_Scale(&temporary[0][0], &DCM_Matrix[1][0], error); //eq.19
	Vector_Scale(&temporary[1][0], &DCM_Matrix[0][0], error); //eq.19

	Vector_Add(&temporary[0][0], &temporary[0][0], &DCM_Matrix[0][0]);//eq.19
	Vector_Add(&temporary[1][0], &temporary[1][0], &DCM_Matrix[1][0]);//eq.19

	Vector_Cross_Product(&temporary[2][0],&temporary[0][0],&temporary[1][0]); // c= a x b //eq.20

	renorm= 0.5 *(3 - Vector_Dot_Product(&temporary[0][0],&temporary[0][0])); //eq.21
	Vector_Scale(&DCM_Matrix[0][0], &temporary[0][0], renorm);

	renorm= 0.5 *(3 - Vector_Dot_Product(&temporary[1][0],&temporary[1][0])); //eq.21
	Vector_Scale(&DCM_Matrix[1][0], &temporary[1][0], renorm);

	renorm= 0.5 *(3 - Vector_Dot_Product(&temporary[2][0],&temporary[2][0])); //eq.21
	Vector_Scale(&DCM_Matrix[2][0], &temporary[2][0], renorm);
}

void MinIMU_AHRS::Drift_correction()
{
  float mag_heading_x;
  float mag_heading_y;
  float errorCourse;
  //Compensation the Roll, Pitch and Yaw drift. 
  static float Scaled_Omega_P[3];
  static float Scaled_Omega_I[3];
  float Accel_magnitude;
  float Accel_weight;
  
  //*****Roll and Pitch***************

  // Calculate the magnitude of the accelerometer vector
  Accel_magnitude = sqrt(Accel_Vector[0]*Accel_Vector[0] + Accel_Vector[1]*Accel_Vector[1] + Accel_Vector[2]*Accel_Vector[2]);
  Accel_magnitude = Accel_magnitude / GRAVITY; // Scale to gravity.
  // Dynamic weighting of accelerometer info (reliability filter)
  // Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
  Accel_weight = constrain(1 - 2*abs(1 - Accel_magnitude),0,1);  //  

  Vector_Cross_Product(&errorRollPitch[0],&Accel_Vector[0],&DCM_Matrix[2][0]); //adjust the ground of reference
  Vector_Scale(&Omega_P[0],&errorRollPitch[0],Kp_ROLLPITCH*Accel_weight);
  
  Vector_Scale(&Scaled_Omega_I[0],&errorRollPitch[0],Ki_ROLLPITCH*Accel_weight);
  Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);     
  
  //*****YAW***************
  // We make the gyro YAW drift correction based on compass magnetic heading
 
  mag_heading_x = cos(MAG_Heading);
  mag_heading_y = sin(MAG_Heading);
  errorCourse=(DCM_Matrix[0][0]*mag_heading_y) - (DCM_Matrix[1][0]*mag_heading_x);  //Calculating YAW error
  Vector_Scale(errorYaw,&DCM_Matrix[2][0],errorCourse); //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.
  
  Vector_Scale(&Scaled_Omega_P[0],&errorYaw[0],Kp_YAW);//.01proportional of YAW.
  Vector_Add(Omega_P,Omega_P,Scaled_Omega_P);//Adding  Proportional.
  
  Vector_Scale(&Scaled_Omega_I[0],&errorYaw[0],Ki_YAW);//.00001Integrator
  Vector_Add(Omega_I,Omega_I,Scaled_Omega_I);//adding integrator to the Omega_I
}

void MinIMU_AHRS::Matrix_update()
{
	Gyro_Vector[0]=Gyro_Scaled_X(gyro_x); //gyro x roll
	Gyro_Vector[1]=Gyro_Scaled_Y(gyro_y); //gyro y pitch
	Gyro_Vector[2]=Gyro_Scaled_Z(gyro_z); //gyro Z yaw

	Accel_Vector[0]=accel_x;
	Accel_Vector[1]=accel_y;
	Accel_Vector[2]=accel_z;

	Vector_Add(&Omega[0], &Gyro_Vector[0], &Omega_I[0]);  //adding proportional term
	Vector_Add(&Omega_Vector[0], &Omega[0], &Omega_P[0]); //adding Integrator term
	  
	Update_Matrix[0][0]=0;
	Update_Matrix[0][1]=-G_Dt*Omega_Vector[2];//-z
	Update_Matrix[0][2]=G_Dt*Omega_Vector[1];//y
	Update_Matrix[1][0]=G_Dt*Omega_Vector[2];//z
	Update_Matrix[1][1]=0;
	Update_Matrix[1][2]=-G_Dt*Omega_Vector[0];//-x
	Update_Matrix[2][0]=-G_Dt*Omega_Vector[1];//-y
	Update_Matrix[2][1]=G_Dt*Omega_Vector[0];//x
	Update_Matrix[2][2]=0;

	Matrix_Multiply(DCM_Matrix,Update_Matrix,Temporary_Matrix); //a*b=c

	for(int x=0; x<3; x++) //Matrix Addition (update)
	{
		for(int y=0; y<3; y++)
		{
		 	DCM_Matrix[x][y]+=Temporary_Matrix[x][y];
		} 
	}
}

void MinIMU_AHRS::updateEulerAngles()
{
	pitch = -asin(DCM_Matrix[2][0]);
	roll = atan2(DCM_Matrix[2][1],DCM_Matrix[2][2]);
	yaw = atan2(DCM_Matrix[1][0],DCM_Matrix[0][0]);
}

//Computes the dot product of two vectors
float Vector_Dot_Product(float vector1[3],float vector2[3])
{
	float op=0;

	for(int c=0; c<3; c++)
	{
		op+=vector1[c]*vector2[c];
	}

	return op; 
}

//Computes the cross product of two vectors
void Vector_Cross_Product(float vectorOut[3], float v1[3],float v2[3])
{
	vectorOut[0] = (v1[1]*v2[2]) - (v1[2]*v2[1]);
	vectorOut[1] = (v1[2]*v2[0]) - (v1[0]*v2[2]);
	vectorOut[2] = (v1[0]*v2[1]) - (v1[1]*v2[0]);
}

//Multiply the vector by a scalar. 
void Vector_Scale(float vectorOut[3],float vectorIn[3], float scale2)
{
	for(int c=0; c<3; c++)
	{
		vectorOut[c]=vectorIn[c]*scale2; 
	}
}

void Vector_Add(float vectorOut[3],float vectorIn1[3], float vectorIn2[3])
{
	for(int c=0; c<3; c++)
	{
		vectorOut[c]=vectorIn1[c]+vectorIn2[c];
	}
}

//Multiply two 3x3 matrixs. This function developed by Jordi can be easily adapted to multiple n*n matrix's. (Pero me da flojera!). 
void Matrix_Multiply(float a[3][3], float b[3][3],float mat[3][3])
{
	float op[3]; 
	for(int x=0; x<3; x++)
	{
		for(int y=0; y<3; y++)
		{
			for(int w=0; w<3; w++)
			{
				op[w] = a[x][w]*b[w][y];
			} 
			mat[x][y] = 0;
			mat[x][y] = op[0]+op[1]+op[2];

			float test = mat[x][y];
		}
	}
}
