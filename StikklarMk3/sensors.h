#ifndef SENSORS_H
#define SENSORS_H

// read a single 
bool readAxServoInfo(int id);

// read the front IR sensors
void readIrSensors();

// returns an error code or -1 if no data was read
int ax12Ping(int id);

// write AHRS test angle output
void outputAHRS();

#endif