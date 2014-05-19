#include "sensors.h"

#include <Arduino.h>

#include "blackboard.h"
#include "ax12.h"

#define AX_PREAMBLE_LENGTH      6
#define AX_SERVO_INFO_LENGTH	8
#define AX_TOTAL_SERVO_INFO_LENGTH   AX_PREAMBLE_LENGTH + AX_SERVO_INFO_LENGTH

bool readAxServoInfo(int id) {  
    setTX(id);
    // 0xFF 0xFF ID LENGTH INSTRUCTION PARAM... CHECKSUM    
    int checksum = ~((id + AX_PREAMBLE_LENGTH + AX_PRESENT_POSITION_L + AX_SERVO_INFO_LENGTH) % 256);
    ax12writeB(0xFF);
    ax12writeB(0xFF);
    ax12writeB(id);
    ax12writeB(4);    // length
    ax12writeB(AX_READ_DATA);
    ax12writeB(AX_PRESENT_POSITION_L);
    ax12writeB(AX_SERVO_INFO_LENGTH);
    ax12writeB(checksum);  
    setRX(id);    
    if(ax12ReadPacket(AX_TOTAL_SERVO_INFO_LENGTH) > 0) {
        // write to the volitile black board
        int index = id-1;
        blackboard.axError[index] = ax_rx_buffer[4];
        blackboard.axPosition[index] = ax_rx_buffer[5] + (ax_rx_buffer[6] << 8);
        blackboard.axSpeed[index] = ax_rx_buffer[7] + (ax_rx_buffer[8] << 8);
        blackboard.axLoad[index] = ax_rx_buffer[9] + (ax_rx_buffer[10] << 8);
        blackboard.axTemperature[index] = ax_rx_buffer[12];
        // we only keep a single voltage as it the same bus for all servos
        blackboard.voltage = ax_rx_buffer[11];
        return true;
    }else{
        return false;
    }
}

void readIrSensors(){
    blackboard.topIrIntensity = ax12GetRegister(AX_SENSOR, AX_CENTER_IR_DATA, 1);
    blackboard.centerIrIntensity = ax12GetRegister(AX_SENSOR, AX_LEFT_IR_DATA, 1);
    blackboard.bottomIrIntensity = ax12GetRegister(AX_SENSOR, AX_LEFT_IR_DATA, 1);
}

int ax12Ping(int id) {
    setTX(id);
    // 0xFF 0xFF ID LENGTH INSTRUCTION PARAM... CHECKSUM    
    int checksum = ~((id + 0x02 + AX_PING) % 256);
    ax12writeB(0xFF);
    ax12writeB(0xFF);
    ax12writeB(byte(id));
    ax12writeB(0x02);    // length
    ax12writeB(AX_PING);
    ax12writeB(checksum);  
    setRX(id);    
    if(ax12ReadPacket(6) > 0){
        // return the error
        return ax_rx_buffer[4];
    }else{
        return -1;
    }
}

#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

void outputAHRS()
{    
    Serial.print("!");
    Serial.print("ANG:");
    Serial.print(ToDeg(blackboard.roll));
    Serial.print(",");
    Serial.print(ToDeg(blackboard.pitch));
    Serial.print(",");
    Serial.println(ToDeg(blackboard.yaw));
}
