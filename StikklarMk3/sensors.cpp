#include "sensors.h"
#include "ax12.h"

#define AX_SERVO_INFO_LENGTH	8

bool axReadServoInfo(int id, volatile BlackBoard& bb) {  
    setTX(id);
    // 0xFF 0xFF ID LENGTH INSTRUCTION PARAM... CHECKSUM    
    int checksum = ~((id + 6 + AX_PRESENT_POSITION_L + AX_SERVO_INFO_LENGTH) % 256);
    ax12writeB(0xFF);
    ax12writeB(0xFF);
    ax12writeB(id);
    ax12writeB(4);    // length
    ax12writeB(AX_READ_DATA);
    ax12writeB(AX_PRESENT_POSITION_L);
    ax12writeB(AX_SERVO_INFO_LENGTH);
    ax12writeB(checksum);  
    setRX(id);    
    if(ax12ReadPacket(AX_SERVO_INFO_LENGTH + 6) > 0){
        bb.axError[id-1] = ax_rx_buffer[4];
        bb.axPosition[id-1] = ax_rx_buffer[5] + (ax_rx_buffer[6] << 8);
        bb.axSpeed[id-1] = ax_rx_buffer[7] + (ax_rx_buffer[8] << 8);
        bb.axLoad[id-1] = ax_rx_buffer[9] + (ax_rx_buffer[10] << 8);
        bb.voltage = ax_rx_buffer[11];
        bb.axTemperature[id-1] = ax_rx_buffer[12];
        return true;
    }else{
        return false;
    }
}