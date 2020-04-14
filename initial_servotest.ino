#include "servos.h"

void setup() {
    Serial.begin(9600);
    Wire.setSpeed(CLOCK_SPEED_400KHZ);
    Wire.begin(); // Join I2C as a master
    servoInit();
    servoSetChannelMicroseconds(1, SRV_STOP_US);
    servoSetChannelMicroseconds(2, SRV_STOP_US);

    servoSetChannelMicroseconds(1, SRV_CW_VERY_SLOW_US);
}

int i = 0;
void loop() {
    /*
    Serial.printlnf("M1 = %X", servoReadSingleRegister(SRV_REG_MODE_1));
    Serial.printlnf("M2 = %X", servoReadSingleRegister(SRV_REG_MODE_2));
    Serial.printlnf("Prescale = %X", servoReadSingleRegister(SRV_REG_PRESCALE));
    */
/*
    if (i == 0) {
        servoSetChannelMicroseconds(1, SRV_STOP_US);
        servoSetChannelMicroseconds(2, SRV_STOP_US);
        i++;
    } else if (i == 1) {
        servoSetChannelMicroseconds(1, SRV_CW_VERY_SLOW_US);
        servoSetChannelMicroseconds(2, SRV_CW_VERY_SLOW_US);
        i++;
    } else if (i == 2) {
        servoSetChannelMicroseconds(1, SRV_STOP_US);
        servoSetChannelMicroseconds(2, SRV_STOP_US);
        i++;
    } else if (i == 3) {
        servoSetChannelMicroseconds(1, SRV_CCW_VERY_SLOW_US);
        servoSetChannelMicroseconds(2, SRV_CCW_VERY_SLOW_US);
        i = 0;
    }
*/
    Serial.printlnf("Position = %d", servoPosition1);
    delay(100);
}
