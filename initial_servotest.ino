#include "servos.h"

void setup() {
    Serial.begin(9600);
    Wire.setSpeed(CLOCK_SPEED_400KHZ);
    Wire.begin(); // Join I2C as a master
    servoInit();
}

void loop() {
    Serial.printlnf("M1 = %X", servoReadSingleRegister(SRV_REG_MODE_1));
    Serial.printlnf("M2 = %X", servoReadSingleRegister(SRV_REG_MODE_2));
    Serial.printlnf("Prescale = %X", servoReadSingleRegister(SRV_REG_PRESCALE));

    servoSetChannelMicroseconds(1, 10000);
  delay(1000);
}
