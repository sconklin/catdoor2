#include "servos.h"

void setup() {
    Serial.begin(9600);
    Wire.setSpeed(CLOCK_SPEED_400KHZ);
    Wire.begin(); // Join I2C as a master

    // Calculate Prescale
    int prescale = (SRV_OSCILLATOR_FREQUENCY/(4096 * UPDATE_RATE))-1;

    // Make sure we're in sleep mode before writing prescale
    writeSingleRegister(SRV_REG_MODE_1, 0x11);
    delay(250);

    // Set PRESCALE for 50 Hz
    writeSingleRegister(SRV_REG_PRESCALE, prescale);

    // Set mode 1 register
    // address auto-increment plus not sleep
    writeSingleRegister(SRV_REG_MODE_1, 0x21);
}

void loop() {
    Serial.printlnf("M1 = %X", readSingleRegister(SRV_REG_MODE_1));
    Serial.printlnf("M2 = %X", readSingleRegister(SRV_REG_MODE_2));
    Serial.printlnf("Prescale = %X", readSingleRegister(SRV_REG_PRESCALE));

    setChannelMicroseconds(1, 10000);
  delay(1000);
}
