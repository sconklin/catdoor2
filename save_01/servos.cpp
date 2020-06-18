#include "application.h""
#include "servos.h""
/*
 * Coded for the Adafruit 1411 servo controller,
 * which uses a PCA9685 I2C PWM LED controller.
 */
double countsPerUsec = 4096.0/20000.0;
int servoPosition1;
int servoPosition2;

void servoHandleTimer()
{
    servoPosition1 = (analogRead(SERVO_1_FEEDBACK_PIN) * 360) / 4096;
    servoPosition2 = (analogRead(SERVO_1_FEEDBACK_PIN) * 360) / 4096;
}

Timer servotimer(10, servoHandleTimer);

void servoInit(void) {
    // Calculate Prescale
    int prescale = (SRV_OSCILLATOR_FREQUENCY/(4096 * SRV_UPDATE_RATE))-1;

    servoSetPrescale(prescale);
    servoSetAutoIncrement(true);
    servotimer.start();
}

void servoSleep(void) {
    unsigned char regval = servoReadSingleRegister(SRV_REG_MODE_1);
    servoWriteSingleRegister(SRV_REG_MODE_1, regval |= SRV_SLEEP_BIT);
    delay(50);
}

void servoWake() {
    unsigned char regval = servoReadSingleRegister(SRV_REG_MODE_1);
    servoWriteSingleRegister(SRV_REG_MODE_1, regval &= ~SRV_SLEEP_BIT);
}

void servoSetPrescale(unsigned char prescale) {
    servoSleep();
    servoWriteSingleRegister(SRV_REG_PRESCALE, prescale);
    servoWake();
}

void servoSetAutoIncrement(bool ai) {
    unsigned char regval = servoReadSingleRegister(SRV_REG_MODE_1);
    servoWriteSingleRegister(SRV_REG_MODE_1, regval |= SRV_AI_BIT);
}

void servoWriteSingleRegister(int regnum, unsigned char regval) {
    Wire.beginTransmission(SRV_I2C_ADDR);
    Wire.write(regnum);
    Wire.write(regval);
    Wire.endTransmission(true);        // stop transmitting
}

unsigned char servoReadSingleRegister(int regnum) {
    Wire.beginTransmission(SRV_I2C_ADDR);
    Wire.write(regnum);
    Wire.endTransmission(true);        // stop transmitting
    Wire.requestFrom(SRV_I2C_ADDR, 1);    // request 1 byte
    if(Wire.available()) {   // slave may send less than requested
        return( Wire.read());
    } else {
        return 0;
    }
}

void servoSetChannelMicroseconds(int channum, long usecs) {
    uint16_t offvalue;
    unsigned char regaddr;

    offvalue = round(double(usecs) * countsPerUsec);
    regaddr = SRV_REG_CHAN_BASE + ((channum-1)*4);

    // Always set start time to zero
    // Assume register address autoincrement is on
    Wire.beginTransmission(SRV_I2C_ADDR);
    Wire.write(regaddr);
    Wire.write(0x00);
    Wire.write(0x00);
    Wire.write(offvalue & 0xFF);
    Wire.write((offvalue & 0x0F00) >> 8);
    Wire.endTransmission(true);
}
