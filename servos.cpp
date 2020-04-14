#include "application.h""
#include "servos.h""
/*
 * Coded for the Adafruit 1411 servo controller,
 * which uses a PCA9685 I2C PWM LED controller.
 */
double countsPerUsec = 4096.0/20000.0;

void writeSingleRegister(int regnum, unsigned char regval) {
    Wire.beginTransmission(SRV_I2C_ADDR);
    Wire.write(regnum);
    Wire.write(regval);
    Wire.endTransmission(true);        // stop transmitting
}

char readSingleRegister(int regnum) {
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

void setChannelMicroseconds(int channum, long usecs) {
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
