/*
 * Coded for the Adafruit 1411 servo controller,
 * which uses a PCA9685 I2C PWM LED controller.
 *
 * Default controller I2C address ix 0x40
 * Default all-call LED address is 0xE0
 * Default SUBADDR1 address is 0xE2
 * Default SUBADDR2 address is 0xE4
 * Default SUBADDR3 address is 0xE8
 */
 // The internal frequancy varies chip to chip, calibrate this to your controller
#define SRV_OSCILLATOR_FREQUENCY (26450000)
#define UPDATE_RATE 50 // 50 Hz update for servos
#define SRV_I2C_ADDR 0x40
#define SRV_REG_MODE_1 0
#define SRV_REG_MODE_2 1
#define SRV_REG_PRESCALE 254
// PRESCALE can only be set when SLEEP = 1
#define SRV_REG_CHAN_BASE 6

// Mode register 1 default
// 0x11
// 0        RESTART (0 = disabled)
//  0       EXTCLK (1 = enabled)
//   0      AUTOINCREMENT (1 enables register autoincrement)
//    1     SLEEP (1 = Low Power Mode)
//     0    SUB1 (1 = responds to I2C subaddress 1)
//      0   SUB2
//       0  SUB3
//        1 ALLCALL (1=responds to AllCall I2C address)

// Mode register 2 default
// 0x08
// 0        RESERVED, read only
//  0       RESERVED, read only
//   0      RESERVED, read only
//    0     INVERT (1 = invert output state)
//     0    OCH (0 = output changes state on STOP command, 1 = output changes state on ACK)
//      1   OUTDRV (0 = open-drain, 1 = totem pole)
//       0  OUTNE (see data sheet)
//        0 OUTNE 

double countsPerUsec = 4096.0/20000.0;

void writeSingleRegister(int regnum, uint8_t regval) {
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
    uint8_t regaddr;

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
