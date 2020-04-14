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

int prescale2 = 0;;

void setup() {
    Serial.begin(9600);
    Wire.setSpeed(CLOCK_SPEED_400KHZ);
    Wire.begin(); // Join I2C as a master
    // Calculate Prescale
    int prescale = (SRV_OSCILLATOR_FREQUENCY/(4096 * UPDATE_RATE))-1;

    // Make sure we're in sleep mode before writing prescale
    Wire.beginTransmission(SRV_I2C_ADDR);
    Wire.write(SRV_REG_MODE_1);
    Wire.write(0x11);
    Wire.endTransmission(true);        // stop transmitting
    delay(250);
    

    // Set PRESCALE for 50 Hz
    Wire.beginTransmission(SRV_I2C_ADDR);
    Wire.write(SRV_REG_PRESCALE);  // send the register address
    Wire.write(uint8_t(prescale)); // set the prescale register
    Wire.endTransmission(true);        // stop transmitting

    // Set mode 1 register
    // address auto-increment plus not sleep
    Wire.beginTransmission(SRV_I2C_ADDR);
    Wire.write(SRV_REG_MODE_1);
    Wire.write(0x21);
    Wire.endTransmission(true);        // stop transmitting
}

byte x = 0;

void loop() {
    // read the first mode register
    Wire.beginTransmission(SRV_I2C_ADDR);
    Wire.write(SRV_REG_MODE_1);
    Wire.endTransmission(true);        // stop transmitting
    Wire.requestFrom(SRV_I2C_ADDR, 1);    // request 1 byte
    if(Wire.available()) {   // slave may send less than requested
        char c = Wire.read();    // receive a byte as character
        Serial.printlnf("M1 = %X", c);
    } else {
        Serial.println("M1 nothing");
    }

    // read the second mode register
    Wire.beginTransmission(SRV_I2C_ADDR);
    Wire.write(SRV_REG_MODE_2);
    Wire.endTransmission(true);        // stop transmitting
    Wire.requestFrom(SRV_I2C_ADDR, 1);    // request 1 byte
    if(Wire.available()) {   // slave may send less than requested
        char c = Wire.read();    // receive a byte as character
        Serial.printlnf("M2 = %X", c);
    } else {
        Serial.println("M2 nothing");
    }

    // read the prescaleregister
    Wire.beginTransmission(SRV_I2C_ADDR);
    Wire.write(SRV_REG_PRESCALE);
    Wire.endTransmission(true);        // stop transmitting
    Wire.requestFrom(SRV_I2C_ADDR, 1);    // request 1 byte
    if(Wire.available()) {   // slave may send less than requested
        char c = Wire.read();    // receive a byte as character
        Serial.printlnf("Prescale = %d", c);
    } else {
        Serial.println("Prescale nothing");
    }


    Wire.beginTransmission(SRV_I2C_ADDR);
    Wire.write(SRV_REG_CHAN_BASE);
    Wire.write(0x00); // set the register data
    Wire.endTransmission();        // stop transmitting

    Wire.beginTransmission(SRV_I2C_ADDR);
    Wire.write(SRV_REG_CHAN_BASE+1);
    Wire.write(0x00); // set the register data
    Wire.endTransmission();        // stop transmitting

    Wire.beginTransmission(SRV_I2C_ADDR);
    Wire.write(SRV_REG_CHAN_BASE+2);
    Wire.write(0x00); // set the register data
    Wire.endTransmission();        // stop transmitting

    Wire.beginTransmission(SRV_I2C_ADDR);
    Wire.write(SRV_REG_CHAN_BASE+3);
    Wire.write(0x08); // set the register data
    Wire.endTransmission(true);        // stop transmitting

  delay(1000);
}
