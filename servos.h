// Internal oscillator frequency varies between ICs, calibrate this to your part
#define SRV_OSCILLATOR_FREQUENCY (26450000)
#define SRV_UPDATE_RATE 50 // 50 Hz update for servos

/* Default controller I2C address ix 0x40
 * Default all-call LED address is 0xE0
 * Default SUBADDR1 address is 0xE2
 * Default SUBADDR2 address is 0xE4
 * Default SUBADDR3 address is 0xE8
 */
#define SRV_I2C_ADDR 0x40
#define SRV_REG_MODE_1 0
#define SRV_REG_MODE_2 1
#define SRV_REG_PRESCALE 254
// PRESCALE can only be set when SLEEP = 1
#define SRV_REG_CHAN_BASE 6

#define SRV_SLEEP_BIT 0x10
#define SRV_AI_BIT 0x20

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

void servoInit(void);
void servoSleep(void);
void servoWake(void);
void servoSetPrescale(unsigned char);
void servoSetAutoIncrement(bool);
void servoWriteSingleRegister(int, unsigned char);
unsigned char servoReadSingleRegister(int);
void servoSetChannelMicroseconds(int, long);
