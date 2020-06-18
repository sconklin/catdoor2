// This #include statement was automatically added by the Particle IDE.
#include "servos.h"

#define NUMSERVOS 2

#define SUCCESS 0
#define E_LOOP_COUNT_EXCEEDED (-1)
#define E_PROGERR (-2)
#define E_BADARG (-3)


int setangle(String command)
{
    int angle = 0;
    int positions[2];
    int status;
    
    Serial.println("setangle");
    angle = command.toInt();
    Serial.printlnf("angle=%d", angle);
    positions[0] = angle;
    positions[1] = angle;
    status = movemotors(positions);
    Serial.printlnf("status=%d", status);
    return status;
}

int set_lock(String command) {
    int openPositions[2] = {90,90};
    int lockedPositions[2] = {180,180};
    int inPositions[2] = {45,45};
    int outPositions[2] = {270,270};
    
    int status;
    int i;
    
    String states[] = {"open", "locked", "in", "out"}; //array of possible commands
      for (i = 0; i <= sizeof(states)/sizeof(states[0]); i++) {
        if (command.equals(states[i]))
        break;
    }

    switch (i) {
        case 0: // open
            status = movemotors(openPositions);
        break;
        case 1: // locked
            status = movemotors(lockedPositions);
        break;
        case 2: // in
            status = movemotors(inPositions);
        break;
        case 3: // out
            status = movemotors(outPositions);
        break;
        default:
            status = E_BADARG;
        break;
    }
    
    return status;
}



void setup() {
    Serial.begin(9600);
    Wire.setSpeed(CLOCK_SPEED_400KHZ);
    Wire.begin(); // Join I2C as a master
    servoInit();
    servoSetChannelMicroseconds(1, SRV_STOP_US);
    servoSetChannelMicroseconds(2, SRV_STOP_US);
    Particle.function("setangle", setangle);
    Particle.function("set", set_lock);
    delay(5000);
}

void loop() {
    delay(1000);
}

/*
void loop() {
    // https://tutorial.cytron.io/2012/06/22/pid-for-embedded-design/
    // Get the current position (will be in range 0-360)
    current_position = (analogRead(SERVO_1_FEEDBACK_PIN) * 360) / 4096;

    // Calculate the error ans clamp to a range of -180 - +180
    rawerror = target_position - current_position;

    if (rawerror < -180)
        error = 360 + rawerror;
    else if (rawerror > 180)
        error = 360 - rawerror;
    else
        error = rawerror;
        
    abserror = abs(error);

    if (first_error) {
        last_error = error;
        first_error = false;
    }
    
    // Calculate the integral
    integral = integral + error;
    
    // calculate the derivative
    derivative = error - last_error;
    
    // calculate the control variable
    rawpwm = int((kp * (double)error) + (ki * (double)integral) + (kd * (double)derivative));
    
    // Map to a control range in uS with:
    //    Max CCW = 1590 uS (center + 100)
    //    Zero = 1490 uS
    //    Max CW = 1280 uS (center - 210)
    pwm = rawpwm * -1;
    pwm = pwm + 1490;
    if (pwm > highclamp) pwm = highclamp;
    if (pwm < lowclamp) pwm = lowclamp;
    
    // Check for exit conditions
    if ((pwm > 1475) && (pwm < 1505) && (abserror < 10)) {
        Serial.println("==> Done <==");
    } 

    if (++loopctr == 50000) {
        loopctr = 0;
        Serial.println("Pausing");
        Serial.printlnf("target_position = %d", target_position);
        Serial.printlnf("current_position = %d", current_position);
        Serial.printlnf("rawerror = %d", rawerror);
        Serial.printlnf("error = %d", error);
        Serial.printlnf("pwm = %d", pwm);
        Serial.printlnf("integral = %d", integral);
        Serial.printlnf("derivative = %d", derivative);
        Serial.println("----------------------------");

        // Having a tight loop can prevent flashing new code, so
        // a delay gives us a chance to reflash it
        delay(5000);
    }

    servoSetChannelMicroseconds(1, pwm);
    // possibly add some test here for zero to exit the loop
    last_error = error;
}
*/

// motor control parameters
int highclamp = 1570;
int lowclamp = 1290;
// PID coefficients
double kp = 0.5;
double ki = 0.0008; // .0005
double kd = 0; //0.05;

#define MAX_CONTROL_LOOP_COUNT 10000

int movemotors(int position[]) {
    int i;
    int current_position[NUMSERVOS];
    int pwm[NUMSERVOS];
    int integral[NUMSERVOS];
    int derivative[NUMSERVOS];
    int rawerror[NUMSERVOS];
    int error[NUMSERVOS];
    int abserror[NUMSERVOS];
    int lasterror[NUMSERVOS];
    bool done[NUMSERVOS];

    int loopctr = 0;
    Serial.printlnf("movemotors %d, %d", position[0], position[1]);

    // init any control variables
    for (i = 0; i < NUMSERVOS; i++) {
        lasterror[i] = 0;
        done[i] = false;
    }
    
    while (true) {

        // test for loop exit conditions
        if (loopctr > MAX_CONTROL_LOOP_COUNT) {
            Serial.println("Loop Count Exceeded");
            for (i=0; i<NUMSERVOS; i++) {
                servoSetChannelMicroseconds(i+1, SRV_STOP_US);
            }
            return (E_LOOP_COUNT_EXCEEDED);
        }

        if (loopctr > 0) {
            bool alldone = true;
            for (i=0; i<NUMSERVOS; i++) {
                if (!done[i]) {
                    alldone = false;
                    break;
                }
            }
            if (alldone) {
                Serial.println("All Done");
                return SUCCESS;
            }
        }

        // Now apply the control loop for each servo
        //Serial.printlnf("movemotors starting perservo");
        for (i=0; i<NUMSERVOS; i++) {
            if (done[i]) continue;
        
            switch(i) {
            // Get the current positions (will be in range 0-360)
            // TODO add this to the servo library and abstract it
            case 0:
                current_position[0] = (analogRead(SERVO_1_FEEDBACK_PIN) * 360) / 4096;
                break;
            case 1:
                current_position[1] = (analogRead(SERVO_2_FEEDBACK_PIN) * 360) / 4096;
                break;
            default:
                return E_PROGERR;
            }

            // Calculate the error and clamp to a range of -180 - +180
            rawerror[i] = position[i] - current_position[i];

            if (rawerror[i] < -180)
                error[i] = 360 + rawerror[i];
            else if (rawerror[i] > 180)
                error[i] = 360 - rawerror[i];
            else
                error[i] = rawerror[i];
        
            abserror[i] = abs(error[i]);

            if (loopctr == 0) {
                // set lasterror the first time through
                lasterror[i] = error[i];
            }

            integral[i] = integral[i] + error[i];
    
            derivative[i] = error[i] - lasterror[i];
    
            // calculate the control variable
            pwm[i] = int((kp * (double)error[i]) + (ki * (double)integral[i]) + (kd * (double)derivative[i]));

            pwm[i] = pwm[i] * -1;
    
            // Map to a control range in uS with:
            //    Max CCW = 1590 uS (center + 100)
            //    Zero = 1490 uS
            //    Max CW = 1280 uS (center - 210)
            pwm[i] = pwm[i] + 1490;
            if (pwm[i] > highclamp) {
                pwm[i] = highclamp;
            }
            if (pwm[i] < lowclamp) {
                pwm[i] = lowclamp;
            }
    
            // Check for exit conditions
            if ((pwm[i] > 1475) && (pwm[i] < 1505) && (abserror[i] < 10)) {
                done[i] = true;
                pwm[i] = SRV_STOP_US; // stop all movement
            }
          
            servoSetChannelMicroseconds(i+1, pwm[i]);
            lasterror[i] = error[i];

            /*
            if (++loopctr == 50000) {
                loopctr = 0;
                Serial.println("Pausing");
                Serial.printlnf("target_position = %d", target_position);
                Serial.printlnf("current_position = %d", current_position);
                Serial.printlnf("rawerror = %d", rawerror);
                Serial.printlnf("error = %d", error);
                Serial.printlnf("pwm = %d", pwm);
                Serial.printlnf("integral = %d", integral);
                Serial.printlnf("derivative = %d", derivative);
                Serial.println("----------------------------");

                // Having a tight loop can prevent flashing new code, so
                // a delay gives us a chance to reflash it
                delay(5000);
            }
            */
        }
        //Serial.printlnf("movemotors finished perservo");
        //delay(100);
        loopctr++;
        //Serial.printlnf("Loop Counter = %d", loopctr);

    }
}
