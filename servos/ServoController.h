#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include <Servo.h>
// TODO: Add ControlOutput

#define MAX_WIDTH 2000. // max throttle,
#define MIN_WIDTH 1000. // min throttle,

#define MAXGIMBAL 10.        // maximum angle, °
#define MINGIMBAL -MAXGIMBAL // minimum angle, °

#define MAX_TILT 15. // the maximum angle the drone can be asked to tilt

// microsecond direct control

#define outer_center 1453
#define inner_center 1411
#define pwm_range 200

#define LOOP_PERIOD 20 // 50hz

// Servo centering
#define ZERO_ANGLE_OUTER 88. //12.5 - 11.7
#define ZERO_ANGLE_INNER 83.

#include <iostream>

class ServoController {
public:
    ServoController();
    void updateServos(ControlOutput output);

private:
    int servo1Pin;
    int servo2Pin;
    double currentAngle1;
    double currentAngle2;
    double desiredAngle1;
    double desiredAngle2;
    double angle_to_width(double angle, double center);

    void writeServo(double d1, double d2);

    Servo servo_inner;
    Servo servo_outer;
};

#endif // SERVO_CONTROLLER_H