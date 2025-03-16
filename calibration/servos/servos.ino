/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/

#include <Servo.h>
#include "PPMReader.h"


#define MAX_WIDTH 2000. // max throttle,
#define MIN_WIDTH 1000. // min throttle,

#define MAXGIMBAL 10.        // maximum angle, °
#define MINGIMBAL -MAXGIMBAL // minimum angle, °

#define MAX_TILT 15. // the maximum angle the drone can be asked to tilt

// microsecond direct control

#define outer_center 1465
#define inner_center 1411
#define pwm_range 200

#define LOOP_PERIOD 20 // 50hz

// Servo centering
#define ZERO_ANGLE_OUTER 88. //12.5 - 11.7
#define ZERO_ANGLE_INNER 83.

Servo servo_inner, servo_outer; // create servo object to control a servo

const int servopin[2] = {14, 15}; // pwm output pin
                                  // x, y, throttle

// Specify the links and initial tuning parameters
uint32_t timer = millis();

// REMOTE CONTROLLER

// Define PPM pin
#define PPM_PIN 6

// Define the number of channels your PPM signal has
#define CHANNELS 5

// Create PPMReader object
PPMReader ppm(PPM_PIN, CHANNELS);

void setup()
{
    // initialize the variables we're linked to
    servo_inner.attach(servopin[1]); // attaches the servo on pin 14 to the servo object
    servo_outer.attach(servopin[0]); // attaches the servo on pin 15 to the servo object

    servo_inner.write(ZERO_ANGLE_INNER); // ZERO ANGLE FOR SERVO X
    servo_outer.write(ZERO_ANGLE_OUTER); // ZERO ANGLE FOR SERVO Y
    // delay(4000);

    // Setup NAV

    // begin serial and wait for opening
    Serial.begin(9600);
    while (!Serial)
        delay(100); // wait for native usb

    timer = millis();

    print_cols();
}

void print_cols()
{
    Serial.print("time, d1, d2\n");
}

void print_line(const std::vector<double> &elements)
{
    int n = elements.size();
    for (int i = 0; i < n; i++)
    {
        Serial.print(elements[i], 10);
        if (i != n - 1)
            Serial.print(",");
    }
    Serial.print("\n");
}

double deg_from_rad(double ang)
{
    return (ang / M_PI) * 180;
}

double rad_from_deg(double deg)
{
    return (deg / 180) * M_PI;
}


double angle_to_width(double angle, double center){
    return center + angle * 13.9;
}


void loop()
{
    long loop_start = millis();

    // REMOTE
    double roll_remote = ppm.rawChannelValue(1);
    double pitch_remote = ppm.rawChannelValue(2);

    double d1 = map(pitch_remote, MIN_WIDTH, MAX_WIDTH, MINGIMBAL, MAXGIMBAL);
    double d2 = map(roll_remote, MIN_WIDTH, MAX_WIDTH, MINGIMBAL, MAXGIMBAL);

    double w1 = angle_to_width(d1, inner_center);
    double w2 = angle_to_width(d2, outer_center);

    servo_inner.writeMicroseconds(w1);
    servo_outer.writeMicroseconds(w2);

    /*
    servo_inner.write(ZERO_ANGLE_INNER + d1);
    servo_outer.write(ZERO_ANGLE_OUTER + d2);
    */

    print_line({loop_start - timer, d1, d2, w1, w2});

    long loop_duration = millis() - loop_start;
    long to_sleep = LOOP_PERIOD - loop_duration;
    if (to_sleep > 0)
        delay(to_sleep);
}
