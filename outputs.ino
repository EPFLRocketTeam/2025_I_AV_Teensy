#include <Servo.h>

#define MAXGIMBAL 15.        // maximum angle, °
#define MINGIMBAL -MAXGIMBAL // minimum angle, °

#define MIN_W 1000
#define W_RANGE 1000

#define outer_center 1465
#define inner_center 1411

#define MAX_CONVERSION_THRUST 20

const int servopin[4] = {14, 15, 9, 8}; // pwm output pin
// x, y, throttle top, throttle bottom

Servo servo_inner, servo_outer, top_motor, bot_motor; // create servo object to control a servo

void setup_outputs()
{
    servo_inner.attach(servopin[1]); // attaches the servo on pin 14 to the servo object
    servo_outer.attach(servopin[0]); // attaches the servo on pin 15 to the servo object
    top_motor.attach(servopin[2]);
    top_motor.writeMicroseconds(1000);
    bot_motor.attach(servopin[3]);
    bot_motor.writeMicroseconds(1000);
    servo_inner.writeMicroseconds(inner_center);
    servo_outer.writeMicroseconds(outer_center);

    Serial.println("Finished output setup");
}

void unarmed_output()
{
    top_motor.writeMicroseconds(1000);
    bot_motor.writeMicroseconds(1000);
    servo_inner.writeMicroseconds(inner_center);
    servo_outer.writeMicroseconds(outer_center);
}

double output_from_thrust(double thrust)
{
    if (thrust < 0)
    {
        return 0;
    }
    else
    {
        double a = 8.49693442;
        double b = 10.75630996;
        double throttle = (-b + std::sqrt(b*b + 4*a*thrust)) / (2*a);
        throttle = std::clamp(throttle, 0., 1.);
        return throttle;
    }
}

double throttle_to_pwm(double throttle){
    return MIN_W + (throttle * W_RANGE);
}

/**
 * Transform a control output to a raw output for the actual motors
 */
RawOutput to_raw(ControlOutput cont){
    double throttle = output_from_thrust(cont.thrust);
    double throttle_diff = -0.2114217 * cont.mz + 0.04766524;

    return {cont.d1, cont.d2, throttle, throttle_diff};
}

double angle_to_width(double angle, double center){
    return center + angle * 13.9;
}


void write_raw_outputs(RawOutput out)
{
    double top_throttle = std::clamp(out.avg_throttle + out.throttle_diff/2, 0., 1.);
    double bot_throttle = std::clamp(out.avg_throttle - out.throttle_diff/2, 0., 1.);

    if(!isnan(out.d1) && !isnan(out.d2) && !isnan(top_throttle) && !isnan(bot_throttle)){
        top_motor.writeMicroseconds(throttle_to_pwm(top_throttle));
        bot_motor.writeMicroseconds(throttle_to_pwm(bot_throttle));
        
        double d1 = std::clamp(out.d1, MINGIMBAL, MAXGIMBAL);
        double d2 = std::clamp(out.d2, MINGIMBAL, MAXGIMBAL);

        double w1 = angle_to_width(d1, inner_center);
        double w2 = angle_to_width(d2, outer_center);

        servo_inner.writeMicroseconds(w1);
        servo_outer.writeMicroseconds(w2);
    }
}