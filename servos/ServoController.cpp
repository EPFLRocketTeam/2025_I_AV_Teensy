#include "ServoController.h"

ServoController::ServoController() {
    // Initialize the servos
    const int servopin[2] = {14, 15};
    servo_inner.attach(servopin[1]);
    servo_outer.attach(servopin[0]);

    // Center the servos
    servo_inner.write(ZERO_ANGLE_INNER);
    servo_outer.write(ZERO_ANGLE_OUTER);
}

void ServoController::updateServos(std::vector<double> output) {
    writeServo(output[0], output[1]);
}

double ServoController::angle_to_width(double angle, double center) {
    return center + angle * 13.9;
}

void ServoController::writeServo(double d1, double d2) {
    double width1 = angle_to_width(d1, inner_center);
    double width2 = angle_to_width(d2, outer_center);
    servo_inner.writeMicroseconds(width1);
    servo_outer.writeMicroseconds(width2);
}