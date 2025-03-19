#include "ServoController.h"

ServoController servoController;
GOD* god;

void setup() {
}


void loop() {
    std::vector<double> output = god->control_memory;
    servoController.updateServos(output);
}

