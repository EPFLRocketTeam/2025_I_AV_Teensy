#include "ServoController.h"
// TODO: Add ControlOutput and GOD


ServoController servoController;
God* god;

void setup() {
}


void loop() {
    ControlOutput output = god->control_output_memory->Read();
    servoController.updateServos(output);
}




