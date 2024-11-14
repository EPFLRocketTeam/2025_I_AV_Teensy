#include <Arduino.h>
#include <cstdint>
#include <cstring>
#include "Client.h"

// Define the module ID for the template module
#define TEMPLATE_MODULE_ID 1

void setup() {
    Serial.begin(9600);
}

void loop() {
    uint8_t numModules = 2;  // Example: sending 2 modules

    Serial.write(numModules);  // Send the number of modules as 1 byte

    // Create instances of the modules
    TemplateModule template_module(TEMPLATE_MODULE_ID);
    char template_message[5];
    float template_value = 25.6f;
    template_module.generate_message(template_value, template_message);

    // Send data for each module
    for (int i = 0; i < numModules; i++) {
        // Send each byte of the message over UART
        for (int j = 0; j < 5; j++) {
            Serial.write(template_message[j]);  // Send one byte at a time
        }
    }
}
