#include "../Client.h"

ThreeFloatModule three_float_module;
OneFloatModule one_float_module;
Manager manager;
int length = 17;    // 1 byte for identifier, 12 bytes for ThreeFloatModule, 4 bytes for OneFloatModule

void setup() {
    Serial.begin(9600);
    manager.add_module(&three_float_module);
    manager.add_module(&one_float_module);

    three_float_module.set_values(1.0f, 2.0f, 3.0f);
    one_float_module.set_value(4.0f);
}

void loop() {
    char combined_message[length];
    manager.generate_combined_message(0x01, combined_message);
    Serial.write(combined_message, length);

    delay(1000);

    if (Serial.available() >= length) {
        char received_message[length];
        Serial.readBytes(received_message, length);

        char identifier;
        std::vector<std::vector<float>> values;
        manager.unpack_combined_message(received_message, identifier, values);

        Serial.print("Identifier: ");
        Serial.println(identifier, HEX);
        for (const auto &module_values : values) {
            Serial.print("Values: ");
            for (const auto &val : module_values) {
                Serial.print(val);
                Serial.print(", ");
            }
            Serial.println();
        }
    }
}