#include "../Client.h"

TemplateModule template_module_1;
TemplateModule template_module_2;
Manager manager;

void setup() {
    Serial.begin(9600);
    manager.add_module(&template_module_1);
    manager.add_module(&template_module_2);
}

void loop() {
    char combined_message[25];
    manager.generate_combined_message(0x01, combined_message);
    Serial.write(combined_message, 25);

    delay(1000);

    if (Serial.available() >= 25) {
        char received_message[25];
        Serial.readBytes(received_message, 25);

        char identifier;
        std::vector<std::tuple<float, float, float>> values;
        manager.unpack_combined_message(received_message, identifier, values);

        Serial.print("Identifier: ");
        Serial.println(identifier, HEX);
        for (const auto &val : values) {
            Serial.print("Values: ");
            Serial.print(std::get<0>(val));
            Serial.print(", ");
            Serial.print(std::get<1>(val));
            Serial.print(", ");
            Serial.println(std::get<2>(val));
        }
    }
}