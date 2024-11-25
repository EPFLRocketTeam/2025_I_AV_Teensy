#include <iostream>
#include <vector>
#include "../Client.h"

ThreeFloatModule three_float_module1;
ThreeFloatModule three_float_module2;
OneFloatModule one_float_module1;
OneFloatModule one_float_module2;
Manager manager;
int length = 41;    // 1 byte for identifier, 24 bytes for two ThreeFloatModules, 8 bytes for two OneFloatModules

void setup() {
    manager.set_module_configuration(0x01, {&three_float_module1, &three_float_module2, &one_float_module1, &one_float_module2});

    three_float_module1.set_values(10.0f, 25.6f, 30.56f);
    three_float_module2.set_values(5.0f, 15.6f, 20.56f);
    one_float_module1.set_value(19.89237f);
    one_float_module2.set_value(7.12345f);
}

void run_once() {
    char combined_message[length];
    manager.generate_combined_message(0x01, combined_message);

    std::cout << "Generated Message: ";
    for (int i = 0; i < length; ++i) {
        std::cout << std::hex << (int)(unsigned char)combined_message[i] << " ";
    }
    std::cout << std::endl;

    char identifier;
    std::vector<std::vector<float>> values;
    manager.unpack_combined_message(combined_message, identifier, values);

    std::cout << "Unpacked Identifier: " << static_cast<int>(identifier) << std::endl;
    for (const auto &module_values : values) {
        std::cout << "Values: ";
        for (const auto &val : module_values) {
            std::cout << val << ", ";
        }
        std::cout << std::endl;
    }
}

int main() {
    setup();
    run_once();
    return 0;
}