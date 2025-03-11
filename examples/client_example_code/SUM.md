# SOFTWARE USAGE MANUAL

## Overview

This software provides a framework for managing and communicating with different types of sensor modules. The modules can generate and unpack messages, which are then combined and encoded/decoded using Base64 encoding. The software supports two types of modules: `ThreeFloatModule` and `OneFloatModule`.

## Classes

### Module

An abstract base class for all modules. It defines the interface for generating and unpacking messages and getting the module type.

### ThreeFloatModule

A module that handles three float values.

- **Methods:**
    - `set_values(float value1, float value2, float value3)`: Sets the three float values.
    - `generate_message(char *buffer)`: Generates a message containing the three float values.
    - `unpack_message(const char *buffer)`: Unpacks a message to retrieve the three float values.
    - `get_type() const`: Returns the type of the module (`Type::ThreeFloat`).

### OneFloatModule

A module that handles a single float value.

- **Methods:**
    - `set_value(float value)`: Sets the float value.
    - `generate_message(char *buffer)`: Generates a message containing the float value.
    - `unpack_message(const char *buffer)`: Unpacks a message to retrieve the float value.
    - `get_type() const`: Returns the type of the module (`Type::OneFloat`).

### Manager

Manages the configuration of modules and handles the generation and unpacking of combined messages.

- **Methods:**
    - `set_module_configuration(char identifier, const std::vector<Module *> &modules)`: Sets the configuration of modules for a given identifier.
    - `generate_combined_message(char identifier, char *combined_buffer, std::string &encoded_message)`: Generates a combined message for the modules associated with the given identifier and encodes it using Base64.
    - `unpack_combined_message(const std::string &encoded_buffer, char &identifier, std::vector<std::vector<float>> &values)`: Unpacks a combined message from the encoded buffer and retrieves the values for the modules.

## Usage

### Setup

1. Create instances of `ThreeFloatModule` and `OneFloatModule`.
2. Set the values for each module.
3. Configure the `Manager` with the modules.

### Example

```cpp
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
    std::string encoded_message;
    manager.generate_combined_message(0x01, combined_message, encoded_message);

    std::cout << "Generated Message: " << encoded_message << std::endl;

    char identifier;
    std::vector<std::vector<float>> values;
    manager.unpack_combined_message(encoded_message, identifier, values);

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
```

### Packet Sizes

- **ThreeFloatModule**: 12 bytes (3 floats, each 4 bytes)
- **OneFloatModule**: 4 bytes (1 float, 4 bytes)
- **Combined Message**:
    - 1 byte for identifier
    - Sum of the sizes of all modules in the configuration

For example, if the configuration includes two `ThreeFloatModule` instances and two `OneFloatModule` instances, the combined message size will be:
- 1 byte (identifier) + 24 bytes (two `ThreeFloatModule` instances) + 8 bytes (two `OneFloatModule` instances) = 33 bytes

The encoded message size will be larger due to Base64 encoding.