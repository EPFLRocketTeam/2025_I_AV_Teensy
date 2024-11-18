#include "Client.h"
#include <cstring>

// ThreeFloatModule.cpp
ThreeFloatModule::ThreeFloatModule() : value1(0.0f), value2(0.0f), value3(0.0f) {}

void ThreeFloatModule::set_values(float v1, float v2, float v3) {
    value1 = v1;
    value2 = v2;
    value3 = v3;
}

void ThreeFloatModule::generate_message(char *buffer) {
    std::memcpy(&buffer[0], &value1, sizeof(float));
    std::memcpy(&buffer[4], &value2, sizeof(float));
    std::memcpy(&buffer[8], &value3, sizeof(float));
}

void ThreeFloatModule::unpack_message(const char *buffer) {
    std::memcpy(&value1, &buffer[0], sizeof(float));
    std::memcpy(&value2, &buffer[4], sizeof(float));
    std::memcpy(&value3, &buffer[8], sizeof(float));
}

// OneFloatModule.cpp
OneFloatModule::OneFloatModule() : value(0.0f) {}

void OneFloatModule::set_value(float v) {
    value = v;
}

void OneFloatModule::generate_message(char *buffer) {
    std::memcpy(&buffer[0], &value, sizeof(float));
}

void OneFloatModule::unpack_message(const char *buffer) {
    std::memcpy(&value, &buffer[0], sizeof(float));
}

void Manager::add_module(Module *module) {
    modules.push_back(module);
}

void Manager::generate_combined_message(char identifier, char *combined_buffer) {
    combined_buffer[0] = identifier;
    int offset = 1;
    for (auto &module : modules) {
        char module_buffer[12];
        module->generate_message(module_buffer);
        std::memcpy(&combined_buffer[offset], module_buffer, 12);
        offset += 12;
    }
}

void Manager::unpack_combined_message(const char *combined_buffer, char &identifier, std::vector<std::vector<float>> &values) {
    identifier = combined_buffer[0];
    int offset = 1;
    for (auto &module : modules) {
        std::vector<float> module_values;
        if (dynamic_cast<ThreeFloatModule*>(module)) {
            float value1, value2, value3;
            module->unpack_message(&combined_buffer[offset]);
            module_values.push_back(value1);
            module_values.push_back(value2);
            module_values.push_back(value3);
            offset += 12;
        } else if (dynamic_cast<OneFloatModule*>(module)) {
            float value;
            module->unpack_message(&combined_buffer[offset]);
            module_values.push_back(value);
            offset += 4;
        }
        values.push_back(module_values);
    }
}