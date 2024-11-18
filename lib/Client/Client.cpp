#include "Client.h"
#include <cstring>

Module::Module() = default;

void Module::generate_message(float value1, float value2, float value3, char *buffer) {
    std::memcpy(&buffer[0], &value1, sizeof(float));
    std::memcpy(&buffer[4], &value2, sizeof(float));
    std::memcpy(&buffer[8], &value3, sizeof(float));
}

void Module::unpack_message(const char *buffer, float &value1, float &value2, float &value3) {
    std::memcpy(&value1, &buffer[0], sizeof(float));
    std::memcpy(&value2, &buffer[4], sizeof(float));
    std::memcpy(&value3, &buffer[8], sizeof(float));
}

TemplateModule::TemplateModule() = default;

void TemplateModule::generate_message(float value1, float value2, float value3, char *buffer) {
    Module::generate_message(value1, value2, value3, buffer);
}

void TemplateModule::unpack_message(const char *buffer, float &value1, float &value2, float &value3) {
    Module::unpack_message(buffer, value1, value2, value3);
}

void Manager::add_module(Module *module) {
    modules.push_back(module);
}

void Manager::generate_combined_message(char identifier, char *combined_buffer) {
    combined_buffer[0] = identifier;
    int offset = 1;
    for (auto &module : modules) {
        char module_buffer[12];
        float value1 = 1.0f, value2 = 2.0f, value3 = 3.0f;
        module->generate_message(value1, value2, value3, module_buffer);
        std::memcpy(&combined_buffer[offset], module_buffer, 12);
        offset += 12;
    }
}

void Manager::unpack_combined_message(const char *combined_buffer, char &identifier, std::vector<std::tuple<float, float, float>> &values) {
    identifier = combined_buffer[0];
    int offset = 1;
    for (auto &module : modules) {
        float value1, value2, value3;
        module->unpack_message(&combined_buffer[offset], value1, value2, value3);
        values.emplace_back(value1, value2, value3);
        offset += 12;
    }
}