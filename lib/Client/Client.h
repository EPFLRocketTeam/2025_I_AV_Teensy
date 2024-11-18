#ifndef CLIENT_H
#define CLIENT_H

#include <Arduino.h>
#include <vector>
#include <tuple>

class Module {
public:
    Module();
    virtual void generate_message(float value1, float value2, float value3, char *buffer);
    virtual void unpack_message(const char *buffer, float &value1, float &value2, float &value3);
};

class TemplateModule : public Module {
public:
    TemplateModule();
    void generate_message(float value1, float value2, float value3, char *buffer) override;
    void unpack_message(const char *buffer, float &value1, float &value2, float &value3) override;
};

class Manager {
public:
    void add_module(Module *module);
    void generate_combined_message(char identifier, char *combined_buffer);
    void unpack_combined_message(const char *combined_buffer, char &identifier, std::vector<std::tuple<float, float, float>> &values);

private:
    std::vector<Module *> modules;
};

#endif // CLIENT_H
