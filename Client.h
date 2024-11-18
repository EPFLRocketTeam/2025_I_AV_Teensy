#ifndef CLIENT_H
#define CLIENT_H

#include <vector>
#include <tuple>
#include <cstring>

class Module {
public:
    virtual void generate_message(char *buffer) = 0;
    virtual void unpack_message(const char *buffer) = 0;
};

class ThreeFloatModule : public Module {
public:
    ThreeFloatModule();
    void set_values(float value1, float value2, float value3);
    void generate_message(char *buffer) override;
    void unpack_message(const char *buffer) override;

private:
    float value1, value2, value3;
};

// OneFloatModule.h
class OneFloatModule : public Module {
public:
    OneFloatModule();
    void set_value(float value);
    void generate_message(char *buffer) override;
    void unpack_message(const char *buffer) override;

private:
    float value;
};

class Manager {
public:
    void add_module(Module *module);
    void generate_combined_message(char identifier, char *combined_buffer);
    void unpack_combined_message(const char *combined_buffer, char &identifier, std::vector<std::vector<float>> &values);

private:
    std::vector<Module *> modules;
};

#endif // CLIENT_H