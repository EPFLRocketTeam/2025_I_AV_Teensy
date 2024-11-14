#include <iostream>
#include <bitset>
#include <cstring>
#include <vector>

// Define module IDs here, max number of ids is 8, can be changed by adding a byte, resulting in 128 ids
#define TEMPLATE_MODULE_ID 1
#define ANOTHER_MODULE_ID 2

// Base class for all the modules that need to send data over UART
class Module {
public:
    explicit Module(int module_id, int data_size) : module_id(module_id), data_size(data_size) {}

    // Method to package data into a byte array (1 byte for sensor ID, 4 bytes for each float value)
    virtual void generate_message(const std::vector<float>& values, char *buffer) {
        // Pack sensor ID (1 byte)
        buffer[0] = static_cast<char>(module_id);

        // Copy each float value into the buffer starting at index 1
        for (size_t i = 0; i < values.size(); ++i) {
            std::memcpy(&buffer[1 + i * sizeof(float)], &values[i], sizeof(float));
        }
    }

    // Method to unpack the message
    virtual void unpack_message(const char *buffer, std::vector<float>& values) {
        // Extract sensor ID (1 byte)
        module_id = static_cast<int>(static_cast<unsigned char>(buffer[0]));

        // Extract the float values from the buffer
        values.clear();
        for (size_t i = 0; i < data_size / sizeof(float); ++i) {
            float value;
            std::memcpy(&value, &buffer[1 + i * sizeof(float)], sizeof(float));
            values.push_back(value);
        }
    }

protected:
    int module_id;
    int data_size;
};

// Example module
class TemplateModule : public Module {
public:
    explicit TemplateModule(int module_id) : Module(module_id, 4 * sizeof(float)) {}

    void generate_message(const std::vector<float>& values, char *buffer) override {
        Module::generate_message(values, buffer);
    }

    void unpack_message(const char *buffer, std::vector<float>& values) override {
        Module::unpack_message(buffer, values);
    }
};

// Helper function to print the message in hex format
void print_message(const char* message, size_t size) {
    for (size_t i = 0; i < size; ++i) {
        printf("%02X ", (unsigned char)message[i]);
    }
    printf("\n");
}

// Example usage
int main() {
    TemplateModule template_module(TEMPLATE_MODULE_ID);
    TemplateModule another_module(ANOTHER_MODULE_ID);
    char another_message[5];
    char template_message[17]; // 1 byte for ID + 4 * 4 bytes for float values
    std::vector<float> another_values = {1.1f};
    std::vector<float> template_values = {25.6f, 26.1f, 27.2f, 28.3f};
    another_module.generate_message(another_values, another_message);
    template_module.generate_message(template_values, template_message);
    std::cout << "Template Message: ";
    print_message(template_message, sizeof(template_message));
    print_message(another_message, sizeof(another_message));
    std::vector<float> unpacked_values;
    std::vector<float> unpacked_template;
    template_module.unpack_message(template_message, unpacked_template);
    another_module.unpack_message(another_message, unpacked_values);
    std::cout << "Unpacked Template: ";
    for (float value : unpacked_template) {
        std::cout << value << " ";
    }
    std::cout << std::endl;
    for (float value : unpacked_values) {
        std::cout << value << " ";
    }
    std::cout << std::endl;
    return 0;
}