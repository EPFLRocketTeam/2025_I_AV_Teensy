#include <iostream>
#include <bitset>
#include <cstring>

// Define module IDs here, max number of ids is 8, can be changed by adding a byte, resulting in 128 ids
#define TEMPLATE_MODULE_ID 1

// Base class for all the modules that need to send data over UART
class Module {
public:
    explicit Module(int module_id) : module_id(module_id) {}

    // Method to package data into a 5-byte array (1 byte for sensor ID, 4 bytes for float value)
    virtual void generate_message(float value, char *buffer) {
        // Pack sensor ID (1 byte) and 4-byte float value (IEEE 754 format)
        buffer[0] = static_cast<char>(module_id);  // 1 byte for sensor ID
        // Copy the 4-byte float into the buffer starting at index 1
        std::memcpy(&buffer[1], &value, sizeof(float));
    }

    // Method to unpack the 5-byte message
    virtual void unpack_message(const char *buffer, float &value) {
        // Extract sensor ID (1 byte)
        module_id = static_cast<int>(static_cast<unsigned char>(buffer[0]));

        // Extract the 4-byte float value from the buffer
        std::memcpy(&value, &buffer[1], sizeof(float));
    }

protected:
    int module_id;
};


// Example module
class TemplateModule : public Module {
public:
    explicit TemplateModule(int module_id) : Module(module_id) {}

    void generate_message(float value, char *buffer) override {
        Module::generate_message(value, buffer);
    }

    void unpack_message(const char *buffer, float &temperature) override {
        Module::unpack_message(buffer, temperature);
    }
};


// Helper function to print the message in hex format
void print_message(const char* message) {
    for (int i = 0; i < 5; ++i) {
        printf("%02X ", (unsigned char)message[i]);
    }
    printf("\n");
}


// example usage
int main() {
    TemplateModule template_module(TEMPLATE_MODULE_ID);
    char template_message[5];
    float template_value = 25.6f;
    template_module.generate_message(template_value, template_message);
    std::cout << "Template Message: ";
    print_message(template_message);
    float unpacked_template;
    template_module.unpack_message(template_message, unpacked_template);
    std::cout << "Unpacked Template: " << unpacked_template << std::endl;
    return 0;
}
