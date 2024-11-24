#include <iostream>
#include <vector>
#include <boost/asio.hpp>
#include "../Client.h"

using namespace boost::asio;

void read_and_unpack_uart(serial_port &serial, Manager &manager) {
    char data[17];
    while (true) {
        read(serial, buffer(data, 17));
        char identifier;
        std::vector<std::vector<float>> values;
        manager.unpack_combined_message(data, identifier, values);

        std::cout << "Identifier: " << static_cast<int>(identifier) << std::endl;
        for (const auto &module_values : values) {
            std::cout << "Values: ";
            for (const auto &val : module_values) {
                std::cout << val << " ";
            }
            std::cout << std::endl;
        }
    }
}

int main() {
    io_service io;
    serial_port serial(io, "/dev/ttyS0");
    serial.set_option(serial_port_base::baud_rate(9600));

    ThreeFloatModule three_float_module;
    OneFloatModule one_float_module;
    Manager manager;

    manager.add_module(&three_float_module);
    manager.add_module(&one_float_module);

    read_and_unpack_uart(serial, manager);

    return 0;
}