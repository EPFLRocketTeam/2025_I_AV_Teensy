#include <iostream>
#include <vector>
#include "Client.h"
#include "Sensors.h"

ThreeFloatModule bno_gyro;
ThreeFloatModule bno_accel;
OneFloatModule bmp_pres;
OneFloatModule bmp_temp;
Manager manager;
int length = 1 + 12 * 2  + 4 * 2;   

TripleBNO055 bno(0x28, 0x29, 0x29, &Wire, &Wire2);
TripleBMP581 bmp(0x46, 0x47, 0x46, &Wire, &Wire2);

const uint8_t bno_data_sz = 6;
const uint8_t bmp_data_sz = 2;
std::array<float, bno_data_sz> bno_data;
std::array<float, bmp_data_sz> bmp_data;


void initWire() {
    Wire.begin();
    Wire.setSCL(16);
    Wire.setSDA(17);
    Wire.setClock(400000);

    Wire2.begin();
    Wire2.setSCL(24);
    Wire2.setSDA(25);
    Wire2.setClock(400000);
}

void setup() {
    Serial.begin(115200);
    manager.set_module_configuration(0x01, {&bno_gyro, &bno_accel, &bmp_pres, &bmp_temp});
    Serial.println("Initializing serial communication");
    initWire();

    bool bno_status = bno.setup();
    bno.displayStatus();
    Serial.println("Finished initializing BNO");

    bool bmp_status = bmp.setup();
    bmp.displayStatus();
    Serial.println("Finished initializing BMP");

    bno.calibrate();
    Serial.println("Finished calibrating BNO");

    bmp.calibrate();
    Serial.println("Finished calibrating BMP");

}

void loop() {
  bno_data = bno.read_data();
    bmp_data = bmp.read_data(); 
    bno_gyro.set_values(bno_data[0], bno_data[1], bno_data[2]);
    bno_accel.set_values(bno_data[3], bno_data[4], bno_data[5]);
    bmp_pres.set_value(bmp_data[0]);
    bmp_temp.set_value(bmp_data[1]);

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

