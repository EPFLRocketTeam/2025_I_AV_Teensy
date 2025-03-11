#include "../Client.h"
#include <Arduino.h>
#include "../Sensors.h"

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
std::array<float, bno_data_sz> bno_data1;
std::array<float, bno_data_sz> bno_data2;
std::array<float, bno_data_sz> bno_data3;


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
    manager.add_module(&bno_gyro);
    manager.add_module(&bno_accel);
    manager.add_module(&bmp_pres);
    manager.add_module(&bmp_temp);


    Serial.begin(115200);  // Increased baud rate for faster data transfer
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
    bno_pres.set_value(bmp_data.pressure);
    bno_temp.set_value(bmp_data.temperature);

    char combined_message[length];
    manager.generate_combined_message(0x01, combined_message);
    Serial.write(combined_message, length);

    delay(1000);

    if (Serial.available() >= length) {
        char received_message[length];
        Serial.readBytes(received_message, length);

        char identifier;
        std::vector<std::vector<float>> values;
        manager.unpack_combined_message(received_message, identifier, values);

        Serial.print("Identifier: ");
        Serial.println(identifier, HEX);
        for (const auto &module_values : values) {
            Serial.print("Values: ");
            for (const auto &val : module_values) {
                Serial.print(val);
                Serial.print(", ");
            }
            Serial.println();
        }
    }
}
