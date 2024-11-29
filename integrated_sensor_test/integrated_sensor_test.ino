#include <iostream>
#include <vector>
#include <array>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "Client.h"
#include "Sensors.h"

// Create a SoftwareSerial object
SoftwareSerial mySerial(28, 29); // RX, TX pins

// Sensor modules
ThreeFloatModule bno_gyro;
ThreeFloatModule bno_accel;
OneFloatModule bmp_pres;
OneFloatModule bmp_temp;

// Communication manager
Manager manager;

// Combined message length calculation
constexpr int length = 1 + 12 * 2 + 4 * 2; // Identifier + BNO data (6 floats) + BMP data (2 floats)

// Sensor configurations
TripleBNO055 bno(0x28, 0x29, 0x29, &Wire, &Wire2);
TripleBMP581 bmp(0x46, 0x47, 0x46, &Wire, &Wire2);

// Data sizes and storage
constexpr uint8_t bno_data_sz = 6;
constexpr uint8_t bmp_data_sz = 2;
std::array<float, bno_data_sz> bno_data;
std::array<float, bmp_data_sz> bmp_data;

// Initialize I2C communication
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
    // Initialize SoftwareSerial communication
    Serial.begin(9600);
    Serial.println("Hello world!");
    mySerial.begin(9600);
    
    mySerial.println("Initializing communication...");

    // Initialize I2C communication
    initWire();

    // Configure manager with modules
    manager.set_module_configuration(0x01, {&bno_gyro, &bno_accel, &bmp_pres, &bmp_temp});

    // Initialize BNO sensor
    if (!bno.setup()) {
        mySerial.println("Error initializing BNO sensors!");
    } else {
        bno.displayStatus();
        bno.calibrate();
        mySerial.println("BNO sensors initialized and calibrated.");
    }

    // Initialize BMP sensor
    if (!bmp.setup()) {
        mySerial.println("Error initializing BMP sensors!");
    } else {
        bmp.displayStatus();
        bmp.calibrate();
        mySerial.println("BMP sensors initialized and calibrated.");
    }
}

void loop() {
    // Read data from BNO and BMP sensors
    bno_data = bno.read_data();
    bmp_data = bmp.read_data();
    Serial.print(bno_data[0]);
    Serial.print(" ");
    Serial.print(bno_data[1]);
    Serial.print(" ");
    Serial.println(bno_data[2]);


    // Set values for modules
    bno_gyro.set_values(bno_data[0], bno_data[1], bno_data[2]);
    bno_accel.set_values(bno_data[3], bno_data[4], bno_data[5]);
    bmp_pres.set_value(bmp_data[0]);
    bmp_temp.set_value(bmp_data[1]);

    // Generate combined message
    char combined_message[length];
    manager.generate_combined_message(0x01, combined_message);
    Serial.println(combined_message);
    // Output combined message over SoftwareSerial
    for (int i = 0; i < length; ++i) {
        if (combined_message[i] < 16) Serial.print("0");
        mySerial.print((unsigned char)combined_message[i], HEX);
    }
    mySerial.println();

    // Small delay to prevent overwhelming the serial port
    delay(100);

    // Uncomment below lines for debugging unpacking of messages
    /*
    char identifier;
    std::vector<std::vector<float>> values;
    manager.unpack_combined_message(combined_message, identifier, values);

    std::cout << "Unpacked Identifier: " << static_cast<int>(identifier) << std::endl;
    for (const auto& module_values : values) {
        std::cout << "Values: ";
        for (const auto& val : module_values) {
            std::cout << val << ", ";
        }
        std::cout << std::endl;
    }
    */
}
