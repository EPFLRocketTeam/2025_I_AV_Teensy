#include <Arduino.h>
#include "Sensors.h"

// BNO055 Sensors
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
    Serial.begin(115200);  // Increased baud rate for faster data transfer
    Serial.println("Initializing serial communication");
    // Initialize I2C buses
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

    // Print CSV headers
    Serial.println("Timestamp,BNO1_Gyro_X,BNO1_Gyro_Y,BNO1_Gyro_Z,BNO1_Accel_X,BNO1_Accel_Y,BNO1_Accel_Z, BNO2_Gyro_X,BNO2_Gyro_Y,BNO2_Gyro_Z,BNO2_Accel_X,BNO2_Accel_Y,BNO2_Accel_Z, BNO3_Gyro_X,BNO3_Gyro_Y,BNO3_Gyro_Z,BNO3_Accel_X,BNO3_Accel_Y,BNO3_Accel_Z,BNO_Gyro_X,BNO_Gyro_Y,BNO_Gyro_Z,BNO_Accel_X,BNO_Accel_Y,BNO_Accel_Z,BMP_Pressure,BMP_Temperature, Temperature1, Temperature2, Temperature3");
}

void loop() {
    // Read data from sensors
    bno_data = bno.read_data();
    bmp_data = bmp.read_data();
    bno_data1 = bno.sensor1.readData();
    bno_data2 = bno.sensor2.readData();
    bno_data3 = bno.sensor3.readData();
    float pres1 = bmp.sensor1.readPressure();
    float pres2 = bmp.sensor2.readPressure();
    float pres3 = bmp.sensor2.readPressure();


    // Get the current timestamp
    unsigned long timestamp = millis();

    // Print data in CSV format
    Serial.print(timestamp);
    Serial.print(',');

      
    for (size_t i = 0; i < bno_data1.size(); ++i) {
        Serial.print(bno_data1[i], 6);  // 6 decimal places for precision
        Serial.print(',');
    }
    for (size_t i = 0; i < bno_data2.size(); ++i) {
        Serial.print(bno_data2[i], 6);  // 6 decimal places for precision
        Serial.print(',');
    }
    for (size_t i = 0; i < bno_data3.size(); ++i) {
        Serial.print(bno_data3[i], 6);  // 6 decimal places for precision
        Serial.print(',');
    }

  
    for (size_t i = 0; i < bno_data.size(); ++i) {
        Serial.print(bno_data[i], 6);  // 6 decimal places for precision
        Serial.print(',');
    }
    for (size_t i = 0; i < bmp_data.size(); ++i) {
        Serial.print(bmp_data[i], 6);
        Serial.print(',');
    }
    Serial.print(pres1);
    Serial.print(',');
    Serial.print(pres2);
    Serial.print(',');
    Serial.print(pres3);
    Serial.println();  // End of data line
}
