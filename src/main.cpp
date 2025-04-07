#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SparkFun_BMP581_Arduino_Library.h>
#include <utils.h>
#include <iostream>
#include <vector>
#include <math.h>

#include "god.h"
#include "../lib/navigation/navigation.h"

using namespace std;

GOD* god;

Adafruit_BNO055 bno1(-1, 0x28, &Wire), bno2(-1, 0x28, &Wire), bno3(-1, 0x28, &Wire);
BMP581 bmp1, bmp2, bmp3;
Navigation nav;

vector<double> read_data(const bool print = false) {
    double time = millis();

    // read bmp data
    bmp5_sensor_data data1 = {0,0}, data2 = {0,0}, data3 = {0,0};
    bmp1.getSensorData(&data1); bmp2.getSensorData(&data2); bmp3.getSensorData(&data3);
    double baro1 = data1.pressure, baro2 = data2.pressure, baro3 = data3.pressure;
    double temperature = data1.temperature;

    // read bno data
    sensors_event_t  angVelocityData1, angVelocityData2, angVelocityData3;
    bno1.getEvent(&angVelocityData1, Adafruit_BNO055::VECTOR_GYROSCOPE); 
    bno2.getEvent(&angVelocityData2, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno3.getEvent(&angVelocityData3, Adafruit_BNO055::VECTOR_GYROSCOPE);
    double gyroX1 = angVelocityData1.gyro.x, gyroY1 = angVelocityData1.gyro.y, gyroZ1 = angVelocityData1.gyro.z;
    double gyroX2 = angVelocityData2.gyro.x, gyroY2 = angVelocityData2.gyro.y, gyroZ2 = angVelocityData2.gyro.z;
    double gyroX3 = angVelocityData3.gyro.x, gyroY3 = angVelocityData3.gyro.y, gyroZ3 = angVelocityData3.gyro.z;

    double gps_latitude = 0.0, gps_longitude = 0.0, gps_altitude = 0.0; // to change

    // print data
    if (print) {
        Serial.print("Time: "); Serial.print(time); Serial.print(" ms, ");
        Serial.print("Baro1: "); Serial.print(baro1); Serial.print(" Pa, ");
        Serial.print("Baro2: "); Serial.print(baro2); Serial.print(" Pa, ");
        Serial.print("Baro3: "); Serial.print(baro3); Serial.print(" Pa, ");
        Serial.print("Temperature: "); Serial.print(temperature); Serial.print(" C, ");
        Serial.print("GPS Latitude: "); Serial.print(gps_latitude); Serial.print(", ");
        Serial.print("GPS Longitude: "); Serial.print(gps_longitude); Serial.print(", ");
        Serial.print("GPS Altitude: "); Serial.print(gps_altitude); Serial.print(", ");
        Serial.print("GyroX1: "); Serial.print(gyroX1); Serial.print(", ");
        Serial.print("GyroY1: "); Serial.print(gyroY1); Serial.print(", ");
        Serial.print("GyroZ1: "); Serial.println(gyroZ1);
    }

    if (temperature < 0.1) temperature = 14.5;

    // Serial.print("Baro1: "); 
    Serial.print(baro1); 
    Serial.print(" ");
    // Serial.print("Baro2: "); 
    Serial.print(baro2); 
    Serial.print(" ");
    // Serial.print("Baro3: "); 
    Serial.print(baro3); 
    Serial.print(" ");
    // Serial.print("Temp: "); 
    Serial.print(temperature); 
    Serial.print(" ");
    // Serial.println();   

    return {time, baro1, baro2, baro3, temperature + 273.15, gps_latitude, gps_longitude, gps_altitude, gyroX1, gyroY1, gyroZ1, gyroX2, gyroY2, gyroZ2, gyroX3, gyroY3, gyroZ3};
}

void setup(void)
{
    Serial.begin(115200); // 110, 300, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200
    Serial.println("serial complete");
    Wire.begin();
    Wire1.begin();
    Wire2.begin();

    while (!bno1.begin()) {
        Serial.println("Ooops, no BNO055 1 detected ... Check your wiring or I2C ADDR!"); 
        delay(100);
    }
    Serial.println("bno 1 complete");
    while (!bno2.begin()) {
        Serial.println("Ooops, no BNO055 2 detected ... Check your wiring or I2C ADDR!"); 
        delay(100);
    }
    Serial.println("bno 2 complete");
    while (!bno3.begin()) {
        Serial.println("Ooops, no BNO055 3 detected ... Check your wiring or I2C ADDR!"); 
        delay(100);
    }
    Serial.println("bno 3 complete");
    while(bmp1.beginI2C(0x47) != BMP5_OK) {
        Serial.println("Error: BMP581 1 not connected, check wiring and I2C address!"); 
        delay(100);
    }
    Serial.println("bmp 1 complete");
    while(bmp2.beginI2C(0x47) != BMP5_OK) {
        Serial.println("Error: BMP581 2 not connected, check wiring and I2C address!"); 
        delay(100);
    }
    Serial.println("bmp 2 complete");
    while(bmp3.beginI2C(0x47) != BMP5_OK) {
        Serial.println("Error: BMP581 3 not connected, check wiring and I2C address!"); 
        delay(100);
    }
    Serial.println("bmp 3 complete");

    for (int i = 0; i < 5; ++i) {
        read_data();
        delay(1000);
    }

    double p0 = 0;
    int count  = 0;

    for (int i = 0; i < 100; ++i) {
        double p = read_data()[1];
        if (p > 100) {
            p0 += p;
            count++;
        }
        delay(10);
    }

    p0 = p0 / count;

    // nav = Navigation(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    vector<double> data = read_data();
    data[1] = p0;
    data[2] = p0;
    data[3] = p0;
    nav.init(data);
    // vector<double> state = nav.get_state();
    // Serial.print("X = ");
    // Serial.print(state[0]);
    
    Serial.println("Setup complete");
}

void loop(void)
{ 
    // Serial.println("test");
    // blink led
    // digitalWrite(LED_BUILTIN, HIGH);
    // delay(500);
    // digitalWrite(LED_BUILTIN, LOW);
    // delay(500);

    // read_data();
    std::vector<double> data = read_data();
    // data = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    // Serial.print(data[1]); Serial.print(data[2]); Serial.print(data[3]); Serial.println();
    nav.update(data);

    // Print the state
    vector<double> state = nav.get_state();
    // Serial.print("X = ");
    // Serial.print(state[0]);
    // Serial.print(" Y = ");
    // Serial.print(state[1]);
    // Serial.print(" Z = ");
    Serial.print(state[2]);
    // Serial.print(" VX = ");
    // Serial.print(state[3]);
    // Serial.print(" VY = ");
    // Serial.print(state[4]);
    // Serial.print(" VZ = ");
    // Serial.print(state[5]);
    // Serial.print(" AX = ");
    // Serial.print(state[6]);
    // Serial.print(" AY = ");
    // Serial.print(state[7]);
    // Serial.print(" AZ = ");
    // Serial.print(state[8]);
    // Serial.print(" OX = ");
    // Serial.print(state[9]);
    // Serial.print(" OY = ");
    // Serial.print(state[10]);
    // Serial.print(" OZ = ");
    // Serial.print(state[11]);
    // Serial.print(" WX = ");
    // Serial.print(state[12]);
    // Serial.print(" WY = ");
    // Serial.print(state[13]);
    // Serial.print(" WZ = ");
    // Serial.print(state[14]);
    Serial.println();

    delay(10);
}
