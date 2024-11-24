// Sensors.cp// Sensors.h
#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_BMP581_Arduino_Library.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
// Structure to hold BNO055 sensor data
struct BNO055Data
{
    imu::Vector<3> orientation;
    imu::Vector<3> gyroscope;
    imu::Vector<3> acceleration;
};

class Sensor
{
public:
    virtual bool setup() = 0;
    virtual void calibrate() = 0;
protected:
    const float calculateMedian(float a, float b, float c) const;
    const float combineData(float m1, float m2, float m3) const;
};

// BNO055 Sensor Class
class BNO055Sensor : public Sensor
{
public:
    BNO055Sensor(uint8_t address, TwoWire *wire);

    bool setup() override;
    void calibrate() override;

    const bool updateCalStatus();
    void displayCalStatus() const;

    BNO055Data readData();
    BNO055Data readRawData();

    imu::Vector<3> readAccelRaw();
    imu::Vector<3> readGyroRaw();
    imu::Vector<3> readMagnetometerRaw();

private:
    Adafruit_BNO055 bno;
    uint8_t system_cal_status;
    uint8_t gyro_cal_status;
    uint8_t accel_cal_status;
    uint8_t mag_cal_status;
};


class TripleBNO055 : public Sensor {
public:
    TripleBNO055(uint8_t address1, uint8_t address2, uint8_t address3, TwoWire *wire, TwoWire *wire2);

    bool setup() override;
    void calibrate() override;
    void displayCalStatus() const;
    void displayStatus() const;

    // Method to read data from all three sensors
    std::array<float, 9> read_data();
    void print_data(const std::array<float, 9>&) const;

private:
    BNO055Sensor sensor1;
    BNO055Sensor sensor2;
    BNO055Sensor sensor3;
    std::array<BNO055Data, 3> readAllData();
    //getter for specific sensorreadDa
    BNO055Sensor& getSensor(uint8_t index);
    // Individual methods to access specific sensors
    BNO055Data readSensorData(uint8_t index);
    imu::Vector<3> readAccelRaw(uint8_t index);
    imu::Vector<3> readGyroRaw(uint8_t index);
    imu::Vector<3> readMagnetometerRaw(uint8_t index);
};
// BMP581 Sensor Class
struct BMPData{
    float pressure;
    float temperature;
};
// BMP581Sensor Class
class BMP581Sensor : public Sensor {
public:
    BMP581Sensor(uint8_t address, TwoWire *wire);

    bool setup() override;
    void calibrate() override;
    BMPData readData(); // Removed 'const'
    float readPressure();
    const float readTemperature() const;
    float getPressureFiltered() const;
    void enableFilteringAndOversampling();

private:
    BMP581 bmp;
    uint8_t i2c_address;
    TwoWire *wire;
    static constexpr int bufferSize = 10;
    int bufferIndex;
    float pressureBuffer[bufferSize];
    float pressureSum;
    int samplesCollected; // Added this variable
};


class TripleBMP581 : public Sensor {
public:
    TripleBMP581(uint8_t address1, uint8_t address2, uint8_t address3, TwoWire *wire, TwoWire *wire2);

    bool setup() override;
    void calibrate() override;
    void displayCalStatus() const;
    void displayStatus() const;
    // Method to read data from all three sensors
    std::array<float, 2> read_data();
    void print_data(const std::array<float, 2>&) const;

private:
    BMP581Sensor sensor1;
    BMP581Sensor sensor2;
    BMP581Sensor sensor3;
    std::array<BMPData, 3> readAllData();
    //getter for specific sensor
    BMP581Sensor& getSensor(uint8_t index);
    // Individual methods to access specific sensors
    BMPData readSensorData(uint8_t index);
};
dif
