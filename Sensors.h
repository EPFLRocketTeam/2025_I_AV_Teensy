// Sensors.h
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
};

// BNO055 Sensor Class
class BNO055Sensor : public Sensor
{
public:
    BNO055Sensor(uint8_t address, TwoWire *wire);

    bool setup() override;
    void calibrate() override;

    const bool updateCalStatus();
    void displayCalStatus();

    BNO055Data readData();

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
    BMPData readData();
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
    int samplesCollected; 
};

#endif //SENSORS_H
