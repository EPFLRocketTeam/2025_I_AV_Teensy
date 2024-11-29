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
    virtual const bool setup() = 0;
    virtual void calibrate() = 0;
    virtual const float get_outlier_threshold() const { return outlier_threshold;}
    virtual void set_outlier_threshold(float _outlier_threshold) { outlier_threshold = _outlier_threshold;}
protected:
    const float calculateMedian(float a, float b, float c) const;
    virtual const float combineData(float m1, float m2, float m3) const;
    const bool checkAllZeros2Vars(std::array<float, 2>) const;
    const bool checkAllZeros6Vars(std::array<float, 6>) const;
    float outlier_threshold;
};

// BNO055 Sensor Class
class BNO055Sensor : public Sensor
{
public:
    BNO055Sensor(uint8_t address, TwoWire *wire, uint8_t delay_time=10, uint8_t calibration_threshold=0, float outlier_threshold = 2);

    const bool setup() override;
    void calibrate() override;

    const bool updateCalStatus();
    void displayCalStatus() const;

    const std::array<float, 6> readData();

private:
    uint8_t delay_time;
    uint8_t calibration_threshold;
    float outlier_threshold = 2;

    Adafruit_BNO055 bno;
    uint8_t system_cal_status;
    uint8_t gyro_cal_status;
    uint8_t accel_cal_status;
    uint8_t mag_cal_status;
};


class TripleBNO055 : public Sensor {
public:
    TripleBNO055(uint8_t address1, uint8_t address2, uint8_t address3, TwoWire *wire, TwoWire *wire2, uint8_t delay_time=10, uint8_t calibration_threshold=0, float outlier_threshold = 2);

    const bool setup() override;
    void calibrate() override;
    void displayCalStatus() const;
    void displayStatus() const;

    // Method to read data from all three sensors
    const std::array<float, 6> read_data();
    void print_data(const std::array<float, 6>&) const;
    BNO055Sensor sensor1;
    BNO055Sensor sensor2;
    BNO055Sensor sensor3;

private:
 
};
// BMP581 Sensor Class
struct BMPData{
    float pressure;
    float temperature;
};
// BMP581Sensor Class
class BMP581Sensor : public Sensor {
public:
    BMP581Sensor(uint8_t address, TwoWire *wire, uint8_t delay_time=20, float outlier_threshold = 2);

    const bool setup() override;
    void calibrate() override;
    const BMPData readData(); // Removed 'const'
    const float readPressure() const;
    const float readTemperature() const;
    void enableFilteringAndOversampling();

private:
    BMP581 bmp;
    uint8_t delay_time;
    float outlier_threshold = 2;

    uint8_t i2c_address;
    TwoWire *wire;
};


class TripleBMP581 : public Sensor {
public:
    TripleBMP581(uint8_t address1, uint8_t address2, uint8_t address3, TwoWire *wire, TwoWire *wire2, uint8_t delay_time = 20, float outlier_threshold = 2);

    const bool setup() override;
    void calibrate() override;
    void displayCalStatus() const;
    void displayStatus() const;
    // Method to read data from all three sensors
    const std::array<float, 2> read_data();
    const float read_pressure();

    void print_data(const std::array<float, 2>&) const;
    BMP581Sensor sensor1;
    BMP581Sensor sensor2;
    BMP581Sensor sensor3;

private:

};

#endif
