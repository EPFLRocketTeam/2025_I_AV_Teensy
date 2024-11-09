// Sensors.h
#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_BMP581_Arduino_Library.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

// Define Vec3 structure if not already defined
struct Vec3
{
    float x;
    float y;
    float z;

    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator+(const Vec3 &other) const
    {
        return Vec3(x + other.x, y + other.y, z + other.z);
    }

    Vec3 &operator+=(const Vec3 &other)
    {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    Vec3 operator-(const Vec3 &other) const
    {
        return Vec3(x - other.x, y - other.y, z - other.z);
    }

    Vec3 operator/(float scalar) const
    {
        return Vec3(x / scalar, y / scalar, z / scalar);
    }

    Vec3 toDegree() const
    {
        return Vec3(x * RAD_TO_DEG, y * RAD_TO_DEG, z * RAD_TO_DEG);
    }
};

// Structure to hold BNO055 sensor data
struct BNO055Data
{
    Vec3 attitude;
    Vec3 rate;
    bool valid_attitude;
    bool valid_rate;

    BNO055Data() : valid_attitude(false), valid_rate(false) {}
};

// Base Sensor class (optional for common functionality)
class Sensor
{
public:
    virtual void setup() = 0;
    virtual void calibrate() = 0;
};

// BNO055 Sensor Class
class BNO055Sensor : public Sensor
{
public:
    BNO055Sensor(uint8_t address, TwoWire *wire, Vec3 att_calib, Vec3 rate_calib);

    void setup() override;
    void calibrate() override;
    BNO055Data readData();

private:
    Adafruit_BNO055 bno;
    Vec3 att_calibration;
    Vec3 rate_calibration;
};

// BMP581 Sensor Class
class BMP581Sensor : public Sensor
{
public:
    BMP581Sensor(uint8_t address, TwoWire *wire);

    void setup() override;
    void calibrate() override;
    void readData(float &pressure, float &temperature);

private:
    BMP581 bmp;
    uint8_t i2c_address;
    TwoWire *wire;
};

// GNSS Sensor Class
class GNSSSensor : public Sensor
{
public:
    GNSSSensor(HardwareSerial &serialPort);

    void setup() override;
    void calibrate() override; // Calibration not typically needed for GNSS
    void readData();

    // Accessor methods for GNSS data
    double getLatitude() const;
    double getLongitude() const;
    double getAltitude() const;
    uint8_t getFixType() const;

private:
    SFE_UBLOX_GNSS gnss;
    HardwareSerial &serial;
    double latitude;
    double longitude;
    double altitude;
    uint8_t fixType;
};

#endif // SENSORS_H

