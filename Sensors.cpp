
// Sensors.cpp

#include "Sensors.h"

// --------------------
// BNO055Sensor Methods
// --------------------

BNO055Sensor::BNO055Sensor(uint8_t address, TwoWire *wire, Vec3 att_calib, Vec3 rate_calib)
    : bno(55, address, wire), att_calibration(att_calib), rate_calibration(rate_calib)
{
}

void BNO055Sensor::setup()
{
    if (!bno.begin())
    {
        Serial.print("Failed to initialize BNO055 sensor at address 0x");
        Serial.println(bno.getAddress(), HEX);
    }
    else
    {
        // Set axis remap and sign (adjust as needed)
        bno.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P6);
        bno.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P6);
    }
}

void BNO055Sensor::calibrate()
{
    // Implement calibration routine if necessary
    // For now, using predefined calibration data
}

BNO055Data BNO055Sensor::readData()
{
    BNO055Data data;

    sensors_event_t orientationData, angVelocityData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);

    const sensors_vec_t &o = orientationData.orientation;
    Vec3 orientation{o.x, o.y, o.z};
    if (orientation.x != 0 || orientation.y != 0 || orientation.z != 0)
    {
        orientation = orientation - att_calibration;
        data.attitude = Vec3{orientation.y, -orientation.z, orientation.x};
        data.valid_attitude = true;
    }

    const sensors_vec_t &g = angVelocityData.gyro;
    Vec3 gyro{g.x, g.y, g.z};
    if (gyro.x != 0 || gyro.y != 0 || gyro.z != 0)
    {
        gyro = gyro - rate_calibration;
        gyro = gyro.toDegree();
        data.rate = Vec3{-gyro.z, gyro.x, gyro.y};
        data.valid_rate = true;
    }

    return data;
}

// --------------------
// BMP581Sensor Methods
// --------------------

BMP581Sensor::BMP581Sensor(uint8_t address, TwoWire *wire)
    : i2c_address(address), wire(wire)
{
}

void BMP581Sensor::setup()
{
    if (bmp.beginI2C(i2c_address, *wire) != BMP5_OK)
    {
        Serial.print("Failed to initialize BMP581 sensor at address 0x");
        Serial.println(i2c_address, HEX);
    }
}

void BMP581Sensor::calibrate()
{
    // Implement calibration routine if necessary
}

void BMP581Sensor::readData(float &pressure, float &temperature)
{
    bmp5_sensor_data data;
    if (bmp.getSensorData(&data) == BMP5_OK)
    {
        pressure = data.pressure;
        temperature = data.temperature;
    }
    else
    {
        Serial.print("Failed to read data from BMP581 sensor at address 0x");
        Serial.println(i2c_address, HEX);
    }
}

// --------------------
// GNSSSensor Methods
// --------------------

GNSSSensor::GNSSSensor(HardwareSerial &serialPort)
    : serial(serialPort), latitude(0), longitude(0), altitude(0), fixType(0)
{
}

void GNSSSensor::setup()
{
    serial.begin(38400);
    if (!gnss.begin(serial))
    {
        Serial.println("Failed to initialize u-blox GNSS module");
    }
    else
    {
        Serial.println("u-blox GNSS module initialized");

        // Configure GNSS settings if necessary
        gnss.setNavigationFrequency(10);               // Set update rate to 10 Hz
        gnss.setUART1Output(COM_TYPE_UBX);             // Set output to UBX protocol
    }
}

void GNSSSensor::calibrate()
{
    // Calibration not typically needed for GNSS modules
}

void GNSSSensor::readData()
{
    if (gnss.getPVT())
    {
        long lat = gnss.getLatitude();
        long lon = gnss.getLongitude();
        long alt = gnss.getAltitude();
        fixType = gnss.getFixType();

        latitude = lat / 1e7;
        longitude = lon / 1e7;
        altitude = alt / 1000.0; // Convert mm to meters
    }
}

double GNSSSensor::getLatitude() const
{
    return latitude;
}

double GNSSSensor::getLongitude() const
{
    return longitude;
}

double GNSSSensor::getAltitude() const
{
    return altitude;
}

uint8_t GNSSSensor::getFixType() const
{
    return fixType;
}

