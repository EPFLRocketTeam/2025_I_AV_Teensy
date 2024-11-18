// Sensors.cpp
#include "Sensors.h"

// --------------------
// BNO055Sensor Methods
// --------------------
BNO055Sensor::BNO055Sensor(uint8_t address, TwoWire *wire)
    :bno(55, address, wire)
{
}

bool BNO055Sensor::setup()
{
    if (!bno.begin()) return 0;
    delay(1000);
    // bno.setExtCrystalUse(false);
    updateCalStatus();
    
    return 1;
}

const bool BNO055Sensor::updateCalStatus(){
    bno.getCalibration(&system_cal_status,
                        &gyro_cal_status, 
                        &accel_cal_status, &mag_cal_status);
    return system_cal_status > 0;
}
void BNO055Sensor::displayCalStatus(){
//to do
}

void BNO055Sensor::calibrate()
{
    //if necessary, according adafruit guide, as soon as you turn it on, it already starts callibrating
    //placeholder stupid calibration:
    while (system_cal_status < 0){
        updateCalStatus();
        displayCalStatus();
    }
}

BNO055Data BNO055Sensor::readData()
{
    imu::Vector<3> orient = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

    return BNO055Data{orient, gyro, accel};
}

imu::Vector<3> BNO055Sensor::readAccelRaw() {
    return bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
}
imu::Vector<3> BNO055Sensor::readGyroRaw() {
    return bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
}
imu::Vector<3> BNO055Sensor::readMagnetometerRaw() {
    return bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
}
// --------------------
// BMP581Sensor Methods
// --------------------

BMP581Sensor::BMP581Sensor(uint8_t address, TwoWire *wire)
    :i2c_address(address), wire(wire), bufferIndex(0)
{

    std::fill(std::begin(pressureBuffer), std::end(pressureBuffer), 0.0f);
}

bool BMP581Sensor::setup()
{
    if (bmp.beginI2C(i2c_address, *wire) != BMP5_OK) return 0;
    return 1;
}

void BMP581Sensor::calibrate()
{
    // Implement calibration if necessary
}
// readData() Method
BMPData BMP581Sensor::readData() {
    BMPData data{0,0};
    bmp5_sensor_data sensorData;

    if (bmp.getSensorData(&sensorData) == BMP5_OK) {
        data.pressure = sensorData.pressure;
        data.temperature = sensorData.temperature;
        return data;
    } else {
        return BMPData{0, 0};
    }
}

// readPressure() Method
float BMP581Sensor::readPressure() {
    bmp5_sensor_data sensorData;
    if (bmp.getSensorData(&sensorData) == BMP5_OK) {
        float pressure = sensorData.pressure;

        pressureSum -= pressureBuffer[bufferIndex];
        pressureBuffer[bufferIndex] = pressure;
        pressureSum += pressure;

        bufferIndex = (bufferIndex + 1) % bufferSize;

        if (samplesCollected < bufferSize) {
            samplesCollected++;
        }

        return pressure;
    } else {
        return 0.0f;
    }
}

// readTemperature() Method
const float BMP581Sensor::readTemperature() const {
    BMPData data = readData();
    return data.temperature;
}

// enableFilteringAndOversampling() Method
void BMP581Sensor::enableFilteringAndOversampling() {
    // Adjust the methods based on the library's API
    // bmp.setOversamplingPressure(BMP5_OVERSAMPLING_16X);
    // bmp.setOversamplingTemperature(BMP5_OVERSAMPLING_2X);
    bmp.setFilterConfig(BMP5_IIR_FILTER_COEFF_7);
}
