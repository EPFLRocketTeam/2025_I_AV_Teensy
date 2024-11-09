// Sensors.cpp
#include "Sensors.h"

// --------------------
// BNO055Sensor Methods
// --------------------
BNO055Sensor::BNO055Sensor(uint8_t address, TwoWire *wire);
    : bno(55, address, wire)
{
}

bool BNO055Sensor::setup()
{
    if (!bno.begin()) return 0;
    delay(1000);
    bno.setExtCrystalUse(true);

    updateCalStatus();

    return 1;
}

const bool updateCalStatus(){
    bno.getCalibration(&system_cal_status,
                        &gyro_cal_status, 
                        &accel_cal_status, &mag_cal_status);
    return system_cal_status > 0;
}
void displayCalStatus(){
//to do
}

void BNO055Sensor::calibrate()
{
    //if necessary, according adafruit guide, as soon as you turn it on, it already starts callibrating
    //placeholder stupid calibration:
    while (system_cal_status < 2){
        updateCalStatus();
        displayCalStatus();
    }
}

const BNO055Data BNO055Sensor::readData() const
{
    imu::Vector<3> orient = bno.getEvent(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> gyro = bno.getEvent(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> accel = bno.getEvent(Adafruit_BNO055::VECTOR_LIENARACCEL);

    return BNO055Data{orient, gyro, accel};
}

const imu::Vector<3> readAccelRaw() const {
    return bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
}
const imu::Vector<3> readGyroRaw() const {
    return bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
}
const imu::Vector<3> readMagnetometerRaw() const {
    return bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
}
// --------------------
// BMP581Sensor Methods
// --------------------

BMP581Sensor::BMP581Sensor(uint8_t address, TwoWire *wire)
    : i2c_address(address), wire(wire), bufferIndex(0)
{

    std::fill(std::begin(pressureBuffer), std::end(pressureBuffer), 0.0f);
}

const bool BMP581Sensor::setup()
{
    if (bmp.beginI2C(i2c_address, *wire) != BMP5_OK) return 0;
    return 1;
}

void BMP581Sensor::calibrate()
{
    // Implement calibration if necessary
}

const BMPData BMP581Sensor::readData() const
{
    BMPData data{0,0};
    if (bmp.getSensorData(&data) == BMP5_OK) { return data; }
    else { return BMPData{0, 0}; }
}


const float BMP581Sensor::readPressure() {
    BMPData data{0,0};
    if (bmp.getSensorData(&data) == BMP5_OK) { 
        if (bufferSize == 10) {
            // Subtract the value that is being overwritten from the sum
            pressureSum -= pressureBuffer[bufferIndex];
        } else {
            bufferSize++;
        }

        // Add the new pressure reading to the buffer and update the sum
        pressureBuffer[bufferIndex] = data.pressure;
        pressureSum += data.pressure;

        // Update the buffer index in a circular manner
        bufferIndex = (bufferIndex + 1) % 10;

        return data.pressure; 
    } else { 
        return 0; 
    }
}
const float BMP581Sensor::getPressureFiltered() const {
    if (bufferSize == 0) return 0;
    return pressureSum / bufferSize;
}

const float BMP581Sensor::readTemperature() const{
    BMPData data{0,0};
    if (bmp.getSensorData(&data) == BMP5_OK) { return data.temperature; }
    else { return 0; }
}

void BMP581Sensor::enableFilteringAndOversampling() {
    //these settings should be adjusted manually according to requirements
    bmp.setPressureOversampling(BMP5_OVERSAMPLING_16X);
    bmp.setTemperatureOversampling(BMP5_OVERSAMPLING_2X);

    // Enable IIR filter
    bmp.setIIRFilterCoefficient(BMP5_IIR_FILTER_COEFF_7);
}
