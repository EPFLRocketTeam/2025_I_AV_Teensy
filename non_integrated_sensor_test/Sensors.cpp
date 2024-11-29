#include "Sensors.h"

const float Sensor::combineData(float m1, float m2, float m3) const {
    float sum = m1 + m2 + m3;
    float mean = (m1 + m2 + m3) / 3.0;
    float variance = ((m1 - mean) * (m1 - mean) + (m2 - mean) * (m2 - mean) + (m3 - mean) * (m3 - mean)) / 2.0;
    float stdDev = sqrt(variance);
    float threshold = outlier_threshold * stdDev;
    //set threshold to some value manually after some tests ... 
    int outlierCount = 0;
    if (fabs(m1 - mean) > threshold) outlierCount++;
    if (fabs(m2 - mean) > threshold) outlierCount++;
    if (fabs(m3 - mean) > threshold) outlierCount++;

    float finalValue;

    if (outlierCount == 0) {
        // No outliers
        finalValue = mean;
    } else if (outlierCount == 1) {
        // Identify which one is the outlier
        if (fabs(m1 - mean) > threshold) {
            finalValue = (m2 + m3) / 2.0;
        } else if (fabs(m2 - mean) > threshold) {
            finalValue = (m1 + m3) / 2.0;
        } else {
            finalValue = (m1 + m2) / 2.0;
        }
    } else {
        // Multiple outliers
        finalValue = calculateMedian(m1, m2, m3); // Fall back to median
    }
    return finalValue;
}

const float Sensor::calculateMedian(float a, float b, float c) const {
    if ((a >= b && a <= c) || (a <= b && a >= c)) {
        return a;
    } else if ((b >= a && b <= c) || (b <= a && b >= c)) {
        return b;
    } else {
        return c;
    }
}

const bool Sensor::checkAllZeros2Vars(std::array<float, 2> arr) const {
    return (arr[0] == arr[1] == 0.0);
}
const bool Sensor::checkAllZeros6Vars(std::array<float, 6> arr) const {
    return (arr[0] == arr[1] == arr[2] == arr[3] == arr[4] == arr[5] == 0.0);
}
// --------------------
// BNO055Sensor Methods
// --------------------
BNO055Sensor::BNO055Sensor(uint8_t address, TwoWire *wire, uint8_t _delay_time, uint8_t _calibration_threshold, float _outlier_threshold)
    :bno(-1, address, wire), delay_time(_delay_time), calibration_threshold(_calibration_threshold), outlier_threshold(_outlier_threshold)
{}

const bool BNO055Sensor::setup()
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
void BNO055Sensor::displayCalStatus() const{
//to do
}

void BNO055Sensor::calibrate()
{
    //if necessary, according adafruit guide, as soon as you turn it on, it already starts callibrating
    //placeholder stupid calibration:
    while (system_cal_status < calibration_threshold){ // 0 is no calibration, 3 is max, might take some time to calibrate, use 2 i guess, might not be necesssary if no fusion
        updateCalStatus();
        displayCalStatus();
    }
}

// const std::array<float, 6> BNO055Sensor::readData()
// {
//     imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
//     imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

//     return std::array<float, 6> {gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]};
// }

const std::array<float, 6> BNO055Sensor::readData()
{
    delay(delay_time);
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    delay(delay_time);
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

    return std::array<float, 6>{gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]};
}


TripleBNO055::TripleBNO055(uint8_t address1, uint8_t address2, uint8_t address3, TwoWire *wire, TwoWire *wire2, uint8_t delay_time, uint8_t calibration_threshold, float outlier_threshold)
    : sensor1(address1, wire, delay_time, calibration_threshold, outlier_threshold), sensor2(address2, wire, delay_time, calibration_threshold, outlier_threshold), sensor3(address3, wire2, delay_time, calibration_threshold, outlier_threshold)
{}

const bool TripleBNO055::setup() {
    bool success1 = sensor1.setup();
    bool success2 = sensor2.setup();
    bool success3 = sensor3.setup();
    return success1 && success2 && success3;
}

void TripleBNO055::calibrate() {
    sensor1.calibrate();
    sensor2.calibrate();
    sensor3.calibrate();
}

void TripleBNO055::displayStatus() const{
//to do
}

void TripleBNO055::displayCalStatus() const{
//to do
}

const std::array<float, 6> TripleBNO055::read_data(){
    std::array<float, 6> data1 = sensor1.readData();
    std::array<float, 6> data2 = sensor2.readData();
    std::array<float, 6> data3 = sensor3.readData();
    bool allZeros1 = checkAllZeros6Vars(data1);
    bool allZeros2 = checkAllZeros6Vars(data2);
    bool allZeros3 = checkAllZeros6Vars(data3);
    if (allZeros1){
        if (allZeros2)){
            return data3;
        }
        else if (allZeros3){
            return data2;
        }
        else {
            return std::array<float, 6> {
                (data2[0]+data3[0])/2, 
                    (data2[1]+data3[1])/2, 
                    (data2[2]+data3[2])/2, 
                    (data2[3]+data3[3])/2, 
                    (data2[4]+data3[4])/2, 
                    (data2[5]+data3[5])/2, 
            };

        }
    }
    else if (allZeros2){
        if (allZeros3){
            return data1;
        }
        else {
            return std::array<float, 6> {
                (data1[0]+data3[0])/2, 
                    (data1[1]+data3[1])/2, 
                    (data1[2]+data3[2])/2, 
                    (data1[3]+data3[3])/2, 
                    (data1[4]+data3[4])/2, 
                    (data1[5]+data3[5])/2, 
            };
        }
    }
    else if (allZeros3){
        return std::array<float, 6> {
            (data1[0]+data2[0])/2, 
                (data1[1]+data2[1])/2, 
                (data1[2]+data2[2])/2, 
                (data1[3]+data2[3])/2, 
                (data1[4]+data2[4])/2, 
                (data1[5]+data2[5])/2, 
        };
    }
    return std::array<float, 6> {
        combineData(data1[0], data2[0], data3[0]), 
            combineData(data1[1], data2[1], data3[1]), 
            combineData(data1[2], data2[2], data3[2]), 
            combineData(data1[3], data2[3], data3[3]), 
            combineData(data1[4], data2[4], data3[4]), 
            combineData(data1[5], data2[5], data3[5])
    };
}

void TripleBNO055::print_data(const std::array<float, 6>& bno_data) const {
  for (int i(0); i < 6; ++i) {
      Serial.print(bno_data[i]);
      Serial.print(",");
  }
}
// --------------------
// BMP581Sensor Methods
// --------------------

BMP581Sensor::BMP581Sensor(uint8_t address, TwoWire *wire, uint8_t _delay_time, float _outlier_threshold)
    :i2c_address(address), wire(wire), delay_time(_delay_time), outlier_threshold(_outlier_threshold)
{
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
// readData() Method
const BMPData BMP581Sensor::readData() {
    BMPData data{0,0};
    bmp5_sensor_data sensorData;
    delay(delay_time);
    if (bmp.getSensorData(&sensorData) == BMP5_OK) {
        data.pressure = sensorData.pressure;
        data.temperature = sensorData.temperature;
        return data;
    } else {
        return BMPData{0, 0};
    }
}


const float BMP581Sensor::readPressure() const{
    bmp5_sensor_data sensorData;
    delay(delay_time);
    if (bmp.getSensorData(&sensorData) == BMP5_OK) {
        return sensorData.pressure;
    } else {
        return 0.0f;
    }
}

// readTemperature() Method
const float BMP581Sensor::readTemperature() const {
    bmp5_sensor_data sensorData;
    delay(delay_time);
    if (bmp.getSensorData(&sensorData) == BMP5_OK) {
        return sensorData.temperature;
    } else {
        return 0.0f;
    }
}

// enableFilteringAndOversampling() Method
void BMP581Sensor::enableFilteringAndOversampling() {
    // Adjust the methods based on the library's API
    // bmp.setOversamplingPressure(BMP5_OVERSAMPLING_16X);
    // bmp.setOversamplingTemperature(BMP5_OVERSAMPLING_2X);
    bmp.setFilterConfig(BMP5_IIR_FILTER_COEFF_7);
}


TripleBMP581::TripleBMP581(uint8_t address1, uint8_t address2, uint8_t address3, TwoWire *wire, TwoWire *wire2, uint8_t delay_time, float _outlier_threshold)
    : sensor1(address1, wire, delay_time, outlier_threshold), sensor2(address2, wire, delay_time, outlier_threshold), sensor3(address3, wire2, delay_time, outlier_threshold) {}

const bool TripleBMP581::setup() {
    bool success1 = sensor1.setup();
    bool success2 = sensor2.setup();
    bool success3 = sensor3.setup();
    return success1 && success2 && success3;
}

void TripleBMP581::calibrate() {
    sensor1.calibrate();
    sensor2.calibrate();
    sensor3.calibrate();
}

void TripleBMP581::displayStatus() const{
    //to do
}

void TripleBMP581::displayCalStatus() const{
    //to do
}

const std::array<float, 2> TripleBMP581::read_data(){
    BMPData data1 = sensor1.readData();
    BMPData data2 = sensor2.readData();
    BMPData data3 = sensor3.readData();
    bool allZeros1 = checkAllZeros6Vars(data1);
    bool allZeros2 = checkAllZeros6Vars(data2);
    bool allZeros3 = checkAllZeros6Vars(data3);
    if (allZeros1){
        if (allZeros2)){
            return data3;
        }
        else if (allZeros3){
            return data2;
        }
        else {
            return std::array<float, 6> {
                (data2[0]+data3[0])/2, 
                    (data2[1]+data3[1])/2, 
            };
        }
    }
    else if (allZeros2){
        if (allZeros3){
            return data1;
        }
        else {
            return std::array<float, 6> {
                (data1[0]+data3[0])/2, 
                    (data1[1]+data3[1])/2, 
            };
        }
    }
    else if (allZeros3){
        return std::array<float, 6> {
            (data1[0]+data2[0])/2, 
                (data1[1]+data2[1])/2, 
        };
    }
    return std::array<float, 6> {
        combineData(data1[0], data2[0], data3[0]), 
            combineData(data1[1], data2[1], data3[1]), 
    };
}

const float TripleBMP581::read_pressure(){
    float pres1 = sensor1.readPressure();
    float pres2 = sensor2.readPressure();
    float pres3 = sensor3.readPressure();
    return combineData(pres1, pres2, pres3);   
}

void TripleBMP581::print_data(const std::array<float, 2>& bno_data) const {
  for (int i(0); i < 2; ++i) {
      Serial.print(bno_data[i]);
      Serial.print(",");
  }
}
