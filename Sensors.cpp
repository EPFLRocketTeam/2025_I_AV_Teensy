#include "Sensors.h"

const float Sensor::combineData(float m1, float m2, float m3) const {
    float mean = (m1 + m2 + m3) / 3.0;
    float variance = ((m1 - mean) * (m1 - mean) + (m2 - mean) * (m2 - mean) + (m3 - mean) * (m3 - mean)) / 2.0;
    float stdDev = sqrt(variance);
    float threshold = 2 * stdDev;
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
// --------------------
// BNO055Sensor Methods
// --------------------
BNO055Sensor::BNO055Sensor(uint8_t address, TwoWire *wire)
    :bno(-1, address, wire)
{}

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
void BNO055Sensor::displayCalStatus() const{
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

TripleBNO055::TripleBNO055(uint8_t address1, uint8_t address2, uint8_t address3, TwoWire *wire, TwoWire *wire2)
    : sensor1(address1, wire), sensor2(address2, wire), sensor3(address3, wire2)
{}

bool TripleBNO055::setup() {
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

std::array<float, 9> TripleBNO055::read_data(){
    std::array<BNO055Data, 3> data = readAllData();        
    return std::array<float, 9> {
        combineData(data[0].acceleration.x(), data[1].acceleration.x(), data[2].acceleration.x()), 
        combineData(data[0].acceleration.y(), data[1].acceleration.y(), data[2].acceleration.y()), 
        combineData(data[0].acceleration.z(), data[1].acceleration.z(), data[2].acceleration.z()), 
        combineData(data[0].orientation.x(), data[1].orientation.x(), data[2].orientation.x()), 
        combineData(data[0].orientation.y(), data[1].orientation.y(), data[2].acceleration.y()), 
        combineData(data[0].orientation.z(), data[1].orientation.z(), data[2].orientation.z()), 
        combineData(data[0].gyroscope.x(), data[1].gyroscope.x(), data[2].gyroscope.x()), 
        combineData(data[0].gyroscope.y(), data[1].gyroscope.y(), data[2].gyroscope.y()), 
        combineData(data[0].gyroscope.z(), data[1].gyroscope.z(), data[2].gyroscope.z()), 
    };
}

std::array<BNO055Data, 3> TripleBNO055::readAllData(){
    return std::array<BNO055Data, 3> {sensor1.readData(), sensor2.readData(), sensor3.readData()};    
}


BNO055Data TripleBNO055::readSensorData(uint8_t index) {
    return getSensor(index).readData();
}

imu::Vector<3> TripleBNO055::readAccelRaw(uint8_t index) {
    return getSensor(index).readAccelRaw();
}

imu::Vector<3> TripleBNO055::readGyroRaw(uint8_t index) {
    return getSensor(index).readGyroRaw();
}

imu::Vector<3> TripleBNO055::readMagnetometerRaw(uint8_t index) {
    return getSensor(index).readMagnetometerRaw();
}

BNO055Sensor& TripleBNO055::getSensor(uint8_t index) {
    switch (index) {
        case 0:
            return sensor1;
        case 1:
            return sensor2;
        case 2:
            return sensor3;
        default:
            return sensor1;
    }
}

void TripleBNO055::print_data(const std::array<float, 9>& bno_data) const {
  Serial.print("<");
  for (int i(0); i < 9 - 1; ++i) {
      Serial.print(bno_data[i]);
      Serial.print(",");
  }
  Serial.print(bno_data[9-1]);
  Serial.print(">");
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


TripleBMP581::TripleBMP581(uint8_t address1, uint8_t address2, uint8_t address3, TwoWire *wire, TwoWire *wire2)
    : sensor1(address1, wire), sensor2(address2, wire), sensor3(address3, wire2) {}

bool TripleBMP581::setup() {
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

std::array<float, 2> TripleBMP581::read_data(){
    std::array<BMPData, 3> data = readAllData();        
    return std::array<float, 2> {
        combineData(data[0].pressure, data[1].pressure, data[2].pressure), 
        combineData(data[0].temperature, data[1].temperature, data[2].temperature), 
    };
}
std::array<BMPData, 3> TripleBMP581::readAllData(){
    return std::array<BMPData, 3> {sensor1.readData(), sensor2.readData(), sensor3.readData()};    
}

BMPData TripleBMP581::readSensorData(uint8_t index) {
    return getSensor(index).readData();
}

BMP581Sensor& TripleBMP581::getSensor(uint8_t index) {
    switch (index) {
        case 0:
            return sensor1;
        case 1:
            return sensor2;
        case 2:
            return sensor3;
        default:
            return sensor1;
    }
}
void TripleBMP581::print_data(const std::array<float, 2>& bno_data) const {
  Serial.print("<");
  for (int i(0); i < 2 - 1; ++i) {
      Serial.print(bno_data[i]);
      Serial.print(",");
  }
  Serial.print(bno_data[2-1]);
  Serial.print(">");
}

