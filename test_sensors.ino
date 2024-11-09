// test.in
// test_sensors.ino

#include <Arduino.h>
#include "Sensors.h"

// Create sensor objects

// BNO055 Sensors
BNO055Sensor bno0(0x28, &Wire, Vec3(0.0, 1.875, 84.375), Vec3(0.0000155833, -0.0000155833, -0.0003133974));
BNO055Sensor bno1(0x29, &Wire, Vec3(0.0, 2.5, 83.75), Vec3(0.0000554660, -0.0001411861, 0.0001042088));
BNO055Sensor bno2(0x29, &Wire1, Vec3(0.0, -0.5, 84.125), Vec3(0.0000536474, 0.0001226227, 0.0002937835));

// BMP581 Sensors
BMP581Sensor bmp0(0x46, &Wire);
BMP581Sensor bmp1(0x47, &Wire);
BMP581Sensor bmp2(0x46, &Wire1);

// GNSS Sensor
GNSSSensor gnss(Serial1);

void setup()
{
    Serial.begin(115200);

    // Initialize I2C buses
    Wire.begin();
    Wire1.begin();

    // Setup sensors
    bno0.setup();
    bno1.setup();
    bno2.setup();

    bmp0.setup();
    bmp1.setup();
    bmp2.setup();

    gnss.setup();

    // Calibrate sensors if necessary
    bno0.calibrate();
    bno1.calibrate();
    bno2.calibrate();

    bmp0.calibrate();
    bmp1.calibrate();
    bmp2.calibrate();

    // No calibration needed for GNSS
}

void loop()
{
    // Read data from BNO055 sensors
    BNO055Data data0 = bno0.readData();
    BNO055Data data1 = bno1.readData();
    BNO055Data data2 = bno2.readData();

    // Collect valid data
    Vec3 total_attitude(0, 0, 0);
    Vec3 total_rate(0, 0, 0);
    int attitude_count = 0;
    int rate_count = 0;

    if (data0.valid_attitude)
    {
        total_attitude += data0.attitude;
        attitude_count++;
    }
    if (data1.valid_attitude)
    {
        total_attitude += data1.attitude;
        attitude_count++;
    }
    if (data2.valid_attitude)
    {
        total_attitude += data2.attitude;
        attitude_count++;
    }

    if (data0.valid_rate)
    {
        total_rate += data0.rate;
        rate_count++;
    }
    if (data1.valid_rate)
    {
        total_rate += data1.rate;
        rate_count++;
    }
    if (data2.valid_rate)
    {
        total_rate += data2.rate;
        rate_count++;
    }

    Vec3 average_attitude;
    Vec3 average_rate;

    if (attitude_count > 0)
    {
        average_attitude = total_attitude / attitude_count;
    }
    else
    {
        average_attitude = Vec3(0, 0, 0); // Default or previous value
    }

    if (rate_count > 0)
    {
        average_rate = total_rate / rate_count;
    }
    else
    {
        average_rate = Vec3(0, 0, 0); // Default or previous value
    }

    // Print attitude and rate
    Serial.print("Attitude: ");
    Serial.print("X: "); Serial.print(average_attitude.x);
    Serial.print(" Y: "); Serial.print(average_attitude.y);
    Serial.print(" Z: "); Serial.println(average_attitude.z);

    Serial.print("Rate: ");
    Serial.print("X: "); Serial.print(average_rate.x);
    Serial.print(" Y: "); Serial.print(average_rate.y);
    Serial.print(" Z: "); Serial.println(average_rate.z);

    // Read data from BMP581 sensors
    float pressure0, temperature0;
    bmp0.readData(pressure0, temperature0);

    float pressure1, temperature1;
    bmp1.readData(pressure1, temperature1);

    float pressure2, temperature2;
    bmp2.readData(pressure2, temperature2);

    // Print pressure and temperature data
    Serial.print("BMP0 Pressure: "); Serial.print(pressure0);
    Serial.print(" Pa, Temperature: "); Serial.print(temperature0); Serial.println(" C");

    Serial.print("BMP1 Pressure: "); Serial.print(pressure1);
    Serial.print(" Pa, Temperature: "); Serial.print(temperature1); Serial.println(" C");

    Serial.print("BMP2 Pressure: "); Serial.print(pressure2);
    Serial.print(" Pa, Temperature: "); Serial.print(temperature2); Serial.println(" C");

    // Read data from GNSS module
    gnss.readData();

    // Print GNSS data
    Serial.print("Latitude: "); Serial.print(gnss.getLatitude(), 7);
    Serial.print(", Longitude: "); Serial.print(gnss.getLongitude(), 7);
    Serial.print(", Altitude: "); Serial.print(gnss.getAltitude(), 2); Serial.println(" m");
    Serial.print("Fix Type: "); Serial.println(gnss.getFixType());

    // Process sensor data as needed
    // ...

    delay(100); // Adjust the delay as necessary
}

