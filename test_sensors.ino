#include <Arduino.h>
#include "Sensors.h"
// Create sensor objects

// BNO055 Sensors
BNO055Sensor bno0(0x28, &Wire);
BNO055Sensor bno1(0x29, &Wire);
BNO055Sensor bno2(0x29, &Wire1);

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
    imu::Vector<3> orientation_sum(0, 0, 0);
    imu::Vector<3> gyro_sum(0, 0, 0);
    imu::Vector<3> accel_sum(0, 0, 0);

    int orientation_count = 0;
    int acceleration_count = 0;
    int gyro_count = 0;

    orientation_sum += data0.orientation;
    orientation_sum += data1.orientation;
    orientation_sum += data2.orientation;
    orientation_count += 3;
    gyro_sum += data0.gyro;
    gyro_sum += data1.gyro;
    gyro_sum += data2.gyro;
    gyro_count += 3;
    accel_sum += data0.acceleration;
    accel_sum += data1.acceleration;
    accel_sum += data2.acceleration;
    acceleration_count += 3;

    imu::Vector<3> avg_orientation;
    imu::Vector<3> avg_accel;
    imu::Vector<3> avg_gyro;

    if (orientation_count > 0)
    {
        avg_orientation = orientation_sum / orientation_count;
    }
    else
    {
        avg_orientation = imu::Vector<3>(0, 0, 0); // Default or previous value
    }

    if (gyro_count > 0)
    {
         avg_gyro = gyro_sum / gyro_count;
    }
    else
    {
        avg_gyro = imu::Vector<3>(0, 0, 0); // Default or previous value
    }

    if (acceleration_count > 0)
    {
        avg_accel = accel_sum / acceleration_count;
    }
    else
    {
        avg_accel = imu::Vector<3>(0, 0, 0); // Default or previous value
    }

    // Print attitude and rate
    Serial.print("Orientation: ");
    Serial.print("X: "); Serial.print(avg_orientation.x);
    Serial.print(" Y: "); Serial.print(avg_orientation.y);
    Serial.print(" Z: "); Serial.println(avg_orientation.z);

    Serial.print("Acceleration: ");
    Serial.print("X: "); Serial.print(avg_accel.x);
    Serial.print(" Y: "); Serial.print(avg_accel.y);
    Serial.print(" Z: "); Serial.println(avg_accel.z);

    Serial.print("Gyroscope: ");
    Serial.print("X: "); Serial.print(avg_gyro.x);
    Serial.print(" Y: "); Serial.print(avg_gyro.y);
    Serial.print(" Z: "); Serial.println(avg_gyro.z);


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

    delay(100); // Adjust the delay as necessary
}

