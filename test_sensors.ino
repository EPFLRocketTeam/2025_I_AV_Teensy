#include <Arduino.h>
#include "Sensors.h"

// BNO055 Sensors
BNO055Sensor bno0(0x28, &Wire);
BNO055Sensor bno1(0x29, &Wire);
BNO055Sensor bno2(0x29, &Wire1);

// BMP581 Sensors
BMP581Sensor bmp0(0x46, &Wire);
BMP581Sensor bmp1(0x47, &Wire);
BMP581Sensor bmp2(0x46, &Wire1);  

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

    // Calibrate sensors if necessary
    bno0.calibrate();
    bno1.calibrate();
    bno2.calibrate();

    bmp0.calibrate();
    bmp1.calibrate();
    bmp2.calibrate();
}

void loop()
{
    // Read data from BNO055 sensors
    BNO055Data data0 = bno0.readData();
    BNO055Data data1 = bno1.readData();
    BNO055Data data2 = bno2.readData();

    // Collect data
    imu::Vector<3> orientation_sum(0, 0, 0);
    imu::Vector<3> gyro_sum(0, 0, 0);
    imu::Vector<3> accel_sum(0, 0, 0);

    int data_count = 0;

    // Sum data from each sensor
    orientation_sum = orientation_sum + data0.orientation;
    gyro_sum = gyro_sum + data0.gyroscope;
    accel_sum = accel_sum + data0.acceleration;
    data_count++;

    orientation_sum = orientation_sum + data1.orientation;
    gyro_sum = gyro_sum + data1.gyroscope;
    accel_sum = accel_sum +data1.acceleration;
    data_count++;

    orientation_sum = orientation_sum +data2.orientation;
    gyro_sum = gyro_sum + data2.gyroscope;
    accel_sum = accel_sum + data2.acceleration;
    data_count++;

    // Calculate averages
    imu::Vector<3> avg_orientation = orientation_sum / data_count;
    imu::Vector<3> avg_gyro = gyro_sum / data_count;
    imu::Vector<3> avg_accel = accel_sum / data_count;

    Serial.print("Orientation: ");
    Serial.print("X: "); Serial.print(avg_orientation.x());
    Serial.print(" Y: "); Serial.print(avg_orientation.y());
    Serial.print(" Z: "); Serial.println(avg_orientation.z());

    Serial.print("Acceleration: ");
    Serial.print("X: "); Serial.print(avg_accel.x());
    Serial.print(" Y: "); Serial.print(avg_accel.y());
    Serial.print(" Z: "); Serial.println(avg_accel.z());

    Serial.print("Gyroscope: ");
    Serial.print("X: "); Serial.print(avg_gyro.x());
    Serial.print(" Y: "); Serial.print(avg_gyro.y());
    Serial.print(" Z: "); Serial.println(avg_gyro.z());

    // Read data from BMP581 sensors
    BMPData bmpData0 = bmp0.readData();
    BMPData bmpData1 = bmp1.readData();
    BMPData bmpData2 = bmp2.readData();

    // Print pressure and temperature data
    Serial.print("BMP0 Pressure: "); Serial.print(bmpData0.pressure);
    Serial.print(" Pa, Temperature: "); Serial.print(bmpData0.temperature); Serial.println(" C");

    Serial.print("BMP1 Pressure: "); Serial.print(bmpData1.pressure);
    Serial.print(" Pa, Temperature: "); Serial.print(bmpData1.temperature); Serial.println(" C");

    Serial.print("BMP2 Pressure: "); Serial.print(bmpData2.pressure);
    Serial.print(" Pa, Temperature: "); Serial.print(bmpData2.temperature); Serial.println(" C");

    delay(1000); // Adjust the delay as necessary
}
