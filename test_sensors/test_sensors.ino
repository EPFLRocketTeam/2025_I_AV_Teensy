#include <Arduino.h>
#include "Sensors.h"

// BNO055 Sensors
BNO055Sensor bno0(0x28, &Wire);
BNO055Sensor bno1(0x29, &Wire);
BNO055Sensor bno2(0x29, &Wire2);

// BMP581 Sensors
BMP581Sensor bmp0(0x46, &Wire); 
BMP581Sensor bmp1(0x47, &Wire);
BMP581Sensor bmp2(0x46, &Wire2);  

void setup()
{
    Serial.begin(9600);
    Serial.println("Initializing serial communication");
    // Initialize I2C buses
    Wire.begin();
    Wire.setSCL(16);
    Wire.setSDA(17);
    Wire.setClock(400000);

    Wire2.begin();
    Wire2.setSCL(24);
    Wire2.setSDA(25);
    Wire2.setClock(400000);

    // Setup sensors
    bool bno0_status = bno0.setup();
    delay(100);
    bool bno1_status = bno1.setup();
    delay(100);
    bool bno2_status = bno2.setup();
    delay(100);
    Serial.println(String("BNO Sensors' status: ") + bno0_status + " " + bno1_status + " " + bno2_status);
    Serial.println("Finished initializing BNO");

    bool bmp0_status = bmp0.setup();
    delay(100);
    bool bmp1_status = bmp1.setup();
    delay(100);
    bool bmp2_status = bmp2.setup();
    delay(100);
    Serial.println(String("bmp Sensors' status: ") + bmp0_status + " " + bmp1_status + " " + bmp2_status);
    Serial.println("Finished initializing bmp");

    // Calibrate sensors if necessary
    bno0.calibrate();
    bno1.calibrate();
    bno2.calibrate();
    Serial.println("Finished calibrating BNO");

    bmp0.calibrate();
    bmp1.calibrate();
    bmp2.calibrate();
    Serial.println("Finished calibrating BMP");

}

void loop()
{
    // Read data from BNO055 sensors
    Serial.println("Receiving data");
    BNO055Data data0 = bno0.readData();
    delay(50);
    BNO055Data data1 = bno1.readData();
    delay(50);
    BNO055Data data2 = bno2.readData();
    delay(50);

    // Collect data
    imu::Vector<3> orientation_sum(0, 0, 0);
    imu::Vector<3> gyro_sum(0, 0, 0);
    imu::Vector<3> accel_sum(0, 0, 0);

    imu::Vector<3> curr_orientation0(0, 0, 0);
    imu::Vector<3> curr_gyro0(0, 0, 0);
    imu::Vector<3> curr_accel0(0, 0, 0);

    imu::Vector<3> curr_orientation1(0, 0, 0);
    imu::Vector<3> curr_gyro1(0, 0, 0);
    imu::Vector<3> curr_accel1(0, 0, 0);

    imu::Vector<3> curr_orientation2(0, 0, 0);
    imu::Vector<3> curr_gyro2(0, 0, 0);
    imu::Vector<3> curr_accel2(0, 0, 0);

    curr_orientation0 = data0.orientation;    
    curr_gyro0 = data0.gyroscope;
    curr_accel0 = data0.acceleration;

    curr_orientation1 = data1.orientation;    
    curr_gyro1 = data1.gyroscope;
    curr_accel1 = data1.acceleration;
    
    curr_orientation2 = data2.orientation;    
    curr_gyro2 = data2.gyroscope;
    curr_accel2 = data2.acceleration;
    int data_count = 0;

    // Sum data from each sensor
    orientation_sum = orientation_sum + curr_orientation0;
    gyro_sum = gyro_sum + curr_gyro0;
    accel_sum = accel_sum + curr_accel0;
    data_count++;

    orientation_sum = orientation_sum + curr_orientation1;
    gyro_sum = gyro_sum + curr_gyro1;
    accel_sum = accel_sum + curr_accel1;
    data_count++;

    orientation_sum = orientation_sum +curr_orientation2;
    gyro_sum = gyro_sum + curr_gyro2;
    accel_sum = accel_sum + curr_accel2;
    data_count++;

    // Calculate averages
    imu::Vector<3> avg_orientation = orientation_sum / data_count;
    imu::Vector<3> avg_gyro = gyro_sum / data_count;
    imu::Vector<3> avg_accel = accel_sum / data_count;

    // // Print attitude and rate
    Serial.print("Orientation 0: ");
    Serial.print("X: "); Serial.print(curr_orientation0.x());
    Serial.print(" Y: "); Serial.print(curr_orientation0.y());
    Serial.print(" Z: "); Serial.println(curr_orientation0.z());

    Serial.print("Orientation 1: ");
    Serial.print("X: "); Serial.print(curr_orientation1.x());
    Serial.print(" Y: "); Serial.print(curr_orientation1.y());
    Serial.print(" Z: "); Serial.println(curr_orientation1.z());

    Serial.print("Orientation 2: ");
    Serial.print("X: "); Serial.print(curr_orientation2.x());
    Serial.print(" Y: "); Serial.print(curr_orientation2.y());
    Serial.print(" Z: "); Serial.println(curr_orientation2.z());

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
