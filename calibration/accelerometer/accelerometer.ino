/********************************************************
 * PID Basic Example
 * Reading analog input 0 to control analog PWM output 3
 ********************************************************/

#include <Wire.h>
#include <SparkFun_BMP581_Arduino_Library.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define LOOP_PERIOD 10 // 50hz

#include "inc/Vec3.h"

// NAV PART

// bmp constructors init
BMP581 bmp;  // use I2C interface
BMP581 bmp1; // use I2C interface
BMP581 bmp2; // use I2C interface

// Sensors Structures
bmp5_sensor_data data;
sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;

// bno constructors
std::vector<Adafruit_BNO055> bnos = {Adafruit_BNO055(55, 0x28, &Wire), Adafruit_BNO055(55, 0x29, &Wire), Adafruit_BNO055(55, 0x29, &Wire2)};
const int IMU_N = bnos.size();

uint32_t timer = millis();

void setup()
{
    Wire.begin();
    Wire2.begin();

    Serial.begin(9600);

    // BMP280 Init
    // status |= (bmp.beginI2C(0x46, Wire) == BMP5_OK);
    // status |= (bmp1.beginI2C(0x47, Wire) == BMP5_OK);
    // status |= (bmp2.beginI2C(0x46, Wire2) == BMP5_OK);

    for (auto &bno : bnos)
    {
        bno.begin();
        bno.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P6);
        bno.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P6);
    }

    timer = millis();

    print_cols();
}

void print_cal(std::vector<Vec3> cal){
  Serial.print("{");
  for(int i = 0; i < 3; i++){
    print_vec(cal[i]);
    if(i != 2){
      Serial.print(", ");
    }
  }
  Serial.print("}\n");
}

void print_vec(Vec3 v)
{
    print_line({v.x, v.y, v.z});
}

void print_cols()
{
    Serial.print("time, pitch, roll, heading\n");
}

void print_line(const std::vector<double> &elements)
{
    Serial.print("{");
    int n = elements.size();
    for (int i = 0; i < n; i++)
    {
        Serial.print(elements[i], 10);
        if (i != n - 1)
            Serial.print(",");
    }
    Serial.print("}");
}

int sample_count = 0;

// xyz att * 3, xyz rates * 3
std::vector<Vec3> imu_sums(6, {0, 0, 0});
std::vector<int> counts(6, 0);

void loop()
{
    long loop_start = millis();

    if (sample_count > 1000)
    {
        std::vector<Vec3> att_cal;
        std::vector<Vec3> rate_cal;
        for (int i = 0; i < 3; i++)
        {
            Vec3 atts = imu_sums[i] / counts[i];
            Vec3 rates = imu_sums[i + 3] / counts[i + 3];
            att_cal.push_back(atts);
            rate_cal.push_back(rates);
        }

        print_cal(att_cal);
        print_cal(rate_cal);

        sample_count = 0;
        imu_sums.assign(6, {0, 0, 0});
        counts.assign(6, 0);
    }

    for (int i = 0; i < 3; i++)
    {
        Adafruit_BNO055 bno = bnos[i];

        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);

        const sensors_vec_t& att = orientationData.orientation;
        const sensors_vec_t& gyro = angVelocityData.gyro;

        if (att.z != 0)
        {
            imu_sums[i] += {att.x, att.y, att.z};
            counts[i]++;
        }

        if(gyro.z != 0){
            imu_sums[i+3] += {gyro.x, gyro.y, gyro.z};
            counts[i+3]++; 
        }
    }

    sample_count++;
    long loop_duration = millis() - loop_start;
    long to_sleep = LOOP_PERIOD - loop_duration;
    if (to_sleep > 0)
    {
        delay(to_sleep);
    }
}
