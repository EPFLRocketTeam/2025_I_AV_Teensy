#include <Wire.h>
#include <SparkFun_BMP581_Arduino_Library.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define LOOP_PERIOD 10 // 50hz

// bmp constructors init
BMP581 bmp;  // use I2C interface
BMP581 bmp1; // use I2C interface
BMP581 bmp2; // use I2C interface

// bno constructors
//Adafruit_BNO055 bno2 = Adafruit_BNO055(55, 0x28, &Wire);
Adafruit_BNO055 bno2 = Adafruit_BNO055(55, 0x29, &Wire);
//Adafruit_BNO055 bno2 = Adafruit_BNO055(55, 0x29, &Wire2);

uint32_t timer = millis();

void setup()
{
    // var
    bool status = 1;

    // begin serial and wait for opening
    Serial.begin(9600);

    Wire.begin();
    Wire2.begin();
    // BMP280 Init
    //status |= (bmp.beginI2C(0x46, Wire) == BMP5_OK);
    //status |= (bmp1.beginI2C(0x47, Wire) == BMP5_OK);
    //status |= (bmp2.beginI2C(0x46, Wire2) == BMP5_OK);

    //status |= bno.begin();
    //status |= bno1.begin();
    status |= bno2.begin();

    bno2.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P6);
    bno2.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P6);

    timer = millis();

    print_cols();
}

void print_cols()
{
    Serial.print("time, pitch, roll, heading\n");
}

void print_line(const std::vector<double> &elements)
{
    int n = elements.size();
    for (int i = 0; i < n; i++)
    {
        Serial.print(elements[i], 10);
        if (i != n - 1)
            Serial.print(",");
    }
    Serial.print("\n");
}


// Sensors Structures
bmp5_sensor_data data;
sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;

int i = 0;
int error_count = 0;
double sum = 0.;
int its = 0;

void loop()
{
    long loop_start = millis();

    bmp2.getSensorData(&data);
    bno2.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno2.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);

    //print_line({orientationData.orientation.y, orientationData.orientation.z});
    
    if(orientationData.orientation.z <= 0. && orientationData.orientation.y <= 0){
        error_count++;
    }
    if(i >= 100){
      its++;
      sum += error_count;
      Serial.println(error_count);
      Serial.println(sum/its);
      error_count = 0;
      i = 0;
    }



    long loop_duration = millis() - loop_start;
    long to_sleep = LOOP_PERIOD - loop_duration;
    if (to_sleep > 0) {
        delay(to_sleep);
    }
    i++;
}
