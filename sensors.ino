#include <SparkFun_BMP581_Arduino_Library.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Wire.h>

std::vector<Vec3> att_calibration = {{327.9375000000,1.2500000000,85.4375000000}, {328.5625000000,2.3750000000,85.3125000000}, {329.3125000000,-1.7500000000,85.0000000000}};
std::vector<Vec3> rate_calibration = {{-0.0000637716,0.0000385986,-0.0000033564}, {-0.0000629007,0.0001092486,-0.0000446926}, {-0.0001784996,0.0001841662,0.0002323328}};

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

std::vector<Vec3> currAtts(3, Vec3::Zero());
std::vector<Vec3> currRates(3, Vec3::Zero());

void setup_sensors()
{
    Wire.begin();
    Wire2.begin();

    // BMP280 Init
    // status |= (bmp.beginI2C(0x46, Wire) == BMP5_OK);
    // status |= (bmp1.beginI2C(0x47, Wire) == BMP5_OK);
    // status |= (bmp2.beginI2C(0x46, Wire2) == BMP5_OK);

    for(auto& bno : bnos){
        bno.begin();
        bno.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P6);
        bno.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P6);
    }
}

DroneState last_state;
DroneState read_sensors()
{
    DroneState res;

    for (int i = 0; i < 3; i++)
    {
        Adafruit_BNO055& bno = bnos[i];
        bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
        bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);

        const sensors_vec_t& o = orientationData.orientation;
        Vec3 orientation {o.x, o.y, o.z}; 
        if (orientation.x != 0 || orientation.y != 0 || orientation.z != 0)
        {   
            orientation = orientation - att_calibration[i]; 
            currAtts[i] = {orientation.y, -orientation.z, orientation.x};
            res.attitude_count++;
        }

        res.attitude += currAtts[i];

        const sensors_vec_t& g = angVelocityData.gyro;
        Vec3 gyro {g.x, g.y, g.z};
        if (gyro.x != 0 || gyro.y != 0 || gyro.z != 0)
        {
            gyro = gyro - rate_calibration[i];
            gyro = gyro.toDegree();
            currRates[i] = {-gyro.z, gyro.x, gyro.y}; // idk the why the rotation is not the same as above, be carefull when changing
            res.rate_count++;
        }

        res.rate += currRates[i];
    }

    res.attitude = res.attitude / IMU_N;
    res.rate = res.rate / IMU_N;

    last_state = res;
    return res;
}