#include <SparkFun_BMP581_Arduino_Library.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <Wire.h>

#define MIN_PRESSURE 90000
#define MAX_PRESSURE 110000

#define MIN_ALTITUDE -1000
#define MAX_ALTITUDE 1000

const double derivativeFilter = 0.5;

std::vector<Vec3> att_calibration = {{327.9375000000,1.2500000000,85.4375000000}, {328.5625000000,2.3750000000,85.3125000000}, {329.3125000000,-1.7500000000,85.0000000000}};
std::vector<Vec3> rate_calibration = {{-0.0000637716,0.0000385986,-0.0000033564}, {-0.0000629007,0.0001092486,-0.0000446926}, {-0.0001784996,0.0001841662,0.0002323328}};
const double ground_altitude = 316; // TODO callibrate before flight automatically


// bmp constructors init
const int BMP_N = 3;
std::vector<BMP581> bmps(BMP_N);

// Sensors Structures
bmp5_sensor_data data;
sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;

/*!
    @brief Calculates the altitude (in meters).

    @param  pressure      Current atmospheric pressure in Pa
    @return Altitude in meters
*/
double get_altitude(double pressure) {
  // Equation taken from BMP180 datasheet (page 16):
  //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

  // Note that using the equation from wikipedia can give bad results
  // at high altitude. See this thread for more information:
  //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064

  pressure = pressure/100; // convert to hPa
  float seaLevel = 1013.25;
  return 44330.0 * (1.0 - pow(pressure / seaLevel, 0.1903));
}

// bno constructors
std::vector<Adafruit_BNO055> bnos = {Adafruit_BNO055(55, 0x28, &Wire), Adafruit_BNO055(55, 0x29, &Wire), Adafruit_BNO055(55, 0x29, &Wire2)};
const int IMU_N = bnos.size();

std::vector<Vec3> currAtts(3, Vec3::Zero());
std::vector<Vec3> currRates(3, Vec3::Zero());
std::vector<double> currPressures(3, 0);
std::vector<double> currAltitudes(3, 0);
void setup_sensors()
{
    Serial.print("Setting up sensors..");

    Wire.begin();
    Wire2.begin();

    // BMP280 Init
    bmps[0].beginI2C(0x46, Wire);
    bmps[1].beginI2C(0x47, Wire);
    bmps[2].beginI2C(0x46, Wire2);

    for(auto& bno : bnos){
        bno.begin();
        bno.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P6);
        bno.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P6);
    }

    Serial.println("Finished sensor setup");
}

State last_state;
SensorData read_sensors()
{
    SensorData res;
    State& state = res.state;

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
            orientation = orientation.toRad();
            currAtts[i] = {orientation.y, -orientation.z, orientation.x};
            res.attitude_count++;
        }

        state.att += currAtts[i];

        const sensors_vec_t& g = angVelocityData.gyro;
        Vec3 gyro {g.x, g.y, g.z};
        if (gyro.x != 0 || gyro.y != 0 || gyro.z != 0)
        {
            gyro = gyro - rate_calibration[i];
            currRates[i] = {-gyro.z, gyro.x, gyro.y}; // idk the why the rotation is not the same as above, be carefull when changing
            res.rate_count++;
        }

        state.rate += currRates[i];
    }

    state.att = state.att / IMU_N;
    state.rate = state.rate / IMU_N;

    // Barometer
    double avg_altitude = 0;
    for (int i = 0; i < BMP_N; i++)
    {
        bmp5_sensor_data data;
        bmps[i].getSensorData(&data);
        double pressure = data.pressure;
        
        if(MIN_PRESSURE < pressure && pressure < MAX_PRESSURE){
            currPressures[i] = pressure;
            res.pressure_count++;
        }

        double altitude = get_altitude(currPressures[i]);
        if(MIN_ALTITUDE < altitude && altitude < MAX_ALTITUDE){
            currAltitudes[i] = altitude - ground_altitude;
        }

        avg_altitude += currAltitudes[i];
    }

    avg_altitude = avg_altitude / BMP_N;
    state.pos.z = avg_altitude;

    double curr_vel = (state.pos.z-last_state.pos.z)*1000 / LOOP_PERIOD;
    state.vel.z = curr_vel * derivativeFilter + last_state.vel.z * (1-derivativeFilter);

    last_state = state;
    return res;
}