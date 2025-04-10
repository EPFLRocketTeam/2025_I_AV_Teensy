#include "navigation.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>
#include <vector>
#include <cmath>

using namespace std;

Navigation::Navigation(double time, double baro1, double baro2, double baro3, double temperature, double gpsLatitude, double gpsLongitude, double gpsAltitude, double gyroX1, double gyroY1, double gyroZ1, double gyroX2, double gyroY2, double gyroZ2, double gyroX3, double gyroY3, double gyroZ3)
{
    init(time, baro1, baro2, baro3, temperature, gpsLatitude, gpsLongitude, gpsAltitude, gyroX1, gyroY1, gyroZ1, gyroX2, gyroY2, gyroZ2, gyroX3, gyroY3, gyroZ3);
}

Navigation::Navigation(std::vector<double> data) {
    if (data.size() != 17) std::cerr << "Invalid data size. Expected 17 values." << std::endl; return;
    init(data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15], data[16]);
}

void Navigation::init(double time, double baro1, double baro2, double baro3, double temperature, double gpsLatitude, double gpsLongitude, double gpsAltitude, double gyroX1, double gyroY1, double gyroZ1, double gyroX2, double gyroY2, double gyroZ2, double gyroX3, double gyroY3, double gyroZ3) {
    prev_time = time;
    prev_gps_time = time;
    prev_baro = avg_baro({baro1, baro2, baro3});
    initial_position = gps_baro_to_meters({gpsLatitude, gpsLongitude, gpsAltitude}, {baro1, baro2, baro3}, temperature);
    prev_temperature = temperature;
    position = {0.0, 0.0, 0.0};
    velocity = {0.0, 0.0, 0.0};
    acceleration = {0.0, 0.0, 0.0};
    orientation = {0.0, 0.0, 0.0};
    angular_velocity = {0.0, 0.0, 0.0};

    cout << "Position initiale : " << position[0] << " " << position[1] << " " << position[2] << endl;
}

Navigation::~Navigation()
{
}

void Navigation::update(double time, double baro1, double baro2, double baro3, double temperature, double gps_latitude, double gps_longitude, double gps_altitude, double gyroX1, double gyroY1, double gyroZ1, double gyroX2, double gyroY2, double gyroZ2, double gyroX3, double gyroY3, double gyroZ3)
{
    vector<double> baro = {baro1, baro2, baro3};
    vector<double> gps = {gps_latitude, gps_longitude, gps_altitude};
    temperature = filter_0_temperature(temperature);

    double dt = (time - prev_time) / 1000.0;
    prev_time = time;

    vector<double> new_position(3, 0.0);
    vector<double> new_velocity(3, 0.0);
    vector<double> new_acceleration(3, 0.0);

    if (false)// ((time - prev_gps_time)/1000 >= GPS_REFRESH_PERIOD)
    {
        new_position = gps_baro_to_meters(gps, baro, temperature);
        new_position[0] = new_position[0] - initial_position[0];
        new_position[1] = new_position[1] - initial_position[1];
        new_position[2] = pressure_to_altitude(avg_baro(baro), temperature) - initial_position[2];

        new_velocity[0] = (new_position[0] - position[0]) / ((time - prev_gps_time)/1000);
        new_velocity[1] = (new_position[1] - position[1]) / ((time - prev_gps_time)/1000);
        new_velocity[2] = (new_position[2] - position[2]) / dt;

        new_acceleration[0] = (new_velocity[0] - velocity[0]) / ((time - prev_gps_time)/1000);
        new_acceleration[1] = (new_velocity[1] - velocity[1]) / ((time - prev_gps_time)/1000);
        new_acceleration[2] = (new_velocity[2] - velocity[2]) / dt;

        prev_gps_time = time;
    }
    else
    {  
        new_position[0] = position[0];
        new_position[1] = position[1];
        new_position[2] = pressure_to_altitude(avg_baro(baro), temperature) - initial_position[2];

        new_velocity[0] = velocity[0];
        new_velocity[1] = velocity[1];
        new_acceleration[0] = acceleration[0];
        new_acceleration[1] = acceleration[1];

        new_velocity[2] = (new_position[2] - position[2]) / dt;
        new_acceleration[2] = (new_velocity[2] - velocity[2]) / dt;
    }

    position = new_position;
    velocity = new_velocity;
    acceleration = new_acceleration;


    angular_velocity = avg_gyro({gyroX1, gyroY1, gyroZ1, gyroX2, gyroY2, gyroZ2, gyroX3, gyroY3, gyroZ3});
    orientation[0] += angular_velocity[0]*dt;
    orientation[1] += angular_velocity[1]*dt;
    orientation[2] += angular_velocity[2]*dt;
}


std::vector<double> Navigation::get_state() const
{
    return {position[0], position[1], position[2], velocity[0], velocity[1], velocity[2], acceleration[0], acceleration[1], acceleration[2], orientation[0], orientation[1], orientation[2], angular_velocity[0], angular_velocity[1], angular_velocity[2]};
}

std::vector<double> Navigation::avg_gyro(const std::vector<double>& gyro)
{
    double sumX = 0;
    double sumY = 0;
    double sumZ = 0;

    for (size_t i = 0; i < gyro.size(); i += 3) {
        sumX += gyro[i];     // Sum of all x values.
        sumY += gyro[i + 1]; // Sum of all y values.
        sumZ += gyro[i + 2]; // Sum of all z values.
    }

    return {sumX / 3.0, sumY / 3.0, sumZ / 3.0};
}

std::vector<double> Navigation::gps_baro_to_meters(const std::vector<double>& gps, const std::vector<double>& baro, const double temperature)
{
    double latitude_meters = gps[0] * 111320/10000000.0; 
    double longitude_meters = (gps[1] * 40075000 * cos((M_PI*gps[0]/180)/10000000.0) / 360) /10000000.0;

    double altitude = pressure_to_altitude(avg_baro(baro), temperature);

    return {latitude_meters, longitude_meters, altitude};
}

double Navigation::pressure_to_altitude(const double& pressure, const double& temperature)
{
    prev_baro = pressure;
    //prev_baro = (pressure*0.7 + prev_baro*0.3);  // Low-pass filter

    return (temperature / L) * (1 - pow(prev_baro / sea_level_pressure, (R * L) / (g * M)));
}


double Navigation::avg_baro(const std::vector<double>& baro)
{
    double sum = 0;
    int count = 0;

    for (size_t i = 0; i < baro.size(); i++)
    {
        if (baro[i] > 1000.0)
        {
            sum += baro[i];
            ++count;
        }
    }

    if (count == 0)
    {
        return prev_baro;
    }
    else
    {
        return sum / count;
    }
}

double Navigation::filter_0_temperature(const double& temperature)
{
    if (temperature > 273.15 + 1)
    {
        prev_temperature = temperature;
        return temperature;
    }
    else
    {
        return prev_temperature;
    }
}