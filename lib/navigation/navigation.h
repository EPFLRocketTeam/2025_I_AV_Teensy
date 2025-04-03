#ifndef NAVIGATION_HPP
#define NAVIGATION_HPP

#include <iostream>
#include <vector>

struct RawData {
    int timestamp;
    double gyroX1, gyroY1, gyroZ1;
    double accelX1, accelY1, accelZ1;
    double gyroX2, gyroY2, gyroZ2;
    double accelX2, accelY2, accelZ2;
    double gyroX3, gyroY3, gyroZ3;
    double accelX3, accelY3, accelZ3;
    double pressure1, pressure2, pressure3;
    double gpsLat, gpsLon, gpsAlt;
};

class Navigation
{
public:
    Navigation() = default;
    Navigation(std::vector<double> data);
    Navigation(double time, double baro1, double baro2, double baro3, double temperature, double gpsLatitude, double gpsLongitude, double gpsAltitude, double gyroX1, double gyroY1, double gyroZ1, double gyroX2, double gyroY2, double gyroZ2, double gyroX3, double gyroY3, double gyroZ3);
    void init(double time, double baro1, double baro2, double baro3, double temperature, double gpsLatitude, double gpsLongitude, double gpsAltitude, double gyroX1, double gyroY1, double gyroZ1, double gyroX2, double gyroY2, double gyroZ2, double gyroX3, double gyroY3, double gyroZ3);
    ~Navigation();

    void update(double time, double baro1, double baro2, double baro3, double temperature, double gps_latitude, double gps_longitude, double gps_altitude, double gyroX1, double gyroY1, double gyroZ1, double gyroX2, double gyroY2, double gyroZ2, double gyroX3, double gyroY3, double gyroZ3);
    
    void update(std::vector<double> &data) {
        update(data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9], data[10], data[11], data[12], data[13], data[14], data[15], data[16]);
    }

    std::vector<double> get_state() const;

private:
    std::vector<double> gps_baro_to_meters(const std::vector<double>& gps, const std::vector<double>& baro, const double temperature);
    double pressure_to_altitude(const double& pressure, const double& temperature);

    double avg_baro(const std::vector<double>& baro);
    std::vector<double> avg_gyro(const std::vector<double>& gyro);

    double prev_baro;
    double sea_level_pressure = 101325.0;
    std::vector<double> initial_position;

    double prev_gps_time;
    double prev_time;


    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> acceleration;
    std::vector<double> orientation;
    std::vector<double> angular_velocity;
};


#endif // NAVIGATION_HPP