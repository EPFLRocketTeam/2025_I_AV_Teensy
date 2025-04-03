#ifndef NAVIGATION_HPP
#define NAVIGATION_HPP

#include <iostream>
#include <vector>

class Navigation
{
public:
    Navigation(double time, double baro1, double baro2, double baro3, double temperature, double gpsLatitude, double gpsLongitude, double gpsAltitude, double gyroX1, double gyroY1, double gyroZ1, double gyroX2, double gyroY2, double gyroZ2, double gyroX3, double gyroY3, double gyroZ3);
    ~Navigation();

    void update(double time, double baro1, double baro2, double baro3, double temperature, double gps_latitude, double gps_longitude, double gps_altitude, double gyroX1, double gyroY1, double gyroZ1, double gyroX2, double gyroY2, double gyroZ2, double gyroX3, double gyroY3, double gyroZ3);
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