#ifndef IMU_UTILS_H
#define IMU_UTILS_H

#include <utility/imumaths.h>
#include <Arduino.h>

void print(const imu::Vector<3>&, const char name[] = "");
void print(const imu::Quaternion&, const char name[] = "");
void print(const int8_t&, char [] = "");

void println(const imu::Vector<3>&, const char name[] = "");
void println(const imu::Quaternion&, const char name[] = "");
void println(const int8_t&, const char [] = "");

#endif // IMU_UTILS_H