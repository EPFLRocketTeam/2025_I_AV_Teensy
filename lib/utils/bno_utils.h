#ifndef BNO_UTILS_H
#define BNO_UTILS_H

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

void displaySensorDetails(const Adafruit_BNO055&);
void displaySensorStatus(const Adafruit_BNO055&);
void displayCalStatus(const Adafruit_BNO055&);
void displaySensorOffsets(const adafruit_bno055_offsets_t&);


#endif // BNO_UTILS_H