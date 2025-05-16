#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SparkFun_BMP581_Arduino_Library.h>
#include <utils.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <ctime>
#include <Arduino.h>
#include <SD.h>
#include <SparkFun_u-blox_GNSS_v3.h>

#include "god.h"
using namespace std;

GOD* god;


//Adafruit_BNO055 bno1(-1, 0x28, &Wire), bno2(-1, 0x29, &Wire), bno3(-1, 0x29, &Wire2);
Adafruit_BNO055 bno1(-1, 0x28, &Wire), bno2(-1, 0x28, &Wire), bno3(-1, 0x28, &Wire);
BMP581 bmp1, bmp2, bmp3;
SFE_UBLOX_GNSS myGNSS; // GPS object

int i(0); 

const int chipSelect = BUILTIN_SDCARD; // Teensy has a built-in SD card reader
File NavLog;


vector<double> read_data(const bool print = false) {
    double time = millis();

    // read bmp data
    bmp5_sensor_data data1 = {0,0}, data2 = {0,0}, data3 = {0,0};
    bmp1.getSensorData(&data1); bmp2.getSensorData(&data2); bmp3.getSensorData(&data3);
    double baro1 = data1.pressure, baro2 = data2.pressure, baro3 = data3.pressure;
    double temperature1 = data1.temperature;
    double temperature2 = data2.temperature;
    double temperature3 = data3.temperature;


    // read bno data
    sensors_event_t  angVelocityData1, angVelocityData2, angVelocityData3;
    bno1.getEvent(&angVelocityData1, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno2.getEvent(&angVelocityData2, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno3.getEvent(&angVelocityData3, Adafruit_BNO055::VECTOR_GYROSCOPE);
    double gyroX1 = angVelocityData1.gyro.x, gyroY1 = angVelocityData1.gyro.y, gyroZ1 = angVelocityData1.gyro.z;
    double gyroX2 = angVelocityData2.gyro.x, gyroY2 = angVelocityData2.gyro.y, gyroZ2 = angVelocityData2.gyro.z;
    double gyroX3 = angVelocityData3.gyro.x, gyroY3 = angVelocityData3.gyro.y, gyroZ3 = angVelocityData3.gyro.z;

    sensors_event_t accelerometerData1, accelerometerData2, accelerometerData3  ;
    bno1.getEvent(&accelerometerData1, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno2.getEvent(&accelerometerData2, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno3.getEvent(&accelerometerData3, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    double accX1 = accelerometerData1.acceleration.x, accY1 = accelerometerData1.acceleration.y, accZ1 = accelerometerData1.acceleration.z;
    double accX2 = accelerometerData2.acceleration.x, accY2 = accelerometerData2.acceleration.y, accZ2 = accelerometerData2.acceleration.z;
    double accX3 = accelerometerData3.acceleration.x, accY3 = accelerometerData3.acceleration.y, accZ3 = accelerometerData3.acceleration.z;


    double gps_latitude = 0.0; // to change
    double gps_longitude = 0.0;
    double gps_altitude = 0.0; // in meters
    double gps_speed = 0.0; // unit is mm/s from library, but original logic didn't divide by 1000 here
    double gps_velN = 0.0; // mm/s
    double gps_velE = 0.0; // mm/s
    double gps_velD = 0.0; // mm/s
    uint8_t fixType = 0;
    uint8_t carrSoln = 0;

    gps_latitude = (double)myGNSS.getLatitude(); // degrees * 10^-7
    gps_longitude = (double)myGNSS.getLongitude(); // degrees * 10^-7
    gps_altitude = (double)myGNSS.getAltitudeMSL() / 1000.0; // m
    gps_speed = (double)myGNSS.getGroundSpeed() / 1000.0; // m/s
    gps_velN = (double)myGNSS.getNedNorthVel() / 1000.0; // m/s
    gps_velE = (double)myGNSS.getNedEastVel() / 1000.0; // m/s
    gps_velD = (double)myGNSS.getNedDownVel() / 1000.0; // m/s

    fixType = myGNSS.getFixType(); // update fixType
    carrSoln = myGNSS.getCarrierSolutionType(); // update carrSoln

    // if (myGNSS.getPVT() == true)
    // {
       
    //     gps_latitude = (double)myGNSS.getLatitude(); // degrees * 10^-7
    //     gps_longitude = (double)myGNSS.getLongitude(); // degrees * 10^-7
    //     gps_altitude = (double)myGNSS.getAltitudeMSL() / 1000.0; // mm
    //     gps_speed = (double)myGNSS.getGroundSpeed(); // mm/s
    //     gps_velN = (double)myGNSS.getNedNorthVel(); // mm/s
    //     gps_velE = (double)myGNSS.getNedEastVel(); // mm/s
    //     gps_velD = (double)myGNSS.getNedDownVel(); // mm/s

    //     fixType = myGNSS.getFixType(); // update fixType
    //     carrSoln = myGNSS.getCarrierSolutionType(); // update carrSoln

    // }

    if (print) {
        Serial.print("Time: "); Serial.print(time); Serial.print(" ms, ");
        Serial.print("Baro1: "); Serial.print(baro1); Serial.print(" Pa, ");
        Serial.print("Baro2: "); Serial.print(baro2); Serial.print(" Pa, ");
        Serial.print("Baro3: "); Serial.print(baro3); Serial.print(" Pa, ");
        Serial.print("Temperature: "); Serial.print(temperature1); Serial.print(" C, ");
        Serial.print("Temperature: "); Serial.print(temperature2); Serial.print(" C, ");
        Serial.print("Temperature: "); Serial.print(temperature3); Serial.print(" C, ");
        Serial.print("GPS Latitude: "); Serial.print(gps_latitude); Serial.print(", ");
        Serial.print("GPS Longitude: "); Serial.print(gps_longitude); Serial.print(", ");
        Serial.print("GPS Altitude: "); Serial.print(gps_altitude); Serial.print(", "); 
        Serial.print("GyroX1: "); Serial.print(gyroX1); Serial.print(", ");
        Serial.print("GyroY1: "); Serial.print(gyroY1); Serial.print(", ");
        Serial.print("GyroZ1: "); Serial.println(gyroZ1); 
        Serial.print(F("Fix Type: ")); Serial.print(fixType); Serial.println();
        Serial.print("carrierSolution: "); Serial.println(carrSoln); 
    }

    std::vector<double> data; 
    data.push_back(time);               // 0
    data.push_back(baro1);              // 1
    data.push_back(baro2);              // 2
    data.push_back(baro3);              // 3
    data.push_back(temperature1 + 273.15); // 4
    data.push_back(temperature2 + 273.15); // 5
    data.push_back(temperature3 + 273.15); // 6
    data.push_back(gps_latitude);       // 
    data.push_back(gps_longitude);      //  
    data.push_back(gps_altitude);       //  
    data.push_back(gps_velN);           //  
    data.push_back(gps_velE);           // 
    data.push_back(gps_velD);           // 10
    data.push_back(accX1);              // 11
    data.push_back(accY1);              // 12
    data.push_back(accZ1);              // 13
    data.push_back(gyroX1);             // 14
    data.push_back(gyroY1);             // 15
    data.push_back(gyroZ1);             // 16
    data.push_back(accX2);              // 17
    data.push_back(accY2);              // 18
    data.push_back(accZ2);              // 19
    data.push_back(gyroX2);             // 20
    data.push_back(gyroY2);             // 21
    data.push_back(gyroZ2);             // 22
    data.push_back(accX3);              // 23
    data.push_back(accY3);              // 24
    data.push_back(accZ3);              // 25
    data.push_back(gyroX3);             // 26
    data.push_back(gyroY3);             // 27
    data.push_back(gyroZ3);             // 28
    data.push_back((double)fixType);    // 29 - Cast uint8_t to double for vector
    data.push_back((double)carrSoln);   // 30 - Cast uint8_t to double for vector

    return data; // Return the vector of raw data

}


void setup(void)
{
    Serial.println("test1");
    Serial.begin(115200); // 110, 300, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200
    Serial.println("serial complete");
    Wire.begin();
    Wire2.begin();


    while(!SD.begin(chipSelect)) {
        Serial.println("SD card initialization failed!");
    }
    Serial.println("SD card initialized.");
    // Supprimer le fichier existant pour le recréer

    //if (SD.exists("raw_data.csv")) { // Use raw_data.csv filename
    //    SD.remove("raw_data.csv"); // Use raw_data.csv filename
    //}

    // Create or open the raw data log file
    time_t timestamp;
    
    NavLog = SD.open(("raw_data" + String(timestamp) + ".csv").c_str(), FILE_WRITE); // Use raw_data.csv filename
    if (!NavLog) {
        Serial.println("Failed to create raw_data log file!"); // Use raw_data.csv filename
         // Stay in a loop if SD card is critical for data logging
        while (true); // Added infinite loop on failure
    }

    
    NavLog.println("Time,Baro1,Baro2,Baro3,Temp1,GPS_Lat,GPS_Lon,GPS_Alt,GPS_VelN,GPS_VelE,GPS_VelD,Accel1X,Accel1Y,Accel1Z,Gyro1X,Gyro1Y,Gyro1Z,Accel2X,Accel2Y,Accel2Z,Gyro2X,Gyro2Y,Gyro2Z,Accel3X,Accel3Y,Accel3Z,Gyro3X,GyroY3,GyroZ3,GPS_FixType,GPS_CarrSoln,LoopTime");
    NavLog.flush();
    Serial.println("SD card header written.");


    while (myGNSS.begin() == false) //on attend la connection en I2C (myGNSS uses Wire default 0x42)
    {
        Serial.println(F("u-blox GNSS not detected at default I2C address. Retrying..."));
        delay(100);
    }
    Serial.println("GNSS module connected");
    myGNSS.setI2COutput(COM_TYPE_UBX);// on veut recevoir seulement les données de position on veut pas de RTCM ou de NMEA
    myGNSS.setNavigationFrequency(20, VAL_LAYER_RAM);// Réglage de la fréquence à 20 Hz 
   
    myGNSS.setAutoPVT(true); // Active l’envoi automatique des messages NAV-PVT
    myGNSS.setI2CpollingWait(10);


    Serial.print("Initializing BNO055 sensor 1 (Wire, 0x28)...");
    while (!bno1.begin()) {
        Serial.println("Ooops, no BNO055 1 detected ... Check your wiring or I2C ADDR!");
        delay(100);
    }
    Serial.println("bno 1 complete");

    
    Serial.print("Initializing BNO055 sensor 2 (Wire, 0x28)...");
    while (!bno2.begin()) {
         Serial.println("Ooops, no BNO055 2 detected ... Check wiring/address!");
         delay(100);
    }
    Serial.println("bno 2 complete");

    Serial.print("Initializing BNO055 sensor 3 (Wire, 0x28)...");
     while (!bno3.begin()) { 
        Serial.println("Ooops, no BNO055 3 detected ... Check wiring/address!");
        delay(100);
    }
    Serial.println("bno 3 complete");


    Serial.print("Initializing BMP581 sensor 1 (Wire, 0x47)...");
    while(bmp1.beginI2C(0x47, Wire) != BMP5_OK) {
        Serial.println("Error: BMP581 1 not connected, check wiring and I2C address!");
        delay(100);
    }
    Serial.println("bmp 1 complete");

    Serial.print("Initializing BMP581 sensor 2 (Wire, 0x47)...");
    while(bmp2.beginI2C(0x47, Wire) != BMP5_OK) { 
        Serial.println("Error: BMP581 2 not connected, check wiring and I2C address!");
        delay(100);
    }
    Serial.println("bmp 2 complete");

    Serial.print("Initializing BMP581 sensor 3 (Wire2, 0x46)...");
    while(bmp3.beginI2C(0x46, Wire2) != BMP5_OK) { 
        Serial.println("Error: BMP581 3 not connected, check wiring and I2C address!");
        delay(100);
    }
    Serial.println("bmp 3 complete");

}

void loop(void)
{
    double startTime = millis();

    i++; // Increment total loop counter

    //lit les nouvelles données
    std::vector<double> data = read_data(true); // Read data from sensors



    
    if (i > 100) { // Skip the first 100 readings
        
        for (size_t j = 0; j < data.size(); ++j) { // Loop through the vector of raw data
            
            if (j == 31 || j == 32) { // GPS_FixType (29) and GPS_CarrSoln (30)
                 NavLog.print((int)data[j]); // Print as integer (no decimal places
            } else {
                 NavLog.print(data[j], 6); // Print with 6 decimal
            }

            if (j < data.size() - 1) {
                NavLog.print(",");
            }
        }

        // Calculate and log loop time - Logged as the LAST column
        double loopTime = millis() - startTime; // temps depuis le début de la boucle
        NavLog.print(","); // Add comma before loop time
        NavLog.print(loopTime, 3);
        NavLog.println();

        // Flush data to SD card periodically, only after logging has started

        if((i - 100) % 10 == 0) { // Flush every 10 logged entries
            NavLog.flush();
        }
    }


    // double executionTime = millis() - startTime;
    // if(executionTime < 10) {
    //     delay(10 - executionTime); // on attend le temps qu'il reste pour faire 10ms
    // }
}