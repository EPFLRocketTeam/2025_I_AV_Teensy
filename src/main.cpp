#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SparkFun_BMP581_Arduino_Library.h>
#include <utils.h>
#include <iostream>
#include <vector>
#include <math.h>

#include <Arduino.h>
#include <SD.h>
#include <SparkFun_u-blox_GNSS_v3.h>

#include "god.h"
#include "../lib/navigation_v2/navigation_v2.h"

using namespace std;

GOD* god;


//Adafruit_BNO055 bno1(-1, 0x28, &Wire), bno2(-1, 0x29, &Wire), bno3(-1, 0x29, &Wire2);
Adafruit_BNO055 bno1(-1, 0x28, &Wire), bno2(-1, 0x28, &Wire), bno3(-1, 0x28, &Wire);
BMP581 bmp1, bmp2, bmp3;
SFE_UBLOX_GNSS myGNSS; // GPS object
Navigation nav;

int i(0);

//SD card logging
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
    sensors_event_t  angVelocityData1, angVelocityData2, angVelocityData3, orientationData;
    bno1.getEvent(&angVelocityData1, Adafruit_BNO055::VECTOR_GYROSCOPE); 
    bno2.getEvent(&angVelocityData2, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno3.getEvent(&angVelocityData3, Adafruit_BNO055::VECTOR_GYROSCOPE);
    double gyroX1 = angVelocityData1.gyro.x, gyroY1 = angVelocityData1.gyro.y, gyroZ1 = angVelocityData1.gyro.z;
    double gyroX2 = angVelocityData2.gyro.x, gyroY2 = angVelocityData2.gyro.y, gyroZ2 = angVelocityData2.gyro.z;
    double gyroX3 = angVelocityData3.gyro.x, gyroY3 = angVelocityData3.gyro.y, gyroZ3 = angVelocityData3.gyro.z;

    // bno1.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    // double roll = orientationData.orientation.x; // Roll angle in degrees
    // double pitch = orientationData.orientation.y; // Pitch angle in degrees
    // Serial.print("roll: "); Serial.print(roll); Serial.print(", ");
    // Serial.print("pitch: "); Serial.print(pitch); Serial.print(" ---- ");

    // read gps data
    double gps_latitude = 0.0, gps_longitude = 0.0, gps_altitude = 0.0, gps_speed = 0.0, gps_velN = 0.0, gps_velE = 0.0, gps_velD = 0.0; // to change
    gps_latitude = myGNSS.getLatitude();
    gps_longitude = myGNSS.getLongitude();
    gps_altitude = myGNSS.getAltitudeMSL() / 1000.0; // in meters
    gps_speed = myGNSS.getGroundSpeed();
    gps_velN = myGNSS.getNedNorthVel();
    gps_velE = myGNSS.getNedEastVel();
    gps_velD = myGNSS.getNedDownVel();
    uint8_t fixType = myGNSS.getFixType();
    uint8_t carrSoln = myGNSS.getCarrierSolutionType();
    uint8_t rtk_quality = myGNSS.getCarrierSolutionType();// Returns RTK solution: 0=no, 1=float solution, 2=fixed solution
    // uint8_t numSV = myGNSS.getSIV();//nbr de sat utilisé

    // Serial.print(F("carrSoln: "));
    // Serial.println(carrSoln);

    if (print) {
        Serial.print(F("Fix Type: "));
        Serial.println(fixType);
    }
    // if (myGNSS.getPVT() == true)
    // {
    //     gps_latitude = myGNSS.getLatitude();
    //     gps_longitude = myGNSS.getLongitude();
    //     gps_altitude = myGNSS.getAltitudeMSL();
    //     gps_speed = myGNSS.getGroundSpeed();
    //     gps_velN = myGNSS.getNedNorthVel();
    //     gps_velE = myGNSS.getNedEastVel();
    //     gps_velD = myGNSS.getNedDownVel();

    //     uint8_t fixType = myGNSS.getFixType();
    //     uint8_t numSV = myGNSS.getSIV();//nbr de sat utilisé
    //     if (print) {
    //         Serial.print(F("Fix Type: "));
    //         Serial.println(fixType); // on veut 4(RTK flottant) ou 5(RTK fixe)
    //     }
    // }        

    // print data
    if (print) {
        Serial.print("Time: "); Serial.print(time); Serial.print(" ms, ");
        Serial.print("Baro1: "); Serial.print(baro1); Serial.print(" Pa, ");
        Serial.print("Baro2: "); Serial.print(baro2); Serial.print(" Pa, ");
        Serial.print("Baro3: "); Serial.print(baro3); Serial.print(" Pa, ");
        Serial.print("Temperature: "); Serial.print(temperature1); Serial.print(" C, ");
        Serial.print("GPS Latitude: "); Serial.print(gps_latitude); Serial.print(", ");
        Serial.print("GPS Longitude: "); Serial.print(gps_longitude); Serial.print(", ");
        Serial.print("GPS Altitude: "); Serial.print(gps_altitude/1000.0); Serial.print(", ");
        Serial.print("GyroX1: "); Serial.print(gyroX1); Serial.print(", ");
        Serial.print("GyroY1: "); Serial.print(gyroY1); Serial.print(", ");
        Serial.print("GyroZ1: "); Serial.println(gyroZ1);

        // Serial.print(F("Lat: "));
        // Serial.print(gps_latitude);
        // Serial.print(F(" Long: "));
        // Serial.print(gps_longitude);
        // Serial.print(F(" (degrees * 10^-7)"));
        // Serial.print(F(" Alt: "));
        // Serial.print(gps_altitude);
        // Serial.print(F(" (mm)"));
        // Serial.print(F("Ground Speed: "));
        // Serial.print(gps_speed / 1000.0);
        // Serial.print(F(" m/s"));
        // Serial.print(F(" VelN: "));
        // Serial.print(gps_velN / 1000.0);
        // Serial.print(F(" m/s"));
        // Serial.print(F(" VelE: "));
        // Serial.print(gps_velE / 1000.0);
        // Serial.print(F(" m/s"));
        // Serial.print(F(" VelD: "));
        // Serial.print(gps_velD / 1000.0);
        // Serial.print(F(" m/s"));
    }

    // // Serial.print("Baro1: "); 
    // Serial.print(baro1); 
    // Serial.print(" ");
    // // Serial.print("Baro2: "); 
    // Serial.print(baro2); 
    // Serial.print(" ");
    // // Serial.print("Baro3: "); 
    // Serial.print(baro3); 
    // Serial.print(" ");
    // // Serial.print("Temp: "); 
    // Serial.print(temperature); 
    // Serial.print(" ");
    // // Serial.println();   

    return {time, baro1, baro2, baro3, temperature1 + 273.15, gps_latitude, gps_longitude, gps_altitude, gps_velN, gps_velE, gps_velD, gyroX1, gyroY1, gyroZ1, gyroX2, gyroY2, gyroZ2, gyroX3, gyroY3, gyroZ3};
}

// Function to write data to the navFlight log file
// This function takes a vector of data and a vector of state variables as input
void write_data(const vector<double>& data, const vector<double>& state) {
    // Write the data to the navFlight log file
    NavLog.print(data[0]); // Time(ms)

    for(int j = 0; j < 15; j++) {
        NavLog.print(",");
        NavLog.print(state[j]);
    }
    // NavLog.println();

    // if(i%10 == 0) {
    //     NavLog.flush(); // Ensure data is written to the SD card
    // }
    // i++;
}

void setup(void)
{
    Serial.begin(115200); // 110, 300, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200
    Serial.println("serial complete");
    Wire.begin();
    Wire2.begin();

    
    if (!SD.begin(chipSelect)) {
        Serial.println("SD card initialization failed!");
        return;
    }
    Serial.println("SD card initialized.");
    // Supprimer le fichier existant pour le recréer
    if (SD.exists("navFlight.csv")) {
        SD.remove("navFlight.csv");
    }
    // Create or open the navFlight log file
    NavLog = SD.open("navFlight.csv", FILE_WRITE);
    if (!NavLog) {
        Serial.println("Failed to create navFlight log file!");
        return;
    }
    // Write the CSV header
    NavLog.println("Time(ms), X(m), Y(m), Z(m), VX(m/s), VY(m/s), VZ(m/s), AX(m/s^2), AY(m/s^2), AZ(m/s^2), OX(degrees), OY(degrees), OZ(degrees), WX(degrees/s), WY(degrees/s), WZ(degrees/s), execution_time");
    NavLog.flush();


    while (myGNSS.begin() == false) //on attend la connection en I2C
    {
        Serial.println(F("u-blox GNSS not detected at default I2C address. Retrying..."));
        delay(100);
    }
    Serial.println("GNSS module connected");
    myGNSS.setI2COutput(COM_TYPE_UBX);// on veut recevoir seulement les données de position on veut pas de RTCM ou de NMEA
    myGNSS.setNavigationFrequency(20, VAL_LAYER_RAM);// Réglage de la fréquence à 5 Hz (plus vite le RTK ne suit pas) 
    // la configuration VAL_LAYER_RAM ne met la frequance que dans la ram a enlever si besoin
    myGNSS.setAutoPVT(true); // Active l’envoi automatique des messages NAV-PVT
    myGNSS.setI2CpollingWait(10);


    while (!bno1.begin()) {
        Serial.println("Ooops, no BNO055 1 detected ... Check your wiring or I2C ADDR!"); 
        delay(100);
    }
    Serial.println("bno 1 complete");

    // while (!bno2.begin()) {
    //     Serial.println("Ooops, no BNO055 2 detected ... Check your wiring or I2C ADDR!"); 
    //     delay(100);
    // }
    // Serial.println("bno 2 complete");
    
    // while (!bno3.begin()) {
    //     Serial.println("Ooops, no BNO055 3 detected ... Check your wiring or I2C ADDR!"); 
    //     delay(100);
    // }
    // Serial.println("bno 3 complete");

    while(bmp1.beginI2C(0x47, Wire) != BMP5_OK) {
        Serial.println("Error: BMP581 1 not connected, check wiring and I2C address!"); 
        delay(100);
    }
    Serial.println("bmp 1 complete");
    
    while(bmp2.beginI2C(0x47, Wire) != BMP5_OK) {
        Serial.println("Error: BMP581 2 not connected, check wiring and I2C address!"); 
        delay(100);
    }
    Serial.println("bmp 2 complete");
    while(bmp3.beginI2C(0x46, Wire2) != BMP5_OK) {
        Serial.println("Error: BMP581 3 not connected, check wiring and I2C address!"); 
        delay(100);
    }
    Serial.println("bmp 3 complete");

    // lit les premières valeurs dand le vide au cas ou le capteur est pas encore stable
    for (int i = 0; i < 5; ++i) {
        read_data();
        delay(1000);
    }

    //Fait une moyenne des 50 premières valeurs de pression et de temperature pour initialiser la navigation avec ca
    double p01 = 0;
    int count1  = 0;
    double p02 = 0;
    int count2  = 0;
    double p03 = 0;
    int count3  = 0;
    double t0 = 0;
    int count4  = 0;
    for (int i = 0; i < 50; ++i) {
        double p1 = read_data()[1];
        double p2 = read_data()[2];
        double p3 = read_data()[3];
        double t = read_data()[4] - 273.15;
        if (p1 > 100) {
            p01 += p1;
            count1++;
        }
        if (p2 > 100) {
            p02 += p2;
            count2++;
        }
        if (p3 > 100) {
            p03 += p3;
            count3++;
        }
        if (t > 1) {
            t0 += t;
            count4++;
        }
        delay(10);
    }
    p01 = p01 / count1;
    p02 = p02 / count2;
    p03 = p03 / count3;
    t0 = 273.15 + t0 / count4;

    //initialise la navigation
    vector<double> data = read_data();
    data[1] = p01;
    data[2] = p02;
    data[3] = p03;
    data[4] = t0;
    nav.init(data);
    vector<double> state = nav.get_state();
    Serial.print("X = ");
    Serial.print(state[0]);
    Serial.print("Y = ");
    Serial.print(state[1]);
    Serial.print("Z = ");
    Serial.print(state[2]);
    
    Serial.println("Setup complete");
}

void loop(void)
{
    double startTime = millis();

    // Serial.println("test");
    // blink led
    // digitalWrite(LED_BUILTIN, HIGH);
    // delay(500);
    // digitalWrite(LED_BUILTIN, LOW);
    // delay(500);

    //lit les nouvelles données
    std::vector<double> data = read_data();

    // Serial.print(data[1]); Serial.print(data[2]); Serial.print(data[3]); Serial.println();

    //appel l'update de la navigation avec les nouvelles données
    nav.update(data);

    // Print the state
    vector<double> state = nav.get_state();
    Serial.print(" Time: ");
    Serial.println(data[0]);
    Serial.print(" X = ");
    Serial.print(state[0]);
    Serial.print(" Y = ");
    Serial.print(state[1]);
    Serial.print(" Z = ");
    Serial.print(state[2]);
    Serial.print(" VX = ");
    Serial.print(state[3]);
    Serial.print(" VY = ");
    Serial.print(state[4]);
    Serial.print(" VZ = ");
    Serial.print(state[5]);
    Serial.print(" AX = ");
    Serial.print(state[6]);
    Serial.print(" AY = ");
    Serial.print(state[7]);
    Serial.print(" AZ = ");
    Serial.print(state[8]);
    Serial.print(" OX = ");
    Serial.print(state[9]);
    Serial.print(" OY = ");
    Serial.print(state[10]);
    Serial.print(" OZ = ");
    Serial.print(state[11]);
    Serial.print(" WX = ");
    Serial.print(state[12]);
    Serial.print(" WY = ");
    Serial.print(state[13]);
    Serial.print(" WZ = ");
    Serial.print(state[14]);
    Serial.println();

    write_data(data, state);

    //delay(10);

    double loopTime = millis() - startTime; // temps depuis le début de la boucle

    if(loopTime < 10) {
        delay(10 - loopTime); // on attend le temps qu'il reste pour faire 10ms
    }

    loopTime = millis() - startTime; // temps depuis le début de la boucle

    NavLog.print(",");
    NavLog.print(loopTime);
    NavLog.println();

    if(i%10 == 0) {
        NavLog.flush(); // Ensure data is written to the SD card
    }
    i++;
}
