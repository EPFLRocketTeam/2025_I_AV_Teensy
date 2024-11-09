// test_sdcard.ino

#include <Arduino.h>
#include "SDCardHandler.h"

// Create an instance of SDCardHandler
SDCardHandler sdCardHandler;

void setup()
{
    Serial.begin(115200);
    sdCardHandler.setup();

    // Create a new flight file
    sdCardHandler.createFlightFile();

    // Write some data to the flight file
    File &flightFile = sdCardHandler.getFlightFile();
    if (flightFile)
    {
        flightFile.println("Time,Altitude,Speed");
        flightFile.println("0,1000,150");
        flightFile.println("1,1050,155");
        Serial.println("Data written to flight file.");
    }
}

void loop()
{
    // Simulate closing the flight file after some time
    delay(5000); // Wait for 5 seconds
    sdCardHandler.closeFlightFile();

    // Create a new flight file
    sdCardHandler.createFlightFile();

    // Write more data to the new flight file
    File &flightFile = sdCardHandler.getFlightFile();
    if (flightFile)
    {
        flightFile.println("Time,Altitude,Speed");
        flightFile.println("0,1100,160");
        flightFile.println("1,1150,165");
        Serial.println("Data written to new flight file.");
    }

    // Close the flight file and stop the program
    sdCardHandler.closeFlightFile();
    while(1){
        //do nothing
    }
}

