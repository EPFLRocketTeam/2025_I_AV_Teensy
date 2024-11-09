
// SDCardHandler.cpp

#include "SDCardHandler.h"

SDCardHandler::SDCardHandler(const char *sessionLogFileName)
    : sessionLogFileName(sessionLogFileName), sessionNumber(0), flightNumber(0)
{
}

void SDCardHandler::setup()
{
    Serial.print("Initializing SD card...");
    // Keep trying to initialize the SD card until it's present
    while (!SD.begin(BUILTIN_SDCARD))
    {
        Serial.println("Card failed, or not present. Please insert SD card.");
        delay(1000); // Wait for a second before trying again
    }
    Serial.println("Card initialized.");

    // Check if the session log file exists
    if (SD.exists(sessionLogFileName))
    {
        File sessionFile = SD.open(sessionLogFileName, FILE_READ);
        if (sessionFile)
        {
            sessionNumber = sessionFile.parseInt() + 1;
            sessionFile.close();
            SD.remove(sessionLogFileName);
        }
        else
        {
            Serial.println("Error reading session log.");
            sessionNumber = 1; // Default to 1 if reading fails
        }
    }
    else
    {
        sessionNumber = 1; // If no session log, start from session 1
    }

    writeSessionNumber();
}

void SDCardHandler::createFlightFile()
{
    logFileName = String(sessionNumber) + "_" + String(flightNumber) + ".csv";
    flightFile = SD.open(logFileName.c_str(), FILE_WRITE);
    if (flightFile)
    {
        // Write column headers
        printColumns();
        Serial.print("Flight file created: ");
        Serial.println(logFileName);
    }
    else
    {
        Serial.println("Error creating flight file.");
    }
}

void SDCardHandler::closeFlightFile()
{
    if (flightFile)
    {
        flightFile.close();
        flightNumber++;
        Serial.println("Flight file closed.");
    }
    else
    {
        Serial.println("No flight file to close.");
    }
}

File &SDCardHandler::getFlightFile()
{
    return flightFile;
}

int SDCardHandler::getSessionNumber() const
{
    return sessionNumber;
}

int SDCardHandler::getFlightNumber() const
{
    return flightNumber;
}

void SDCardHandler::writeSessionNumber()
{
    File sessionFile = SD.open(sessionLogFileName, FILE_WRITE);
    if (sessionFile)
    {
        sessionFile.print(sessionNumber);
        sessionFile.close();
    }
    else
    {
        Serial.println("Error writing session log.");
    }
}

// --- Logging Methods ---

void SDCardHandler::doublePrint(const String &s)
{
    Serial.print(s);
    if (flightFile)
    {
        flightFile.print(s);
    }
    else
    {
        Serial.println("Error: flightFile is not open.");
    }
}

void SDCardHandler::printColumns()
{
    doublePrint("Time,Altitude,Speed\n");
}

void SDCardHandler::printLine(const std::vector<double> &elements)
{
    String toPrint;
    toPrint.reserve(elements.size() * 12); // Approximate size

    size_t n = elements.size();
    for (size_t i = 0; i < n; ++i)
    {
        toPrint += String(elements[i], 2); // Adjust precision as needed
        if (i != n - 1)
        {
            toPrint += ",";
        }
    }
    toPrint += "\n";

    doublePrint(toPrint);
}

void SDCardHandler::printData(double time, double altitude, double speed)
{
    std::vector<double> toPrint = {time, altitude, speed};
    printLine(toPrint);
}

