
// SDCardHandler.h

#ifndef SDCARDHANDLER_H
#define SDCARDHANDLER_H

#include <Arduino.h>
#include <SD.h>
#include <vector>

class SDCardHandler
{
public:
    SDCardHandler(const char *sessionLogFileName = "last_session.txt");

    void setup();
    void createFlightFile();
    void closeFlightFile();
    File &getFlightFile();

    int getSessionNumber() const;
    int getFlightNumber() const;

    // Logging methods
    void printColumns();
    void printData(double time, double altitude, double speed);

private:
    const char *sessionLogFileName;
    String logFileName;
    int sessionNumber;
    int flightNumber;
    File flightFile;

    void writeSessionNumber();

    // Helper methods
    void doublePrint(const String &s);
    void printLine(const std::vector<double> &elements);
};

#endif // SDCARDHANDLER_H

