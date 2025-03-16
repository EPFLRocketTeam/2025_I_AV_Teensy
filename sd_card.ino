// SD card setup
#define SESSION_LOG "last_session.txt"
String log_name = "ERROR.csv";
int session_number = 0;
int flight_number = 0;

void setup_sd()
{
    Serial.print("Initializing SD card...");
    // see if the card is present and can be initialized:
    if (!SD.begin(BUILTIN_SDCARD))
    {
        Serial.println("Card failed, or not present");
        while (1)
        {
            // No SD card, so don't do anything more - stay stuck here
        }
    }
    Serial.println("card initialized.");

    if (SD.exists(SESSION_LOG))
    {
        File session_file = SD.open(SESSION_LOG, FILE_READ);
        session_number = session_file.parseInt() + 1;
        session_file.close();
        SD.remove(SESSION_LOG);
    }

    File session_file = SD.open(SESSION_LOG, FILE_WRITE);
    session_file.print(session_number);
    session_file.close();
}

void close_flight_file()
{
    flightFile.close();
    flight_number++;
}

void create_flight_file()
{
    std::string log_name = std::to_string(session_number) + "_" + std::to_string(flight_number) + ".csv";
    flightFile = SD.open(log_name.c_str(), FILE_WRITE);
    print_cols();
}