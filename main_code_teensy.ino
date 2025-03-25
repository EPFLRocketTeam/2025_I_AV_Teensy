// Necessary includes for UART
// These unfortunately have to be included in the main file
#include "com_client/inc/Vec3.h"
#include "com_client/inc/State.h"
#include "com_client/inc/SetpointSelection.h"
#include "com_client/inc/Packets.h"
#include "com_client/inc/Payload.h"
#include "com_client/inc/UART.h"
#include "com_client/inc/TeensyUART.h"
#include "com_client/src/Payload.cpp"
#include "com_client/src/UART.cpp"
#include "com_client/src/TeensyUART.cpp"

// Arduino libraries
#include <ArduinoJson.h>
#include <SD.h>

// C++ libraries
#include <list>
#include <memory>

#define MAX_WIDTH 2000. // max throttle,
#define MIN_WIDTH 1000. // min throttle,

#define MAX_TILT 10.   // the maximum angle the drone can be asked to tilt
#define LOOP_PERIOD 10 // ends up being closer to 14 in reality
#define MAX_THRUST 19  // max thrust

#define SD_FLUSH_PERIOD 1000 // ms
uint32_t lastSDWrite = 0;

// offsets in degrees due to the cg not being in the center
#define outer_offset 0
#define inner_offset 3

// enum of the different control modes
enum ControlMode
{
    ATTITUDE,
    RATE,
    ALTITUDE,
    ALTITUDE_VEL,
};

struct RemoteInput
{
    State desired;
    SetpointSelection setpointSelection;
    std::optional<double> inline_thrust = std::nullopt;
    ControlMode controlMode;
    bool arm = false;
};

struct SensorData
{
    State state;
    double attitude_count = 0;
    double rate_count = 0;
    double pressure_count = 0;
};

struct RawOutput
{
    double d1;            // in degrees
    double d2;            // in degrees
    double avg_throttle;  // from 0 to 1
    double throttle_diff; // from -1 to 1, top_throttle - bot_throttle
};

struct FlightNumber
{
    int session_number = 0;
    int flight_number = 0;
};

struct ControlOutput
{
    double d1 = 0;
    double d2 = 0;
    double thrust = 0;
    double mz = 0;

    ControlOutput toDegrees()
    {
        return {d1 * 180 / M_PI, d2 * 180 / M_PI, thrust, mz};
    }
};

RawOutput received_output;

uint32_t timer = millis();

File flightFile;
bool armed = false;

void setup()
{
    Serial.begin(9600);

    FlightNumber current_flight = setup_sd();

    DynamicJsonDocument doc = setup_json();
    write_json(doc, current_flight);

    setup_uart();
    setup_outputs();
    setup_sensors();
    timer = millis();

    Serial.println("Setup finished");
}

const ControlOutput ERROR_OUT = {-1., -1., -1.};
const Vec3 ERROR_VEC = {-1., -1., -1.};
const State ERROR_STATE = {ERROR_VEC, ERROR_VEC, ERROR_VEC, ERROR_VEC};

void loop()
{
    update_uart();

    long loop_start = millis();
    const RemoteInput remote_input = get_remote_input();

    // armed state transitions
    if (!armed && remote_input.arm)
    {
        create_flight_file();
        timer = loop_start;
        Serial.print("Armed\n");
        armed = true;
    }
    if (armed && !remote_input.arm)
    {
        armed = false;
        close_flight_file();
        Serial.print("Disarmed\n");
    }

    // disarmed behavior
    if (!armed)
    {
        unarmed_output();

        ControlInputPacket control_input;
        control_input.armed = false;
        control_input.timestamp = millis();
        SendControlInput(control_input);
    }
    // armed behavior
    else
    {
        SensorData sensor_data = read_sensors();

        ControlInputPacket control_input = {
            true,
            millis(),
            remote_input.desired,
            sensor_data.state,
            remote_input.setpointSelection,
            remote_input.inline_thrust.value_or(0)
        };
        SendControlInput(control_input);

        // TODO: Check if sequence number has increased here
        write_raw_outputs(received_output);

        print_state(loop_start - timer, received_output, sensor_data);

        if (loop_start - lastSDWrite > SD_FLUSH_PERIOD)
        {
            flightFile.flush();
        }
    }

    long loop_duration = millis() - loop_start;
    long to_sleep = LOOP_PERIOD - loop_duration;
    if (to_sleep > 0)
    {
        delay(to_sleep);
    }
}
