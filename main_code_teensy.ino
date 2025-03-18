// Nescessary includes for UART
#include "com_client/inc/PacketID.h"
#include "com_client/inc/Payload.h"
#include "com_client/inc/UART.h"
#include "com_client/inc/TeensyUART.h"

#include "com_client/src/Payload.cpp"
#include "com_client/src/UART.cpp"
#include "com_client/src/TeensyUART.cpp"

#include "inc/DroneController.h"

#include <SD.h>

#define MAX_WIDTH 2000. // max throttle,
#define MIN_WIDTH 1000. // min throttle,

#define MAX_TILT 15.   // the maximum angle the drone can be asked to tilt
#define LOOP_PERIOD 10 // ends up being closer to 14 in reality

#define SD_FLUSH_PERIOD 1000 // ms
uint32_t lastSDWrite = 0;

struct DroneState
{
    Vec3 attitude = {0, 0, 0}; // in °
    Vec3 rate = {0, 0, 0};     // in °/s
    int attitude_count = 0;
    int rate_count = 0;
};

struct AttRemoteInput
{
    Vec3 att_ref = {0, 0, 0}; // in °
    double inline_thrust;
    double yaw_rate_ref;
    bool arm = false;
};

struct RateRemoteInput
{
    Vec3 rate_ref = {0, 0, 0}; // in °/s
    double inline_thrust;
    bool arm = false;
};

struct RawOutput
{
    double d1;            // in degrees
    double d2;            // in degrees
    double avg_throttle;  // from 0 to 1
    double throttle_diff; // from -1 to 1, top_throttle - bot_throttle
};

struct ControlInput
{
    DroneState state;
    AttRemoteInput remote_input;
};

// Specify the links and initial tuning parameters
Controller my_controller = DRONE_CONTROLLER;

ControlOutput received_control_output;
bool control_output_received = false;

uint32_t timer = millis();

File flightFile;
bool armed = false;

void setup()
{
    setup_outputs();
    my_controller.reset();
    Serial.begin(9600);
    setup_sensors();
    timer = millis();
    setup_sd();
    setup_uart();
}

const ControlOutput ERROR_OUT = {-1., -1., -1.};
const Vec3 ERROR_VEC = {-1., -1., -1.};

void loop()
{
    update_uart();

    long loop_start = millis();
    AttRemoteInput remote_input = get_remote_att();
    // RateRemoteInput remote_input = get_remote_rate();

    // armed state transitions
    if (!armed && remote_input.arm)
    {
        create_flight_file();
        timer = loop_start;
        my_controller.reset();
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
    }
    // armed behavior
    else
    {
        DroneState state = read_sensors();

        ControlInput control_input = {state, remote_input};
        SendControlInput(control_input);

        if (control_output_received)
        {
            RawOutput raw_out = to_raw(received_control_output);
            write_raw_outputs(raw_out);
            print_state(loop_start - timer, raw_out, state, remote_input.inline_thrust);
        }

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
