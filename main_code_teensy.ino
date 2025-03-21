// Nescessary includes for UART
#include "com_client/inc/PacketID.h"
#include "com_client/inc/Payload.h"
#include "com_client/inc/UART.h"
#include "com_client/inc/TeensyUART.h"
#include "com_client/src/Payload.cpp"
#include "com_client/src/UART.cpp"
#include "com_client/src/TeensyUART.cpp"

// All of the types from control
#include "inc/Vec3.h"
#include "inc/State.h"
#include "inc/SetpointSelection.h"

// Arduino libraries
#include <SD.h>
#include <ArduinoJson.h>

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
enum ControlMode {
    ATTITUDE,
    RATE,
    ALTITUDE,
    ALTITUDE_VEL,
};

struct RemoteInput {
    State desired;
    SetpointSelection setpointSelection;
    std::optional<double>   inline_thrust = std::nullopt;
    ControlMode controlMode;
    bool arm = false;
};

struct SensorData {
    State state;
    double attitude_count=0;
    double rate_count=0;
    double pressure_count=0;
};

struct RawOutput {
    double d1; // in degrees
    double d2; // in degrees
    double avg_throttle; // from 0 to 1
    double throttle_diff; // from -1 to 1, top_throttle - bot_throttle
};

struct FlightNumber {
    int session_number = 0;
    int flight_number = 0;
  };

struct ControlInput
{
    bool armed;
    State desired_state;
    State current_state;
    SetpointSelection setpointSelection;
    double inline_thrust;
};

struct ControlOutput{
    double d1 = 0;
    double d2 = 0;
    double thrust = 0;
    double mz = 0;

    ControlOutput toDegrees(){
        return {d1*180/M_PI, d2*180/M_PI, thrust, mz};
    }
};

ControlOutput received_control_output;
bool control_output_received = false;

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

        ControlInput control_input = {false, ERROR_STATE, ERROR_STATE, ATTITUDE_CONTROL_SELECTION, 0.0};
        SendControlInput(control_input);
    }
    // armed behavior
    else
    {
        SensorData sensor_data = read_sensors();

        ControlInput control_input = {true, remote_input.desired, sensor_data.state, remote_input.setpointSelection, remote_input.inline_thrust.value_or(0)};
        SendControlInput(control_input);

        if (control_output_received)
        {
            RawOutput raw_out = to_raw(received_control_output.toDegrees());
        
            write_raw_outputs(raw_out);

            print_state(loop_start - timer, raw_out, sensor_data);
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
