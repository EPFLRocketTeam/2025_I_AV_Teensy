#include "Tester.cpp"
#include "inc/DroneController.h"

#define MAX_WIDTH 2000. // max throttle,
#define MIN_WIDTH 1000. // min throttle,

#define MAX_TILT 15.   // the maximum angle the drone can be asked to tilt
#define LOOP_PERIOD 20 // 50hz

#define ZERO_YAW_DIFF 0.03

#define REMOTE_DIFF_RANGE 0.05 // -> 5% diff

struct DroneState
{
    Vec3 attitude = {0, 0, 0}; // in °
    Vec3 rate = {0, 0, 0}; // in °/s
    int attitude_count = 0;
    int rate_count = 0;
};

struct RemoteInput
{
    Vec3 att_ref = {0, 0, 0}; // in °
    double inline_thrust;
    bool arm = false;
    bool manual = false;
};

struct RawOutput {
    double d1; // in degrees
    double d2; // in degrees
    double avg_throttle; // from 0 to 1
    double throttle_diff; // from -1 to 1, top_throttle - bot_throttle
};

Tester tester;
uint32_t timer = millis();

bool armed = false;
bool manual = true;

void setup()
{
    setup_outputs();
    Serial.begin(9600);
    timer = millis();
}

void loop()
{
    long loop_start = millis();
    RemoteInput remote_input = get_remote();

    // armed state transitions
    if (!armed && remote_input.arm)
    {
        timer = loop_start;
        Serial.print("Armed\n");
        armed = true;
    }
    if (armed && !remote_input.arm)
    {
        armed = false;
        Serial.print("Disarmed\n");
    }

    if (manual && !remote_input.manual){
        tester.reset();
    }
    manual = remote_input.manual;

    // disarmed behavior
    if (!armed)
    {
        unarmed_output();
    }
    // armed behavior
    else
    {
        double throttle_diff = remote_input.att_ref.z * REMOTE_DIFF_RANGE + ZERO_YAW_DIFF;
        RawOutput out = {0., 0., 0., 0.};

        if(manual){
            out = {0., 0., remote_input.inline_thrust, throttle_diff};
        } else {
            double t = tester.get_next_thrust();
            out = {0., 0., t, throttle_diff};

            Serial.println(t, 4);
        }

        write_raw_outputs(out);
    }

    long loop_duration = millis() - loop_start;
    long to_sleep = LOOP_PERIOD - loop_duration;
    if (to_sleep > 0)
    {
        delay(to_sleep);
    }
}
