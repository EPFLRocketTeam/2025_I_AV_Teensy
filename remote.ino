#include "PPMReader.h"

// REMOTE CONTROLLER
// Define PPM pin
#define PPM_PIN 6

// Define the number of channels your PPM signal has
#define CHANNELS 5

// Create PPMReader object
PPMReader ppm(PPM_PIN, CHANNELS);

#define MAX_RATE 100. // in °/s

double thrust_from_remote(double thrust_remote)
{
    if (thrust_remote < 0)
    {
        return 1000;
    }
    else
    {
        double TRHOTTLE_DELTA = MAX_WIDTH - MIN_WIDTH;
        return 1 + MAX_THRUST * (thrust_remote - MIN_WIDTH) / TRHOTTLE_DELTA;
    }
}

AttRemoteInput get_remote_att()
{
    AttRemoteInput res;

    // REMOTE
    double roll_remote = ppm.rawChannelValue(1);
    double pitch_remote = ppm.rawChannelValue(2);
    double throttle_remote = ppm.rawChannelValue(3);
    double yaw_remote = ppm.rawChannelValue(4);
    
    res.arm = (ppm.rawChannelValue(5) > 1700);
    res.att_ref = {
        (double)map(pitch_remote, MIN_WIDTH, MAX_WIDTH, -MAX_TILT, MAX_TILT),
        (double)map(roll_remote, MIN_WIDTH, MAX_WIDTH, -MAX_TILT, MAX_TILT),
        0.};

    res.yaw_rate_ref = map(yaw_remote, MIN_WIDTH, MAX_WIDTH, -MAX_RATE, MAX_RATE);
    res.inline_thrust = thrust_from_remote(throttle_remote);

    return res;
}

RateRemoteInput get_remote_rate(){
    RateRemoteInput res;

    // REMOTE
    double roll_remote = ppm.rawChannelValue(1);
    double pitch_remote = ppm.rawChannelValue(2);
    double throttle_remote = ppm.rawChannelValue(3);
    double yaw_remote = ppm.rawChannelValue(4);
    
    res.arm = (ppm.rawChannelValue(5) > 1700);
    res.rate_ref = {
        map(pitch_remote, MIN_WIDTH, MAX_WIDTH, -MAX_RATE, MAX_RATE),
        map(roll_remote, MIN_WIDTH, MAX_WIDTH, -MAX_RATE, MAX_RATE),
        map(yaw_remote, MIN_WIDTH, MAX_WIDTH, -MAX_RATE, MAX_RATE)
    };

    res.inline_thrust = thrust_from_remote(throttle_remote);
    return res;
}