#include "PPMReader.h"

// REMOTE CONTROLLER

// Define PPM pin
#define PPM_PIN 6

// Define the number of channels your PPM signal has
#define CHANNELS 6

// Create PPMReader object
PPMReader ppm(PPM_PIN, CHANNELS);

double thrust_from_remote(double thrust_remote)
{
    if (thrust_remote < 0)
    {
        return 1000;
    }
    else
    {
        double TRHOTTLE_DELTA = MAX_WIDTH - MIN_WIDTH;
        return (thrust_remote - MIN_WIDTH) / TRHOTTLE_DELTA;
    }
}

RemoteInput get_remote()
{
    RemoteInput res;

    // REMOTE
    double roll_remote = ppm.rawChannelValue(1);
    double pitch_remote = ppm.rawChannelValue(2);
    double throttle_remote = ppm.rawChannelValue(3);
    double yaw_remote = ppm.rawChannelValue(4);
    
    res.arm = (ppm.rawChannelValue(5) > 1700);
    res.manual = (ppm.rawChannelValue(6) < 1700);

    res.att_ref = {
        (double)map(pitch_remote, MIN_WIDTH, MAX_WIDTH, -MAX_TILT, MAX_TILT),
        (double)map(roll_remote, MIN_WIDTH, MAX_WIDTH, -MAX_TILT, MAX_TILT),
        (double)map(yaw_remote, MIN_WIDTH, MAX_WIDTH, -1, 1)};

    res.inline_thrust = thrust_from_remote(throttle_remote);

    return res;
}