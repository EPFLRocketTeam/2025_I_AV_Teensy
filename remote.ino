#include "PPMReader.h"

// REMOTE CONTROLLER
// Define PPM pin
#define PPM_PIN 6

// Define the number of channels your PPM signal has
#define CHANNELS 5

// Create PPMReader object
PPMReader ppm(PPM_PIN, CHANNELS);

#define MAX_RATE 100. // in °/s

#define MODE_SWITCH 100 // TODO: change to actual switch

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

double altitude_scaling(double channel_value) {
    const double MIN_ALTITUDE = 0;
    const double MAX_ALTITUDE = 10;
    return (double) map(channel_value, MIN_WIDTH, MAX_WIDTH, MIN_ALTITUDE, MAX_ALTITUDE);
}

double altitude_vel_scaling(double channel_value) {
    const double MIN_ALTITUDE_VEL = -2;
    const double MAX_ALTITUDE_VEL = 2;
    return (double) map(channel_value, MIN_WIDTH, MAX_WIDTH, MIN_ALTITUDE_VEL, MAX_ALTITUDE_VEL);
}

ControlMode get_control_mode() {
    return ControlMode::ATTITUDE;
    if (ppm.rawChannelValue(MODE_SWITCH) > 1500) {
        return ControlMode::ALTITUDE;
    }
    else if (ppm.rawChannelValue(MODE_SWITCH) < 1500) {
        return ControlMode::ATTITUDE;
    }
}

RemoteInput get_remote_input() {
    RemoteInput res;
    ControlMode controlMode = get_control_mode();
    
    double roll_remote = ppm.rawChannelValue(1);
    double pitch_remote = ppm.rawChannelValue(2);
    double throttle_remote = ppm.rawChannelValue(3);
    double yaw_remote = ppm.rawChannelValue(4);
    
    res.arm = (ppm.rawChannelValue(5) > 1700);
    Vec3 att_ref = {
        - (double)map(pitch_remote, MIN_WIDTH, MAX_WIDTH, -MAX_TILT, MAX_TILT),
        (double)map(roll_remote, MIN_WIDTH, MAX_WIDTH, -MAX_TILT, MAX_TILT),
        0.
    };

    att_ref.x -= inner_offset;
    att_ref.y -= outer_offset;

    att_ref = att_ref.toRad();

    Vec3 rate_ref = {
        -map(pitch_remote, MIN_WIDTH, MAX_WIDTH, -MAX_RATE, MAX_RATE),
        map(roll_remote, MIN_WIDTH, MAX_WIDTH, -MAX_RATE, MAX_RATE),
        map(yaw_remote, MIN_WIDTH, MAX_WIDTH, -MAX_RATE, MAX_RATE)
    };
    rate_ref = rate_ref.toRad();
    
    switch (controlMode) {
        case ControlMode::ATTITUDE:
        {
            res.inline_thrust = thrust_from_remote(throttle_remote);
            res.desired = {
                {0, 0, 0},
                {0, 0, 0},
                att_ref,
                {0, 0, rate_ref.z}
            };

            res.setpointSelection = ATTITUDE_CONTROL_YAW_RATE_SELECTION;

            return res;
        }

        case ControlMode::RATE:
        {
            res.inline_thrust = thrust_from_remote(throttle_remote);

            res.desired = {
                {0, 0, 0},
                {0, 0, 0},
                {0, 0, 0},
                rate_ref
            };

            res.setpointSelection = RATE_CONTROL_SELECTION;

            return res;
        }

        case ControlMode::ALTITUDE:
        {
            double desired_altitude = altitude_scaling(throttle_remote);
            res.desired = {
                {0, 0, desired_altitude},
                {0, 0, 0},
                att_ref,
                {0, 0, rate_ref.z}
            };

            res.setpointSelection = ALTITUDE_CONTROL_SELECTION;

            return res;
        }

        case ControlMode::ALTITUDE_VEL:
        {
            double desired_altitude_vel = altitude_vel_scaling(throttle_remote);
            res.desired = {
                {0, 0, 0},
                {0, 0, desired_altitude_vel},
                att_ref,
                {0, 0, rate_ref.z}
            };

            res.setpointSelection = VERTICAL_VELOCITY_CONTROL_SELECTION;

            return res;
        }
    }
}
