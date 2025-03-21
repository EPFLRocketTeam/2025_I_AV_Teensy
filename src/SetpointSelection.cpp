#include "../inc/SetpointSelection.h"
#include <iostream>

Vec3 SetpointSelection::selectSetpoint(const Vec3 &desiredStateSP, const Vec3 &pidChainSP, const bool *activeSP)
{
    Vec3 setpoint = pidChainSP;

    if (activeSP[0])
        setpoint.x = desiredStateSP.x;
    if (activeSP[1])
        setpoint.y = desiredStateSP.y;
    if (activeSP[2])
        setpoint.z = desiredStateSP.z;

    return setpoint;
}
