//
// Created by alberts on 20/03/24.
//

#ifndef CONTROLLER_PROTO_PID_H
#define CONTROLLER_PROTO_PID_H

#include <list>
#include "Vec3.h"

class PID {
public:
    Vec3 evaluate(const Vec3& error);
    Vec3 evaluate(const Vec3& sp, const Vec3& val);

    PID(Vec3 pGain_, Vec3 iGain_, Vec3 dGain_, double dt_, Vec3 limit_, Vec3 initialInt_);
    PID(Vec3 pGain_, Vec3 iGain_, Vec3 dGain_, double dt_, Vec3 limit_);
    PID(Vec3 pGain_, Vec3 iGain_, Vec3 dGain_, double dt_, double limit_);
    PID(Vec3 pGain_, double dt_, double limit_);
    PID(Vec3 pGain_, double dt_, Vec3 limit_);

    void reset();

    /**
     * Returns the current state of the PID controller, in the following order:
     * setpoint, value, output, error, p contribution, i contribution, d contribution
     * @return
     */
    std::list<double> getState();

private:
    const double dFilter = 0.7;

    Vec3 pGain;
    Vec3 iGain;
    Vec3 dGain;
    double dt;

    Vec3 currSP = Vec3(0, 0, 0);
    Vec3 currVal = Vec3(0, 0, 0);
    Vec3 currResult = Vec3(0, 0, 0);
    Vec3 currDContribution = Vec3(0, 0, 0);
    Vec3 currIContribution = Vec3(0, 0, 0);
    Vec3 currPContribution = Vec3(0, 0, 0);
    
    Vec3 currError = Vec3(0, 0, 0);
    Vec3 currD = Vec3(0, 0, 0);

    Vec3 limit;
    Vec3 initialInt = Vec3(0, 0, 0);
};


#endif //CONTROLLER_PROTO_PID_H
