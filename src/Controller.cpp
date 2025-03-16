//
// Created by alberts on 20/03/24.
//

/*
#if ARDUINO >= 100
  #include "Arduino.h"
#else
    #include "WProgram.h"
#endif*/

#include <tuple>
#include <algorithm>
#include "../inc/Controller.h"


std::pair<double, Vec3> forcesToAttitude(const Vec3& f) {
    double alpha = atan(f.y / f.z);
    double beta = -atan(cos(alpha) * f.x / f.z);
    double thrust = f.norm();

    return {thrust, {alpha, beta, 0}};
}


double limitRate(double out, double prevOut, double rateLimit, double dt){
    double maxDelta = rateLimit * dt;
    return std::clamp(out, prevOut-maxDelta, prevOut+maxDelta);
}

Controller::Controller(PID rateCont_, PID attCont_, PID velCont_, PID posCont_, double m_, double r_,
                       double gimbalLimit_, double maxGimbalRate_, double maxThrust_, double maxThrustRate_) :
                       rateCont(rateCont_), attCont(attCont_), velCont(velCont_), posCont(posCont_), m(m_),
                                     r(r_), gimbalLimit(gimbalLimit_), maxThrust(maxThrust_), maxGimbalRate(maxGimbalRate_),
                                     maxThrustRate(maxThrustRate_) {}

ControlOutput Controller::thrustAndDeltaAllocation(double inlineThrust_, Vec3 moments){
    double fz = - inlineThrust_;
    double fx = moments.y/r;
    double fy = -moments.x/r;

    double delta1 = atan(-fy/fz);
    double delta2 = atan(cos(delta1) * fx/fz);
    double thrust = Vec3(fx, fy, fz).norm();
    double mz = moments.z;

    delta1 = std::clamp(delta1, -gimbalLimit, gimbalLimit);
    delta2 = std::clamp(delta2, -gimbalLimit, gimbalLimit);
    thrust = std::clamp(thrust, 0., maxThrust);

    return {delta1, delta2, thrust, mz};
}

ControlOutput Controller::posControlR(const Vec3& desiredPos, const Vec3& currPos, const Vec3& currVel, const Vec3& currAtt, const Vec3& currRate) {
    if(iter % POS_DIV == 0){
        intermediateVelSP = posCont.evaluate(desiredPos, currPos);
    }

    return velControlR(intermediateVelSP, currVel, currAtt, currRate);
}

ControlOutput Controller::velControlR(const Vec3& desiredVel, const Vec3& currVel, const Vec3& currAtt, const Vec3& currRate){
    if(iter % VEL_DIV == 0){
        Vec3 desiredAccs = velCont.evaluate(desiredVel, currVel);
        Vec3 desiredForces = (desiredAccs +  Vec3{0, 0, G}) * m;

        std::tie(inlineThrust, intermediateAttSP) = forcesToAttitude(desiredForces);
    }

    return attControlR(inlineThrust, intermediateAttSP, 0, currAtt, currRate);
}

ControlOutput Controller::attControlR(double inlineThrust_, const Vec3 &desiredAtt, double desiredYawRate, const Vec3 &currAtt, const Vec3 &currRate) {
    inlineThrust = inlineThrust_;
    if(iter % ATT_DIV == 0){
        intermediateRateSP = attCont.evaluate(desiredAtt, currAtt);
    }

    intermediateRateSP.z = desiredYawRate; // yaw attitude controller is not used
    Vec3 momentOutput = rateCont.evaluate(intermediateRateSP, currRate);

    iter++;
    return thrustAndDeltaAllocation(inlineThrust_, momentOutput);
}

ControlOutput Controller::rateControlR(double inlineThrust_, const Vec3 &desiredRate, const Vec3 &currRate, const Vec3 &currAtt){
    inlineThrust = inlineThrust_;
    intermediateRateSP = desiredRate;

    Vec3 momentOutput = rateCont.evaluate(intermediateRateSP, currRate);

    Vec3 attLog = attCont.evaluate({0., 0., 0.}, currAtt); // can remove, this is just so that attitude gets logged

    iter++;
    ControlOutput out = thrustAndDeltaAllocation(inlineThrust_, momentOutput);

    out.d1 = limitRate(out.d1, prevOut.d1, maxGimbalRate, RATE_DT);
    out.d2 = limitRate(out.d2, prevOut.d2, maxGimbalRate, RATE_DT);
    out.thrust = limitRate(out.thrust, prevOut.thrust, maxThrustRate, RATE_DT);

    prevOut = out;
    return out;

}

void Controller::reset() {
    rateCont.reset();
    attCont.reset();
    velCont.reset();
    posCont.reset();
    iter = 0;
    intermediateVelSP = {0, 0, 0};
    intermediateAttSP = {0, 0, 0};
    intermediateRateSP = {0, 0, 0};
    inlineThrust = 0;
}

ControlOutput Controller::altControlR(const double desiredAlt, const double currentAlt, const double currentAltVel,
                                      const Vec3 &currAtt, const Vec3 &currRate) {

    // simply use the position control with x and y = 0
    // this will keep the vehicle upright and only control the altitude
    return posControlR({0, 0, desiredAlt}, {0, 0, currentAlt},
                       {0, 0, currentAltVel}, currAtt, currRate);
}

ControlOutput
Controller::posControlD(const Vec3 &desiredPos, const Vec3 &currPos, const Vec3 &currVel, const Vec3 &currAtt,
                        const Vec3 &currRate) {
    ControlOutput out = posControlR(desiredPos, currPos, currVel, currAtt.toRad(), currRate.toRad());
    return out.toDegrees();
}

ControlOutput
Controller::attControlD(double inlineThrust_, const Vec3 &desiredAtt, double desiredYawRate, const Vec3 &currAtt, const Vec3 &currRate) {
    ControlOutput out = attControlR(inlineThrust_, desiredAtt.toRad(), desiredYawRate * M_PI/180, currAtt.toRad(), currRate.toRad());
    return out.toDegrees();
}

ControlOutput Controller::rateControlD(double inlineThrust_, const Vec3 &desiredRate, const Vec3 &currRate, const Vec3 &currAtt){
    ControlOutput out = rateControlR(inlineThrust_, desiredRate.toRad(), currRate.toRad(), currAtt.toRad());
    return out.toDegrees();
}

ControlOutput Controller::altControlD(const double desiredAlt, const double currentAlt, const double currentAltVel,
                                      const Vec3 &currAtt, const Vec3 &currRate) {
    ControlOutput out = altControlR(desiredAlt, currentAlt, currentAltVel, currAtt.toRad(), currRate.toRad());
    return out.toDegrees();
}

std::list<double> Controller::getState() {
    std::list<double> state = rateCont.getState();
    state.splice(state.end(), attCont.getState());
    state.splice(state.end(), velCont.getState());
    state.splice(state.end(), posCont.getState());
    return state;
}



