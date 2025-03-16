//
// Created by alberts on 20/03/24.
//

/*
#if ARDUINO >= 100
#include "Arduino.h"
#else
  #include "WProgram.h"
#*/

#include "../inc/PID.h"
#include <algorithm>

Vec3 clamp(const Vec3 &v, const Vec3 &lo, const Vec3 &hi)
{
  return v.cwiseMax(lo).cwiseMin(hi);
}

Vec3 PID::evaluate(const Vec3 &error)
{
  currIContribution += iGain * pGain * error * dt; // TODO check if this is correct against matlab
  currIContribution = clamp(currIContribution, -limit, limit);

  Vec3 d = (error - currError) / dt;
  currD = dFilter * currD + (1-dFilter) * d;

  currError = error;
  currDContribution = currD * dGain * pGain;
  currPContribution = error * pGain;

  Vec3 res = currPContribution + currIContribution + currDContribution;
  res = clamp(res, -limit, limit);
  
  currResult = res;
  return res;
}

PID::PID(Vec3 pGain_, Vec3 iGain_, Vec3 dGain_, double dt_, Vec3 limit_, Vec3 initialInt_) : pGain(pGain_), iGain(iGain_), dGain(dGain_), dt(dt_), limit(limit_), initialInt(initialInt_) {
  currIContribution = initialInt;
}
PID::PID(Vec3 pGain_, Vec3 iGain_, Vec3 dGain_, double dt_, Vec3 limit_) : pGain(pGain_), iGain(iGain_), dGain(dGain_), dt(dt_), limit(limit_) {}


Vec3 PID::evaluate(const Vec3 &sp, const Vec3 &val)
{
  currSP = sp;
  currVal = val;
  return evaluate(sp - val);
}

PID::PID(Vec3 pGain_, double dt_, double limit_) : PID(std::move(pGain_), Vec3::Zero(), Vec3::Zero(), dt_, limit_) {}

void PID::reset() {
    currError = {0, 0, 0};
    currResult = {0, 0, 0};
    currDContribution = {0, 0, 0};
    currIContribution = initialInt;
    currPContribution = {0, 0, 0};
}

PID::PID(Vec3 pGain_, Vec3 iGain_, Vec3 dGain_, double dt_, double limit_) : PID(std::move(pGain_), std::move(iGain_), std::move(dGain_), dt_, Vec3::Constant(limit_)) {}

PID::PID(Vec3 pGain_, double dt_, Vec3 limit_) : PID(std::move(pGain_), Vec3::Zero(), Vec3::Zero(), dt_, limit_){}

std::list<double> PID::getState() {
    return {
            currSP.x, currSP.y, currSP.z,
            currVal.x, currVal.y, currVal.z,
            currResult.x, currResult.y, currResult.z,
            currError.x, currError.y, currError.z,
            currPContribution.x, currPContribution.y, currPContribution.z,
            currIContribution.x, currIContribution.y, currIContribution.z,
            currDContribution.x, currDContribution.y, currDContribution.z};
}
