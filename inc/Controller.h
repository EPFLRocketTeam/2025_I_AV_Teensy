//
// Created by alberts on 20/03/24.
//

#ifndef CONTROLLER_PROTO_CONTROLLER_H
#define CONTROLLER_PROTO_CONTROLLER_H

#include "PID.h"

/**
 * The control output of the controller
 * @param d1 the deviation from horizontal of the outside servo,
 *             a deviation causes a moment in the x axis
 * @param d2 the deviation from horizontal of the inside servo,
 *             a deviation causes a moment in the y axis
 * @param thrust
*/

struct ControlOutput{
    double d1 = 0;
    double d2 = 0;
    double thrust = 0;
    double mz = 0;

    ControlOutput toDegrees(){
        return {d1*180/M_PI, d2*180/M_PI, thrust, mz};
    }
};

class Controller {
public:
    /**
     * Returns the desired thrust and gimbal angles to reach the desired position
     * Angles are in radians
     * @param desiredPos the desired position
     * @param currPos the current position
     * @param currVel the current velocity
     * @param currAtt the current angles
     * @param currRate the current rates
     * @return thrust and gimbal angles to apply
     */
    ControlOutput posControlR(const Vec3& desiredPos, const Vec3& currPos, const Vec3& currVel, const Vec3& currAtt, const Vec3& currRate);
    ControlOutput posControlD(const Vec3& desiredPos, const Vec3& currPos, const Vec3& currVel, const Vec3& currAtt, const Vec3& currRate);


    ControlOutput velControlR(const Vec3& desiredVel, const Vec3& currVel, const Vec3& currAtt, const Vec3& currRate);


    /**
     * Returns the desired thrust and gimbal angles to reach the desired attitude
     * Angles are in radians
     * @param inlineThrust_
     * @param desiredAtt
     * @param currAtt
     * @param currRate
     * @return thrust and gimbal angles to apply
     */
    ControlOutput attControlR(double inlineThrust_, const Vec3& desiredAtt, double desiredYawRate, const Vec3& currAtt, const Vec3& currRate);
    ControlOutput attControlD(double inlineThrust_, const Vec3& desiredAtt, double desiredYawRate, const Vec3& currAtt, const Vec3& currRate);

    /**
     * Returns the desired thrust and gimbal angles to reach the desired rate
     * Angles are in radians
     * @param inlineThrust_
     * @param desiredRate
     * @param currRate
     * @return thrust and gimbal angles to apply
     */
    ControlOutput rateControlR(double inlineThrust_, const Vec3& desiredRate, const Vec3& currRate, const Vec3& currAtt);
    ControlOutput rateControlD(double inlineThrust_, const Vec3& desiredRate, const Vec3& currRate, const Vec3& currAtt);

    /**
     * Returns the desired thrust and gimbal angles to reach the desired altitude
     * Angles are in radians
     * Will try to keep the vehicle attitude at 0, 0, 0
     * @param desiredAlt the desired altitude
     * @param currentAlt the current altitude
     * @param currentAltVel the current vertical velocity
     * @param currAtt the current angles
     * @param currRate the current rates
     * @return thrust and gimbal angles to apply
     */
    ControlOutput altControlR(const double desiredAlt, const double currentAlt, const double currentAltVel, const Vec3& currAtt, const Vec3& currRate);
    ControlOutput altControlD(const double desiredAlt, const double currentAlt, const double currentAltVel, const Vec3& currAtt, const Vec3& currRate);

    /**
     * Returns the current state of the controller in the following order:
     * - state of the rate controller
     * - state of the attitude controller
     * - state of the velocity controller
     * - state of the position controller
     *
     * @return
     */
    std::list<double> getState();

    Controller(PID rateCont_, PID attCont_, PID velCont_, PID posCont_, double m_, double r_, double gimbalLimit_, double maxGimbalRate_ , double maxThrust_, double maxThrustRate_);

    void reset();

private:
    PID rateCont;
    PID attCont;
    PID velCont;
    PID posCont;
    const double m;
    const double r;
    const double gimbalLimit;
    const double maxThrust;
    const double maxGimbalRate;
    const double maxThrustRate;

    int iter = 0;
    static constexpr int RATE_DIV = 1;
    static constexpr int ATT_DIV = 2*RATE_DIV;
    static constexpr int VEL_DIV = 2*ATT_DIV;
    static constexpr int POS_DIV = 2*VEL_DIV;

    static constexpr double G = 9.81;

    Vec3 intermediateVelSP = {0, 0, 0};
    Vec3 intermediateAttSP = {0, 0, 0};
    Vec3 intermediateRateSP = {0, 0, 0};
    double inlineThrust = 0;
    ControlOutput prevOut = {0, 0, 0};

    ControlOutput thrustAndDeltaAllocation(double fz, Vec3 moments);

public:
    static constexpr double RATE_DT = 0.014; // 14 ms, 71hz
    static constexpr double ATT_DT = ATT_DIV*RATE_DT;
    static constexpr double VEL_DT = VEL_DIV*RATE_DT;
    static constexpr double POS_DT = POS_DIV*RATE_DT;

};


#endif //CONTROLLER_PROTO_CONTROLLER_H
