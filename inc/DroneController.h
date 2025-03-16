//
// Created by alberts on 26/03/24.
//

#include "PID.h"
#include "Controller.h"
#include <math.h>

#ifndef CONTROLLER_PROTO_DRONECONTROLLER_H
#define CONTROLLER_PROTO_DRONECONTROLLER_H

/**
 * Defines some constants and a controller in case of the drone
 */


constexpr double M = 1.508;
constexpr double R = 0.12;// 12 cm;
constexpr double G = 9.81;

constexpr double ANG_RATE_LIMIT = 1;
const Vec3 ACC_LIMIT = {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), G};
constexpr double SPEED_LIMIT = std::numeric_limits<double>::infinity();

constexpr double GIMBAL_LIMIT = 15 * M_PI / 180;
constexpr double MAX_THRUST = 19;

//constexpr double TORQUE_LIMIT = 0.55;
constexpr double TORQUE_LIMIT_XY = MAX_THRUST * R * std::sin(GIMBAL_LIMIT);
constexpr double TORQUE_LIMIT_Z = 2;

constexpr double GIMBAL_RATE_LIMIT = 100 * M_PI / 180;
constexpr double THRUST_RATE_LIMIT = MAX_THRUST * 2;

const PID rateControl = {{0.08, 0.08, 0.8}, {0.6, 0.6, 0.5}, {0.035, 0.035, 0.01}, Controller::RATE_DT, {TORQUE_LIMIT_XY, TORQUE_LIMIT_XY, TORQUE_LIMIT_Z}, {0., 0., 0.}};
const PID attControl = {{3.5, 3.5, 0.05}, Controller::ATT_DT, ANG_RATE_LIMIT};
const PID velControl = {{4.5, 4.5, 4.5}, {0, 0, 0}, {0, 0, 0.04}, Controller::VEL_DT, ACC_LIMIT};
const PID posControl = {{1.3, 1.3, 1.45}, Controller::POS_DT, SPEED_LIMIT};

const Controller DRONE_CONTROLLER = {rateControl, attControl, velControl, posControl,
                                     M, R, GIMBAL_LIMIT, GIMBAL_RATE_LIMIT,
                                     MAX_THRUST,THRUST_RATE_LIMIT};

#endif //CONTROLLER_PROTO_DRONECONTROLLER_H
