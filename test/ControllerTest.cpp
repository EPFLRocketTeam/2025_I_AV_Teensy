//
// Created by alberts on 26/03/24.
//

#include <iostream>

#include "../inc/Controller.h"
#include "../inc/DroneController.h"
#include "inc/csv.h"
#include "inc/catch.hpp"

#include "inc/matplotlibcpp.h"

namespace plt = matplotlibcpp;

std::vector<ControlOutput> controlAttitudeErrors(Controller C, std::string runName){
    C.reset();
    std::vector<ControlOutput> errors{};

    auto prefix = "test/matlab_runs/";
    io::CSVReader<12> in(prefix + runName);
    in.read_header(io::ignore_extra_column, "d1", "d2", "thrust", "atta", "attb", "attg", "ratea", "rateb", "rateg", "desiredAtta", "desiredAttb", "desiredAttg");
    double d1, d2, t, aa, ab, ag, ra, rb, rg, daa, dab, dag;
    while(in.read_row(d1, d2, t, aa, ab, ag, ra, rb, rg, daa, dab, dag)){
        auto out = C.attControlR(G * M, {daa, dab, dag}, {aa, ab, ag}, {ra, rb, rg});
        ControlOutput err = {out.d1 - d1, out.d2 - d2, out.thrust - t};
        errors.push_back(err);
    }

    return errors;
}

std::vector<ControlOutput> controlPosErrors(Controller C, std::string runName){
    C.reset();
    std::vector<ControlOutput> errors{};

    auto prefix = "test/matlab_runs/";
    io::CSVReader<18> in(prefix + runName);
    in.read_header(io::ignore_extra_column, "d1", "d2", "thrust", "px", "py", "pz", "vx", "vy", "vz", "atta", "attb", "attg", "ratea", "rateb", "rateg", "desiredpx", "desiredpy", "desiredpz");
    double d1, d2, t, px, py, pz, vx, vy, vz, aa, ab, ag, ra, rb, rg, dpx, dpy, dpz;
    while(in.read_row(d1, d2, t, px, py, pz, vx, vy, vz, aa, ab, ag, ra, rb, rg, dpx, dpy, dpz)){
        auto out = C.posControlR({dpx, dpy, dpz}, {px, py, pz}, {vx, vy, vz}, {aa, ab, ag}, {ra, rb, rg});
        ControlOutput err = {out.d1 - d1, out.d2 - d2, out.thrust - t};
        errors.push_back(err);
    }

    return errors;
}

void plotControl(Controller C, std::string runName, bool posControl, bool altControl){
    C.reset();
    std::vector<double> d1s{}, d2s{}, ts{}, ed1s{}, ed2s{}, ets{}, times{}, d1errors{}, d2errors{}, terrors{};

    auto prefix = "test/matlab_runs/";
    // a reader that reads all the columns


    io::CSVReader<22> in(prefix + runName);
    in.read_header(io::ignore_extra_column, "time", "d1", "d2", "thrust","px", "py", "pz", "vx", "vy", "vz", "atta", "attb", "attg", "ratea", "rateb", "rateg", "desiredpx", "desiredpy", "desiredpz", "desiredAtta", "desiredAttb", "desiredAttg");
    double time, d1, d2, t, px, py, pz, vx, vy, vz, aa, ab, ag, ra, rb, rg, dpx, dpy, dpz, daa, dab, dag;
    while(in.read_row(time, d1, d2, t, px, py, pz, vx, vy, vz, aa, ab, ag, ra, rb, rg, dpx, dpy, dpz, daa, dab, dag)){
        ControlOutput out{};
        if(posControl){
            out = C.posControlR({dpx, dpy, dpz}, {px, py, pz}, {vx, vy, vz}, {aa, ab, ag}, {ra, rb, rg});
        } else if (altControl){
            out = C.altControlR(dpz, pz, vz, {aa, ab, ag}, {ra, rb, rg});
        } else {
            out = C.attControlR(G * M, {daa, dab, dag}, {aa, ab, ag}, {ra, rb, rg});
        }

        times.push_back(time);
        d1s.push_back(out.d1);
        d2s.push_back(out.d2);
        ts.push_back(out.thrust);
        ed1s.push_back(d1);
        ed2s.push_back(d2);
        ets.push_back(t);

        d1errors.push_back(out.d1 - d1);
        d2errors.push_back(out.d2 - d2);
        terrors.push_back(out.thrust - t);
    }

    plt::figure();
    plt::title("expected and actual deltas");
    plt::plot(times, d1s, {{"label", "actual d1"}});
    plt::plot(times,ed1s, {{"label", "expected d1"}});
    plt::plot(times,d2s, {{"label", "actual d2"}});
    plt::plot(times,ed2s, {{"label", "expected d2"}});
    plt::legend();

    plt::figure();
    plt::title("errors");
    plt::plot(times, d1errors, {{"label", "d1 error"}});
    plt::plot(times, d2errors, {{"label", "d2 error"}});
    plt::plot(times, terrors, {{"label", "thrust error"}});
    plt::legend();

    plt::figure();
    plt::title("expected and actual thrusts");
    plt::plot(times, ts,  {{"label", "actual"}});
    plt::plot(times, ets, {{"label", "expected"}});
    plt::legend();
}


TEST_CASE("Compare with attitude run 1"){
    auto errs = controlAttitudeErrors(DRONE_CONTROLLER, "att_run_1.csv");

    const double DELTA = 1e-6;
    for(auto e : errs){
        REQUIRE(e.thrust < DELTA);
        REQUIRE(e.d1 < DELTA);
        REQUIRE(e.d2 < DELTA);
    }
}

TEST_CASE("vert pos run"){
    auto errs = controlPosErrors(DRONE_CONTROLLER, "vert_pos_run.csv");

    const double DELTA = 1e-6;
    for(auto e : errs){
        REQUIRE(e.thrust < DELTA);
        REQUIRE(e.d1 < DELTA);
        REQUIRE(e.d2 < DELTA);
    }
}

TEST_CASE("pos run"){
    auto errs = controlPosErrors(DRONE_CONTROLLER, "pos_run_1.csv");

    const double DELTA = 1e-6;
    for(auto e : errs){
        REQUIRE(e.thrust < DELTA);
        REQUIRE(e.d1 < DELTA);
        REQUIRE(e.d2 < DELTA);
    }
}

TEST_CASE("Plot run 1"){
    plotControl(DRONE_CONTROLLER, "latest_altitude_run.csv", false, true);
    plt::show();
}

/*
TEST_CASE("Compare with attitude run 1"){
    auto errs = controlAttitudeErrors(DRONE_CONTROLLER, "att_run_1.csv");

    const double DELTA = 1e-6;
    for(auto e : errs){
        REQUIRE(e.thrust < DELTA);
        REQUIRE(e.d1 < DELTA);
        REQUIRE(e.d2 < DELTA);
    }
}*/