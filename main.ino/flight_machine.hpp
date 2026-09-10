#pragma once
#include "flight_context.hpp"
#include "prelaunch_state.hpp"
#include "boost_state.hpp"
#include "coast_state.hpp"
#include "apogee_state.hpp"
#include "descent_state.hpp"
#include "landed_state.hpp"

class FlightMachine {
public:
    FlightMachine();

    // wire states and set initial state
    void start();

    // called from main loop
    void update();

    FlightContext ctx;

private:
    // concrete state instances 
    PrelaunchState prelaunch;
    BoostState boost;
    CoastState coast;
    ApogeeState apogee;
    DescentState descent;
    LandedState landed;
};

