#include "flight_machine.hpp"

FlightMachine::FlightMachine() {
    // nothing here
}

void FlightMachine::start() {
    // wire context pointers to the concrete state instances
    ctx.prelaunchState = &prelaunch;
    ctx.boostState = &boost;
    ctx.coastState = &coast;
    ctx.apogeeState = &apogee;
    ctx.descentState = &descent;
    ctx.landedState = &landed;

    // initial state
    ctx.setState(&prelaunch);
}

void FlightMachine::update() {
    ctx.updateState();
}
