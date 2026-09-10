#include "landed_state.hpp"
#include "flight_context.hpp"
#include "flight_config.hpp"
#include <Arduino.h>

void LandedState::enter(FlightContext& ctx) {
    // Stop telemetry collection on landing
    ctx.collecting = false;
}

void LandedState::update(FlightContext& ctx) {
    // Do nothing; stay in landed state
}

void LandedState::exit(FlightContext& ctx) {
    // Nothing needed
}
