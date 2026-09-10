#include "landed_state.hpp"
#include "flight_context.hpp"
#include "flight_config.hpp"
#include <Arduino.h>

void LandedState::enter(FlightContext& ctx) {
    // Serial.println(F("[STATE] Landed enter"));
    ctx.logBegin();
    ctx.logPrintln(F("[STATE] Landed enter"));
    ctx.logEnd();

    // Stop telemetry collection on landing
    ctx.stopTelemetry();
}

void LandedState::update(FlightContext& ctx) {
    // Do nothing; stay in landed state

}

void LandedState::exit(FlightContext& ctx) {
    // Serial.println(F("[STATE] Landed exit "));
    ctx.logBegin();
    ctx.logPrintln(F("[STATE] Landed exit"));
    ctx.logEnd();
}