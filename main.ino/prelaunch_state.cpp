#include "prelaunch_state.hpp"
#include "boost_state.hpp"
#include "flight_context.hpp"
#include "flight_config.hpp"
#include <Arduino.h>


void PrelaunchState::enter(FlightContext& ctx) {
    // Serial.println(F("[STATE] Prelaunch enter"));
    ctx.logBegin();
    ctx.logPrintln(F("[STATE] Prelaunch enter"));
    ctx.logEnd();
    consecutiveHighAccel = 0;
}

void PrelaunchState::update(FlightContext& ctx) {

    // Detect sustained upward acceleration
    if (ctx.zA > LAUNCH_ACCEL_THRESHOLD) {
        consecutiveHighAccel++;
    } else {
        consecutiveHighAccel = 0;
    }

    // Launch confirmed after consecutive readings above threshold 
    if (consecutiveHighAccel >= LAUNCH_CONSECUTIVE) {
        // Serial.print(F("[EVENT] Launch detected -> BoostState"));
        // Serial.print(F(" | accel: "));
        // Serial.print(-ctx.zA, 2);
        // Serial.println(F(" m/s^2"));

        ctx.logBegin();
        ctx.logPrint(F("[EVENT] Launch detected -> BoostState"));
        ctx.logPrint(F(" | accel: "));
        ctx.logPrint(-ctx.zA, 2);
        ctx.logPrintln(F(" m/s^2"));
        ctx.logEnd();

        // Start telemetry
        ctx.collecting = true;

        // Activate failsafe timer
        ctx.armFailsafe();

        ctx.setState(ctx.boostState);
    }
}

void PrelaunchState::exit(FlightContext& ctx) {
    // Serial.println(F("[STATE] Prelaunch exit"));
    ctx.logBegin();
    ctx.logPrintln(F("[STATE] Prelaunch exit"));
    ctx.logEnd();
}