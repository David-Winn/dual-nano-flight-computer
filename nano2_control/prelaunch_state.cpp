#include "prelaunch_state.hpp"
#include "boost_state.hpp"
#include "flight_context.hpp"
#include "flight_config.hpp"
#include <Arduino.h>

void PrelaunchState::enter(FlightContext& ctx) {
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
        // Send launch event over UART
        ctx.sendEvent(EVT_LAUNCH_DETECTED, (int16_t)(ctx.zA * 100));

        // Start telemetry flag
        ctx.collecting = true;

        // Activate failsafe timer
        ctx.armFailsafe();

        ctx.setState(ctx.boostState, STATE_BOOST);
    }
}

void PrelaunchState::exit(FlightContext& ctx) {
    // Nothing needed
}
