#include "coast_state.hpp"
#include "apogee_state.hpp"
#include "flight_context.hpp"
#include "distance_alg.hpp"
#include "flight_config.hpp"
#include <Arduino.h>

void CoastState::enter(FlightContext& ctx) {
    // Initialize distance algorithm to track velocity
    distanceAlg.reset();
    lastUpdateMicros = micros();
    consecutiveNegativeVelocity = 0;
}

void CoastState::update(FlightContext& ctx) {
    // Calculate time delta since last update
    unsigned long currentMicros = micros();
    float dtSeconds = (currentMicros - lastUpdateMicros) / 1000000.0f;
    lastUpdateMicros = currentMicros;

    // Update distance algorithm to track velocity
    distanceAlg.update(ctx.zA, dtSeconds);
    float velocity = distanceAlg.getVelocity();

    // NOTE: Velocity is negative while climbing, flips positive at apogee
    if (velocity > APOGEE_VELOCITY_THRESHOLD) {
        consecutiveNegativeVelocity++;
    } else {
        consecutiveNegativeVelocity = 0;
    }

    if (consecutiveNegativeVelocity >= APOGEE_CONSECUTIVE) {
        ctx.sendEvent(EVT_APOGEE_DETECTED, (int16_t)(velocity * 100));
        ctx.setState(ctx.apogeeState, STATE_APOGEE);
    }
}

void CoastState::exit(FlightContext& ctx) {
    // Nothing needed
}
