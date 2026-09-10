#include "coast_state.hpp"
#include "apogee_state.hpp"
#include "flight_context.hpp"
#include "descent_state.hpp"
#include "distance_alg.hpp"
#include "flight_config.hpp"
#include <Arduino.h>

void ApogeeState::enter(FlightContext& ctx) {
    entryMs = millis();

    // Reset distance algorithm at apogee
    distanceAlg.reset();
    lastUpdateMicros = micros();
}

void ApogeeState::update(FlightContext& ctx) {
    unsigned long currentMicros = micros();
    float dtSeconds = (currentMicros - lastUpdateMicros) / 1000000.0f;
    lastUpdateMicros = currentMicros;

    // Update distance algorithm with current acceleration
    distanceAlg.update(ctx.zA, dtSeconds);

    unsigned long timeAtApogee = millis() - entryMs;
    if (timeAtApogee >= APOGEE_DWELL_MS) {
        ctx.setState(ctx.descentState, STATE_DESCENT);
    }
}

void ApogeeState::exit(FlightContext& ctx) {
    // Nothing needed
}
