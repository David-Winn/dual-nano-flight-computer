#include "descent_state.hpp"
#include "coast_state.hpp"
#include "flight_context.hpp"
#include "landed_state.hpp"
#include "flight_config.hpp"
#include <Arduino.h>

void DescentState::enter(FlightContext& ctx) {
    lastAlt = ctx.zA;
    parachuteDeployed = false;
    stableCount = 0;
    deployStartTime = 0;

    // Initialize distance algorithm at start of descent
    distanceAlg.reset();
    lastUpdateMicros = micros();
}

void DescentState::update(FlightContext& ctx) {
    // Calculate time delta since last update
    unsigned long currentMicros = micros();
    float dtSeconds = (currentMicros - lastUpdateMicros) / 1000000.0f;
    lastUpdateMicros = currentMicros;

    // Update distance algorithm with current acceleration
    distanceAlg.update(ctx.zA, dtSeconds);

    float distanceFallen = distanceAlg.getDistance();
    float velocity = distanceAlg.getVelocity();

    // Deploy parachute after falling DEPLOY_DISTANCE meters
    if (!parachuteDeployed && distanceFallen >= DEPLOY_DISTANCE) {
        if (deployStartTime == 0) {
            digitalWrite(PARACHUTE_PIN, HIGH);
            deployStartTime = millis();
            return;
        }
        if (millis() - deployStartTime > 250) {
            digitalWrite(PARACHUTE_PIN, LOW);

            ctx.sendEvent(EVT_PARACHUTE_DEPLOY, (int16_t)(distanceFallen * 10));

            ctx.failsafeArmed = false;
            parachuteDeployed = true;
        }
    }

    // Detect landing
    float accelMagnitude = fabs(ctx.zA);

    if (parachuteDeployed && velocity < LANDING_VELOCITY_THRESHOLD && accelMagnitude < LANDING_ACCEL_THRESHOLD) {
        stableCount++;
    } else {
        stableCount = 0;
    }

    if (stableCount >= LANDING_CONSECUTIVE) {
        ctx.sendEvent(EVT_LANDING_DETECTED, (int16_t)(velocity * 100));
        ctx.setState(ctx.landedState, STATE_LANDED);
    }
}

void DescentState::exit(FlightContext& ctx) {
    // Nothing needed
}
