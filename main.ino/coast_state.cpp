#include "coast_state.hpp"
#include "apogee_state.hpp"
#include "flight_context.hpp"
#include "distance_alg.hpp"
#include "flight_config.hpp"
#include <Arduino.h>


void CoastState::enter(FlightContext& ctx) {
    // Serial.println(F("[STATE] Coast enter"));
    ctx.logBegin();
    ctx.logPrintln(F("[STATE] Coast enter"));
    ctx.logEnd();
    
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
        // Serial.print(F("[EVENT] Apogee detected -> ApogeeState"));
        // Serial.print(F(" | velocity: "));
        // Serial.print(velocity, 2);
        // Serial.println(F(" m/s"));

        ctx.logBegin();
        ctx.logPrint(F("[EVENT] Apogee detected -> ApogeeState"));
        ctx.logPrint(F(" | velocity: "));
        ctx.logPrint(velocity, 2);
        ctx.logPrintln(F(" m/s"));
        ctx.logEnd();

        ctx.setState(ctx.apogeeState);
    }
}

void CoastState::exit(FlightContext& ctx) {
    // Serial.println(F("[STATE] Coast exit"));
    ctx.logBegin();
    ctx.logPrintln(F("[STATE] Coast exit"));
    ctx.logEnd();
}