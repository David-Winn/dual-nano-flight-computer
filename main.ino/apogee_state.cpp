#include "coast_state.hpp"
#include "apogee_state.hpp"
#include "flight_context.hpp"
#include "descent_state.hpp"
#include "distance_alg.hpp"
#include "flight_config.hpp"
#include <Arduino.h>


void ApogeeState::enter(FlightContext& ctx) {
    // Serial.println(F("[STATE] Apogee enter"));
    ctx.logBegin();
    ctx.logPrintln(F("[STATE] Apogee enter"));
    ctx.logEnd();
    
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
    
    float distanceFallen = distanceAlg.getDistance();
    float velocity = distanceAlg.getVelocity();

        unsigned long timeAtApogee = millis() - entryMs;
    if (timeAtApogee >= APOGEE_DWELL_MS) {
        // Serial.print(F("[EVENT] Descent detected -> DescentState"));
        // Serial.print(F(" | distance_fallen: "));
        // Serial.print(distanceFallen, 2);
        // Serial.print(F(" m | velocity: "));
        // Serial.print(velocity, 2);
        // Serial.println(F(" m/s"));

        ctx.logBegin();
        ctx.logPrint(F("[EVENT] Descent detected -> DescentState"));
        ctx.logPrint(F(" | distance_fallen: "));
        ctx.logPrint(distanceFallen, 2);
        ctx.logPrint(F(" m | velocity: "));
        ctx.logPrint(velocity, 2);
        ctx.logPrintln(F(" m/s"));
        ctx.logEnd();

        ctx.setState(ctx.descentState);
    }

}

void ApogeeState::exit(FlightContext& ctx) {
    Serial.println(F("[STATE] Apogee state exit"));
    ctx.logBegin();
    ctx.logPrintln(F("[STATE] Apogee state exit"));
    ctx.logEnd();
}