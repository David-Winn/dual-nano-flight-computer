#include "boost_state.hpp"
#include "flight_context.hpp"
#include "coast_state.hpp"
#include "flight_config.hpp"
#include <Arduino.h>

void BoostState::enter(FlightContext &ctx) {
    startZ = ctx.zA;
    stage = 1;
    consecutiveLowAccel = 0;
    ignitionStartTime = 0;

#if STAGE2_ENABLED
    pinMode(STAGE2_IGNITION_PIN, OUTPUT);
    digitalWrite(STAGE2_IGNITION_PIN, LOW);
#endif
}

void BoostState::update(FlightContext &ctx) {
    // Look for sustained low vertical acceleration
    if (ctx.zA < BURNOUT_ACCEL_THRESHOLD) {
        consecutiveLowAccel++;
    } else {
        consecutiveLowAccel = 0;
    }

    if (consecutiveLowAccel >= BURNOUT_CONSECUTIVE) {

#if STAGE2_ENABLED
        if (stage == 1) {
            if (ignitionStartTime == 0) {
                digitalWrite(STAGE2_IGNITION_PIN, HIGH);
                ignitionStartTime = millis();
                ctx.sendEvent(EVT_STAGE2_IGNITION);
                return;
            }

            if (millis() - ignitionStartTime > 500) {
                digitalWrite(STAGE2_IGNITION_PIN, LOW);
                ctx.sendEvent(EVT_STAGE2_IGNITION_END);

                consecutiveLowAccel = 0;
                stage = 2;
                ignitionStartTime = 0;
                return;
            }
            return;
        }
#endif

        // Motor burnout detected
        ctx.sendEvent(EVT_MOTOR_BURNOUT, (int16_t)(ctx.zA * 100));
        ctx.setState(ctx.coastState, STATE_COAST);
    }
}

void BoostState::exit(FlightContext &ctx) {
    // Nothing needed
}
