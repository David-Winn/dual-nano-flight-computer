#include "boost_state.hpp"
#include "flight_context.hpp"
#include "coast_state.hpp"
#include "flight_config.hpp"
#include <Arduino.h>


void BoostState::enter(FlightContext &ctx)
{
    // Serial.println(F("[STATE] Boost enter"));
    ctx.logBegin();
    ctx.logPrintln(F("[STATE] Boost enter"));
    ctx.logEnd();
    startZ = ctx.zA;
    
    stage = 1;
    consecutiveLowAccel = 0;
    ignitionStartTime = 0;

#if STAGE2_ENABLED

    pinMode(STAGE2_IGNITION_PIN, OUTPUT);
    digitalWrite(STAGE2_IGNITION_PIN, LOW);

#endif
}

void BoostState::update(FlightContext &ctx)
{
    // Look for sustained low vertical acceleration 
    if (ctx.zA < BURNOUT_ACCEL_THRESHOLD)
    {
        // Acceleration has dropped below threshold
        consecutiveLowAccel++;
    }
    else
    {
        // Still burning
        consecutiveLowAccel = 0;
    }

    if (consecutiveLowAccel >= BURNOUT_CONSECUTIVE)
    {

#if STAGE2_ENABLED

        if (stage == 1)
        {

            // float totalAccel = sqrt(ctx.xA*ctx.xA + ctx.yA*ctx.yA + ctx.zA*ctx.zA);
            // float tiltAngle = acos(ctx.zA / totalAccel) * 57.2958; 

            // // Tilt failsafe 
            // if (tiltAngle > MAX_ALLOWED_TILT_DEGREES) {
            //     ctx.logPrintln(F("[CRITICAL] Tilt detected! Disabling Stage 2 for safety."));
            //     stage = 0; 
            //     ctx.setState(ctx.coastState); 
            //     return;
            // }

            if (ignitionStartTime == 0){
                digitalWrite(STAGE2_IGNITION_PIN, HIGH);
                ignitionStartTime = millis();
                ctx.logBegin();
                ctx.logPrintln("[EVENT] Second stage ignition start!");
                ctx.logEnd();
                return;
            }

            if (millis() - ignitionStartTime > 500){

                digitalWrite(STAGE2_IGNITION_PIN, LOW);
                ctx.logBegin();
                ctx.logPrintln(F("[EVENT] Stage 2 Ignition End"));
                ctx.logEnd();

                consecutiveLowAccel = 0; // Reset for stage 2 burnout
                stage = 2;
                ignitionStartTime = 0; // Reset timer for future use
                return;
            }
            return;
        } 
#endif
        
        // Serial.print(F("[EVENT] Motor burnout -> CoastState"));
        // Serial.print(F(" | accel: "));
        // Serial.print(ctx.zA, 2);
        // Serial.println(F(" m/s^2"));

        ctx.logBegin();
        ctx.logPrint(F("[EVENT] Motor burnout -> CoastState"));
        ctx.logPrint(F(" | accel: "));
        ctx.logPrint(ctx.zA, 2);
        ctx.logPrintln(F(" m/s^2"));
        ctx.logEnd();

        ctx.setState(ctx.coastState);
        
    }
}

void BoostState::exit(FlightContext &ctx)
{
    // Serial.println(F("[STATE] Boost exit"));
    ctx.logBegin();
    ctx.logPrintln(F("[STATE] Boost exit"));
    ctx.logEnd();
}