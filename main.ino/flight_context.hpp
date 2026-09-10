#pragma once
#include <Arduino.h>
#include <SD.h>
#include "pins.hpp"
#include "flight_state.hpp"


static constexpr int TELEMETRY_BUFFER_SIZE = 100;

struct TelemSample {
    float xA, yA, zA;
    float xG, yG, zG;
    unsigned long timestamp;
};

class FlightContext {
public:
    FlightContext();

    /* Sensor data */
    float xA, yA, zA;
    float xG, yG, zG;

    /* Offsets */
    float xAO, yAO, zAO;
    float xGO, yGO, zGO;

    char logFileName[20];
    File _logFile;

    char fileName[20]; 
    
    bool collecting = false;

    /* State machine */
    FlightState* currentState;
    unsigned long stateEntryTime;

    // state pointers
    class PrelaunchState* prelaunchState;
    class BoostState*     boostState;
    class CoastState*     coastState;
    class ApogeeState*    apogeeState;
    class DescentState*   descentState;
    class LandedState*    landedState;

    /* Failsafe members*/
    unsigned long launchTime;
    bool failsafeArmed;
    bool failsafeTriggered;

    /* Hardware logic */
    void initHardware();
    void readSensors();
    void bufferTelemetry();
    void flushTelemetry();
    void stopTelemetry();
    void tick();

    /* State machine control */
    void setState(FlightState* newState);
    void updateState();
    unsigned long timeSinceStateEntryMs() const;
    /* Failsafe moethds */
    void armFailsafe();
    void checkFailsafe();
    void logBegin();

    /* Logging */
    void logEnd();
    void logPrint(const __FlashStringHelper* msg);
    void logPrint(const char* msg);
    void logPrint(float val, int decimals = 2);
    void logPrint(unsigned long val);
    void logPrintln(const __FlashStringHelper* msg);
    void logPrintln(const char* msg);
    void logPrintln();

private:
    unsigned long lastTickMicros;

    TelemSample buffer[TELEMETRY_BUFFER_SIZE];
    int bufferCount;

public:
    bool isBufferFull() const { return bufferCount >= TELEMETRY_BUFFER_SIZE; }

};