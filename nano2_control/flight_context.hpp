#pragma once
#include <Arduino.h>
#include "pins.hpp"
#include "flight_state.hpp"
#include "uart_protocol.hpp"

class FlightContext {
public:
    FlightContext();

    /* Sensor data */
    float xA, yA, zA;
    float xG, yG, zG;

    /* Offsets */
    float xAO, yAO, zAO;
    float xGO, yGO, zGO;

    bool collecting = false;

    /* State machine */
    FlightState* currentState;
    unsigned long stateEntryTime;
    FlightStateId currentStateId;

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
    void tick();

    /* State machine control */
    void setState(FlightState* newState, FlightStateId newStateId);
    void updateState();
    unsigned long timeSinceStateEntryMs() const;

    /* Failsafe methods */
    void armFailsafe();
    void checkFailsafe();

    /* UART transmission (replaces SD logging) */
    void sendStateChange(FlightStateId newState, FlightStateId prevState);
    void sendEvent(EventCode event, int16_t dataValue = 0);
    void sendHeartbeat();

private:
    unsigned long lastTickMicros;
    unsigned long lastHeartbeatMs;

    // UART transmission helpers
    void uartSendFrame(UartMsgType msgType, const uint8_t* payload, size_t len);
};
