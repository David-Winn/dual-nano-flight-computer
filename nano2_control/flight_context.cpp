#include "flight_context.hpp"
#include "flight_state.hpp"
#include "flight_config.hpp"
#include "descent_state.hpp"
#include <Arduino_LSM6DS3.h>

FlightContext::FlightContext() {
    lastTickMicros = micros();
    stateEntryTime = 0;
    lastHeartbeatMs = 0;
    currentStateId = STATE_PRELAUNCH;

    // Initialize failsafe
    launchTime = 0;
    failsafeArmed = false;
    failsafeTriggered = false;
}

void FlightContext::initHardware() {
    pinMode(LED0, OUTPUT);
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
    pinMode(LED3, OUTPUT);
    pinMode(BUTTON_1, INPUT);
    pinMode(BUTTON_2, INPUT);
    pinMode(PARACHUTE_PIN, OUTPUT);

    // Initialize UART for communication with Nano 1
    Serial.begin(UART_BAUD_RATE);
    delay(100);

    // Initialize IMU
    if (!IMU.begin()) {
        // IMU failed - signal with LED
        while (1) {
            digitalWrite(LED1, LOW);
        }
    }
    digitalWrite(LED1, HIGH);

    delay(100);

    // Read initial offsets
    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(xA, yA, zA);
        IMU.readGyroscope(xG, yG, zG);
        xAO = xA;
        yAO = yA;
        zAO = zA;
        xGO = xG;
        yGO = yG;
        zGO = zG;
    }

    lastTickMicros = micros();
    stateEntryTime = millis();
    lastHeartbeatMs = millis();

    digitalWrite(LED0, HIGH);  // Indicate hardware init complete
}

void FlightContext::readSensors() {
    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(xA, yA, zA);
        IMU.readGyroscope(xG, yG, zG);
        xA -= xAO;
        yA -= yAO;
        zA -= zAO;
        xG -= xGO;
        yG -= yGO;
        zG -= zGO;
    }
}

void FlightContext::tick() {
    static unsigned long timeLast = micros();
    const int TICK_LENGTH = 10000;
    long uTickLength = (long)(timeLast + TICK_LENGTH - micros());
    if (uTickLength > 0) {
        delayMicroseconds(uTickLength);
    }
    timeLast = micros();

    // Send periodic heartbeat
    if (millis() - lastHeartbeatMs >= HEARTBEAT_INTERVAL_MS) {
        sendHeartbeat();
        lastHeartbeatMs = millis();
    }
}

void FlightContext::setState(FlightState* newState, FlightStateId newStateId) {
    if (currentState == newState) {
        return;
    }

    FlightStateId prevStateId = currentStateId;

    if (currentState) {
        currentState->exit(*this);
    }
    currentState = newState;
    currentStateId = newStateId;
    stateEntryTime = millis();

    // Send state change over UART
    sendStateChange(newStateId, prevStateId);

    if (currentState) {
        currentState->enter(*this);
    }
}

void FlightContext::updateState() {
    if (currentState) {
        currentState->update(*this);
    }
}

unsigned long FlightContext::timeSinceStateEntryMs() const {
    return millis() - stateEntryTime;
}

// Parachute failsafe
void FlightContext::armFailsafe() {
    launchTime = millis();
    failsafeArmed = true;
    failsafeTriggered = false;

    sendEvent(EVT_FAILSAFE_ARMED);
}

void FlightContext::checkFailsafe() {
    if (failsafeArmed && !failsafeTriggered) {
        unsigned long elapsed = millis() - launchTime;

        if (elapsed >= FAILSAFE_TIMEOUT_MS) {
            sendEvent(EVT_FAILSAFE_TRIGGERED);

            digitalWrite(PARACHUTE_PIN, HIGH);
            delay(1000);
            digitalWrite(PARACHUTE_PIN, LOW);

            failsafeTriggered = true;
            failsafeArmed = false;
        }
    }
}

// ---- UART Transmission ----

void FlightContext::uartSendFrame(UartMsgType msgType, const uint8_t* payload, size_t len) {
    // Calculate CRC over message type + payload
    uint8_t crcData[16];
    crcData[0] = msgType;
    for (size_t i = 0; i < len && i < 15; i++) {
        crcData[1 + i] = payload[i];
    }
    uint8_t crcVal = crc8(crcData, 1 + len);

    // Send frame: [SYNC] [MSG_TYPE] [PAYLOAD...] [CRC]
    Serial.write(UART_SYNC_BYTE);
    Serial.write(msgType);
    Serial.write(payload, len);
    Serial.write(crcVal);
}

void FlightContext::sendStateChange(FlightStateId newState, FlightStateId prevState) {
    StateChangePayload payload;
    payload.newState = newState;
    payload.prevState = prevState;
    payload.timestampLow = (uint16_t)(millis() & 0xFFFF);

    uartSendFrame(MSG_STATE_CHANGE, (uint8_t*)&payload, sizeof(payload));
}

void FlightContext::sendEvent(EventCode event, int16_t dataValue) {
    EventPayload payload;
    payload.eventCode = event;
    payload.stateId = currentStateId;
    payload.timestampLow = (uint16_t)(millis() & 0xFFFF);
    payload.dataValue = dataValue;

    uartSendFrame(MSG_EVENT, (uint8_t*)&payload, sizeof(payload));
}

void FlightContext::sendHeartbeat() {
    HeartbeatPayload payload;
    payload.currentState = currentStateId;
    payload.flags = 0;
    if (collecting) payload.flags |= 0x01;
    if (failsafeArmed) payload.flags |= 0x02;
    if (failsafeTriggered) payload.flags |= 0x04;
    payload.timestampLow = (uint16_t)(millis() & 0xFFFF);

    uartSendFrame(MSG_HEARTBEAT, (uint8_t*)&payload, sizeof(payload));
}
