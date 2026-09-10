#pragma once
#include <Arduino.h>

// UART Protocol for Nano2 (Control) -> Nano1 (Telemetry) communication
// Frame format: [0xAA] [MSG_TYPE] [PAYLOAD...] [CRC8]

static constexpr uint8_t UART_SYNC_BYTE = 0xAA;
static constexpr uint32_t UART_BAUD_RATE = 57600;

// Message types
enum UartMsgType : uint8_t {
    MSG_STATE_CHANGE = 0x01,   // Flight state transition
    MSG_EVENT        = 0x02,   // Event notification (launch, burnout, etc.)
    MSG_HEARTBEAT    = 0x03,   // Periodic heartbeat with current state
};

// Flight states (matches existing state machine)
enum FlightStateId : uint8_t {
    STATE_PRELAUNCH = 0,
    STATE_BOOST     = 1,
    STATE_COAST     = 2,
    STATE_APOGEE    = 3,
    STATE_DESCENT   = 4,
    STATE_LANDED    = 5,
};

// Event codes
enum EventCode : uint8_t {
    EVT_LAUNCH_DETECTED     = 0x01,
    EVT_FAILSAFE_ARMED      = 0x02,
    EVT_MOTOR_BURNOUT       = 0x03,
    EVT_STAGE2_IGNITION     = 0x04,
    EVT_STAGE2_IGNITION_END = 0x05,
    EVT_APOGEE_DETECTED     = 0x06,
    EVT_PARACHUTE_DEPLOY    = 0x07,
    EVT_FAILSAFE_TRIGGERED  = 0x08,
    EVT_LANDING_DETECTED    = 0x09,
};

// State change message payload (4 bytes)
struct StateChangePayload {
    uint8_t newState;           // FlightStateId
    uint8_t prevState;          // FlightStateId
    uint16_t timestampLow;      // Lower 16 bits of millis()
};

// Event message payload (6 bytes)
struct EventPayload {
    uint8_t eventCode;          // EventCode
    uint8_t stateId;            // Current FlightStateId
    uint16_t timestampLow;      // Lower 16 bits of millis()
    int16_t dataValue;          // Optional data (e.g., accel * 100)
};

// Heartbeat payload (4 bytes)
struct HeartbeatPayload {
    uint8_t currentState;       // FlightStateId
    uint8_t flags;              // Bit flags: 0=collecting, 1=failsafeArmed, 2=failsafeTriggered
    uint16_t timestampLow;      // Lower 16 bits of millis()
};

// CRC8 calculation (polynomial 0x07)
inline uint8_t crc8(const uint8_t* data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// Get state name string
inline const char* getStateName(FlightStateId state) {
    switch (state) {
        case STATE_PRELAUNCH: return "Prelaunch";
        case STATE_BOOST:     return "Boost";
        case STATE_COAST:     return "Coast";
        case STATE_APOGEE:    return "Apogee";
        case STATE_DESCENT:   return "Descent";
        case STATE_LANDED:    return "Landed";
        default:              return "Unknown";
    }
}

// Get event name string
inline const char* getEventName(EventCode event) {
    switch (event) {
        case EVT_LAUNCH_DETECTED:     return "Launch detected";
        case EVT_FAILSAFE_ARMED:      return "Failsafe armed";
        case EVT_MOTOR_BURNOUT:       return "Motor burnout";
        case EVT_STAGE2_IGNITION:     return "Stage 2 ignition";
        case EVT_STAGE2_IGNITION_END: return "Stage 2 ignition end";
        case EVT_APOGEE_DETECTED:     return "Apogee detected";
        case EVT_PARACHUTE_DEPLOY:    return "Parachute deploy";
        case EVT_FAILSAFE_TRIGGERED:  return "Failsafe triggered";
        case EVT_LANDING_DETECTED:    return "Landing detected";
        default:                      return "Unknown event";
    }
}
