#pragma once
#include <Arduino.h>
#include <SD.h>
#include "pins.hpp"
#include "uart_protocol.hpp"

static constexpr int TELEMETRY_BUFFER_SIZE = 100;

struct TelemSample {
    float xA, yA, zA;
    float xG, yG, zG;
    unsigned long timestamp;
};

class TelemetryContext {
public:
    TelemetryContext();

    /* Sensor data (local IMU) */
    float xA, yA, zA;
    float xG, yG, zG;

    /* Offsets */
    float xAO, yAO, zAO;
    float xGO, yGO, zGO;

    /* State received from Nano 2 */
    FlightStateId currentState;
    bool collecting;
    bool failsafeArmed;
    bool failsafeTriggered;

    /* File names */
    char logFileName[20];
    char fileName[20];

    /* Hardware init */
    void initHardware();

    /* Sensor reading */
    void readSensors();

    /* Telemetry buffering */
    void bufferTelemetry();
    void flushTelemetry();
    void stopTelemetry();
    bool isBufferFull() const { return bufferCount >= TELEMETRY_BUFFER_SIZE; }

    /* UART receive */
    void processUartRx();

    /* Logging */
    void logBegin();
    void logEnd();
    void logPrint(const __FlashStringHelper* msg);
    void logPrint(const char* msg);
    void logPrint(float val, int decimals = 2);
    void logPrint(unsigned long val);
    void logPrint(int16_t val);
    void logPrintln(const __FlashStringHelper* msg);
    void logPrintln(const char* msg);
    void logPrintln();

    /* Timing */
    void tick();

private:
    File _logFile;
    unsigned long lastTickMicros;

    TelemSample buffer[TELEMETRY_BUFFER_SIZE];
    int bufferCount;

    /* UART receiver state machine */
    enum RxState { RX_WAIT_SYNC, RX_WAIT_TYPE, RX_WAIT_PAYLOAD, RX_WAIT_CRC };
    RxState rxState;
    uint8_t rxMsgType;
    uint8_t rxPayload[16];
    uint8_t rxPayloadLen;
    uint8_t rxPayloadIdx;

    void handleStateChange(const StateChangePayload& payload);
    void handleEvent(const EventPayload& payload);
    void handleHeartbeat(const HeartbeatPayload& payload);
};
