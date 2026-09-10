#include "telemetry_context.hpp"
#include <Arduino_LSM6DS3.h>

TelemetryContext::TelemetryContext() {
    lastTickMicros = micros();
    fileName[0] = '\0';
    logFileName[0] = '\0';
    bufferCount = 0;
    currentState = STATE_PRELAUNCH;
    collecting = false;
    failsafeArmed = false;
    failsafeTriggered = false;

    // UART receiver state
    rxState = RX_WAIT_SYNC;
    rxPayloadLen = 0;
    rxPayloadIdx = 0;
}

void TelemetryContext::initHardware() {
    pinMode(LED0, OUTPUT);
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
    pinMode(LED3, OUTPUT);

    // Initialize UART for communication with Nano 2
    Serial.begin(UART_BAUD_RATE);
    delay(100);

    // Initialize SD card
    if (!SD.begin(CS_SD)) {
        while (1) {
            digitalWrite(LED0, LOW);
        }
    } else {
        digitalWrite(LED0, HIGH);
    }

    delay(1000);

    // Create telemetry data file
    char testName[20];
    int fileNum = 1;

    strcpy(testName, "data.txt");
    if (!SD.exists(testName)) {
        strcpy(fileName, testName);
    } else {
        do {
            snprintf(testName, sizeof(testName), "data%d.txt", fileNum);
            fileNum++;
        } while (SD.exists(testName) && fileNum < 1000);
        strcpy(fileName, testName);
    }

    File file = SD.open(fileName, FILE_WRITE);
    if (file) {
        file.close();
    }

    // Create event log file
    {
        int logNum = 1;
        char testLog[20];
        strcpy(testLog, "log.txt");
        if (!SD.exists(testLog)) {
            strcpy(logFileName, testLog);
        } else {
            do {
                snprintf(testLog, sizeof(testLog), "log%d.txt", logNum);
                logNum++;
            } while (SD.exists(testLog) && logNum < 1000);
            strcpy(logFileName, testLog);
        }

        File lf = SD.open(logFileName, FILE_WRITE);
        if (lf) {
            lf.println(F("=== Flight Event Log (Nano 1 Telemetry) ==="));
            lf.close();
        }
    }

    // Initialize IMU
    if (!IMU.begin()) {
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

    logBegin();
    logPrintln(F("=== System Ready ==="));
    logEnd();
}

void TelemetryContext::readSensors() {
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

void TelemetryContext::bufferTelemetry() {
    if (!collecting) {
        digitalWrite(LED3, LOW);
        return;
    }

    if (bufferCount < TELEMETRY_BUFFER_SIZE) {
        TelemSample& s = buffer[bufferCount];
        s.xA = xA;
        s.yA = yA;
        s.zA = zA;
        s.xG = xG;
        s.yG = yG;
        s.zG = zG;
        s.timestamp = millis();
        bufferCount++;
    }
    digitalWrite(LED3, HIGH);
}

void TelemetryContext::flushTelemetry() {
    if (bufferCount == 0) return;

    File file = SD.open(fileName, FILE_WRITE);
    if (!file) {
        digitalWrite(LED0, LOW);
        return;
    }

    // Write buffer as binary
    file.write((uint8_t*)buffer, bufferCount * sizeof(TelemSample));
    file.flush();
    bufferCount = 0;
}

void TelemetryContext::stopTelemetry() {
    collecting = false;
    flushTelemetry();
    digitalWrite(LED3, LOW);
}

void TelemetryContext::tick() {
    static unsigned long timeLast = micros();
    const int TICK_LENGTH = 10000;
    long uTickLength = (long)(timeLast + TICK_LENGTH - micros());
    if (uTickLength > 0) {
        delayMicroseconds(uTickLength);
    }
    timeLast = micros();
}

// ---- UART Receiver ----

void TelemetryContext::processUartRx() {
    while (Serial.available()) {
        uint8_t byte = Serial.read();

        switch (rxState) {
            case RX_WAIT_SYNC:
                if (byte == UART_SYNC_BYTE) {
                    rxState = RX_WAIT_TYPE;
                }
                break;

            case RX_WAIT_TYPE:
                rxMsgType = byte;
                rxPayloadIdx = 0;

                // Determine payload length based on message type
                switch (rxMsgType) {
                    case MSG_STATE_CHANGE:
                        rxPayloadLen = sizeof(StateChangePayload);
                        break;
                    case MSG_EVENT:
                        rxPayloadLen = sizeof(EventPayload);
                        break;
                    case MSG_HEARTBEAT:
                        rxPayloadLen = sizeof(HeartbeatPayload);
                        break;
                    default:
                        rxState = RX_WAIT_SYNC;  // Unknown message, reset
                        continue;
                }
                rxState = RX_WAIT_PAYLOAD;
                break;

            case RX_WAIT_PAYLOAD:
                rxPayload[rxPayloadIdx++] = byte;
                if (rxPayloadIdx >= rxPayloadLen) {
                    rxState = RX_WAIT_CRC;
                }
                break;

            case RX_WAIT_CRC: {
                // Verify CRC
                uint8_t crcData[16];
                crcData[0] = rxMsgType;
                for (uint8_t i = 0; i < rxPayloadLen; i++) {
                    crcData[1 + i] = rxPayload[i];
                }
                uint8_t expectedCrc = crc8(crcData, 1 + rxPayloadLen);

                if (byte == expectedCrc) {
                    // Valid frame - process message
                    switch (rxMsgType) {
                        case MSG_STATE_CHANGE:
                            handleStateChange(*(StateChangePayload*)rxPayload);
                            break;
                        case MSG_EVENT:
                            handleEvent(*(EventPayload*)rxPayload);
                            break;
                        case MSG_HEARTBEAT:
                            handleHeartbeat(*(HeartbeatPayload*)rxPayload);
                            break;
                    }
                }
                // Reset for next frame
                rxState = RX_WAIT_SYNC;
                break;
            }
        }
    }
}

void TelemetryContext::handleStateChange(const StateChangePayload& payload) {
    FlightStateId newState = (FlightStateId)payload.newState;
    FlightStateId prevState = (FlightStateId)payload.prevState;

    currentState = newState;

    logBegin();
    logPrint(F("[STATE] "));
    logPrint(getStateName(prevState));
    logPrint(F(" -> "));
    logPrint(getStateName(newState));
    logPrint(F(" @"));
    logPrint((unsigned long)payload.timestampLow);
    logPrintln();
    logEnd();
}

void TelemetryContext::handleEvent(const EventPayload& payload) {
    EventCode event = (EventCode)payload.eventCode;

    logBegin();
    logPrint(F("[EVENT] "));
    logPrint(getEventName(event));
    logPrint(F(" | state="));
    logPrint(getStateName((FlightStateId)payload.stateId));
    logPrint(F(" | data="));
    logPrint(payload.dataValue);
    logPrint(F(" @"));
    logPrint((unsigned long)payload.timestampLow);
    logPrintln();
    logEnd();

    // Update collecting flag based on events
    if (event == EVT_LAUNCH_DETECTED) {
        collecting = true;
    } else if (event == EVT_LANDING_DETECTED) {
        stopTelemetry();
    }
}

void TelemetryContext::handleHeartbeat(const HeartbeatPayload& payload) {
    currentState = (FlightStateId)payload.currentState;
    collecting = (payload.flags & 0x01) != 0;
    failsafeArmed = (payload.flags & 0x02) != 0;
    failsafeTriggered = (payload.flags & 0x04) != 0;

    // Heartbeat received - toggle LED to show communication
    static bool ledState = false;
    ledState = !ledState;
    digitalWrite(LED2, ledState ? HIGH : LOW);
}

// ---- Event logging helpers ----

void TelemetryContext::logBegin() {
    _logFile = SD.open(logFileName, FILE_WRITE);
}

void TelemetryContext::logEnd() {
    if (_logFile) {
        _logFile.flush();
        _logFile.close();
    }
}

void TelemetryContext::logPrint(const __FlashStringHelper* msg) {
    if (_logFile) _logFile.print(msg);
}

void TelemetryContext::logPrint(const char* msg) {
    if (_logFile) _logFile.print(msg);
}

void TelemetryContext::logPrint(float val, int decimals) {
    if (_logFile) _logFile.print(val, decimals);
}

void TelemetryContext::logPrint(unsigned long val) {
    if (_logFile) _logFile.print(val);
}

void TelemetryContext::logPrint(int16_t val) {
    if (_logFile) _logFile.print(val);
}

void TelemetryContext::logPrintln(const __FlashStringHelper* msg) {
    if (_logFile) _logFile.println(msg);
}

void TelemetryContext::logPrintln(const char* msg) {
    if (_logFile) _logFile.println(msg);
}

void TelemetryContext::logPrintln() {
    if (_logFile) _logFile.println();
}
