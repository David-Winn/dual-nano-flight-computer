// Nano 1 - Telemetry Board
// Flight recorder, best-effort logging
// Responsibilities:
//   - IMU reading (dedicated, independent sensor reads)
//   - Telemetry buffer (28-byte TelemSample structs)
//   - SD card writer (binary flush, state-gated)
//   - Event log (state annotations, timestamps)
//   - UART receive from Nano 2 (control board)
// Outputs: data.txt, log.txt

#include <Arduino.h>
#include "pins.hpp"
#include "telemetry_context.hpp"

TelemetryContext ctx;

void setup() {
    delay(200);

    ctx.initHardware();
}

void loop() {
    // Process incoming UART messages from Nano 2
    ctx.processUartRx();

    // Read local IMU
    ctx.readSensors();

    // Buffer telemetry when collecting
    ctx.bufferTelemetry();

    // Flush buffer to SD when full
    if (ctx.isBufferFull()) {
        ctx.flushTelemetry();
    }

    ctx.tick();
}
