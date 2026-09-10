// Nano 2 - Control Board
// Safety-critical, real-time flight control
// Responsibilities:
//   - IMU reading (dedicated, near CG)
//   - Flight state machine
//   - PID controller / TVC servo actuation
//   - Parachute deployment
//   - Stage 2 ignition
//   - UART transmit to Nano 1 (telemetry board)

#include <Arduino.h>
#include "pins.hpp"
#include "flight_machine.hpp"

FlightMachine fm;

void setup() {
    delay(200);

    fm.ctx.initHardware();
    fm.start();
}

void loop() {
    digitalWrite(LED2, HIGH);

    fm.ctx.readSensors();
    fm.update();
    fm.ctx.checkFailsafe();

    fm.ctx.tick();

    digitalWrite(LED2, LOW);
}
