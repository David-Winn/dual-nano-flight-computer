#include <Arduino.h>
#include "pins.hpp"
#include "flight_machine.hpp"
#include "distance_alg.hpp"


FlightMachine fm;
  
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("Booting...");

  fm.ctx.initHardware();  
  fm.start();             

  Serial.println("Ready.");

  fm.ctx.logBegin();
  fm.ctx.logPrintln(F("=== System Ready ==="));
  fm.ctx.logEnd();

  // Serial.println(F("=== System Ready ==="));

}

void loop() {

    digitalWrite(LED2, HIGH);

    fm.ctx.readSensors();
    fm.ctx.bufferTelemetry();
    fm.update();
    fm.ctx.checkFailsafe();

    // Flush buffer to SD when full
    if (fm.ctx.isBufferFull()) {
        fm.ctx.flushTelemetry();
    }
    
    fm.ctx.tick();

    digitalWrite(LED2, LOW);
}
