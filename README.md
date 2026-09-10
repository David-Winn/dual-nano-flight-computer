# dual-nano-flight-computer

A two-board flight computer for model rockets, built on a pair of Arduino Nanos. The design splits real-time flight control from telemetry so that a stall, SD write, or filesystem error on the logging board can never delay a deployment decision.

Written in C++ as a set of Arduino sketches, with a small Python utility for decoding flight data on the ground.

> **Scope note.** The guidance and actuation layer for this vehicle is developed in a separate private repository and is not published here. The pin map reserves `SERVO_0` / `SERVO_1` and `nano2_control.ino` references the actuation task in a comment, but no control code ships in this repo. Everything below covers flight phase detection, recovery, staging, and telemetry only.

---

## Architecture

Early versions ran everything on one Nano (preserved under `main.ino/`). That design had a structural problem: SD card writes are blocking and occasionally slow, and the telemetry loop was the same loop responsible for detecting apogee, firing the recovery charge, and thrust vectoring. 

The current design puts those responsibilities on separate microcontrollers:

| | **Nano 2 — Control** | **Nano 1 — Telemetry** |
|---|---|---|
| Role | Safety-critical, real-time | Flight recorder, best-effort |
| IMU | Dedicated, mounted near CG | Dedicated, independent reads |
| Owns | Flight state machine, recovery deployment, stage 2 ignition, timer failsafe | Telemetry buffering, SD writes, event log |
| Storage | None | microSD (`data.txt`, `log.txt`) |
| Link | UART TX → Nano 1 | UART RX ← Nano 2 |

Nano 2 never touches the SD card and never blocks on I/O. It broadcasts state transitions, discrete events, and a 10 Hz heartbeat over UART; Nano 1 listens and annotates its own recording. If Nano 1 fails outright, Nano 2 flies the mission unaffected — the flight is just unrecorded.

Both boards read their own IMU rather than sharing one over the link. Nano 1's samples are the recorded flight data; Nano 2's samples drive the state machine.

---

## Repository layout

```
main.ino/            Legacy single-board firmware (kept for reference)
nano2_control/       Control board — state machine, recovery, staging, UART TX
nano1_telemetry/     Telemetry board — IMU logging, SD writer, UART RX
parser.py            Ground-side binary → CSV decoder
```

Shared between the two boards: `uart_protocol.hpp` (frame format, message and event enums, CRC8) and `flight_config.hpp` (tuning constants). These are duplicated in each sketch directory because the Arduino IDE builds each sketch folder in isolation — **if you change one, change both.**

---

## Flight state machine

Both boards use the same state model: a `FlightState` base class with `enter` / `update` / `exit`, and a `FlightContext` holding sensor data, timers, and state pointers. `FlightMachine` wires the concrete state instances together at startup.


**Prelaunch** — Idle on the pad. Watches for sustained vertical acceleration above `LAUNCH_ACCEL_THRESHOLD`. Requires `LAUNCH_CONSECUTIVE` readings in a row so a bump on the rail doesn't trigger it. On confirmation: starts telemetry collection and arms the timer failsafe.

**Boost** — Watches for acceleration dropping below `BURNOUT_ACCEL_THRESHOLD` for `BURNOUT_CONSECUTIVE` readings. If `STAGE2_ENABLED`, first burnout fires the stage 2 igniter for a 500 ms pulse, then resets the burnout counter and waits for the second burnout. Otherwise transitions straight to coast.

**Coast** — Integrates acceleration to estimate vertical velocity. Velocity is negative while climbing and crosses zero at apogee; `APOGEE_CONSECUTIVE` readings above `APOGEE_VELOCITY_THRESHOLD` confirm the vehicle is descending.

**Apogee** — Resets the integrator to establish a clean zero reference, then dwells for `APOGEE_DWELL_MS` before handing off. The dwell keeps deployment out of the high-drag tumble right at the top.

**Descent** — Integrates from the apogee reference to track distance fallen. Fires the recovery charge after `DEPLOY_DISTANCE` metres via a 250 ms pulse on `PARACHUTE_PIN`, and disarms the timer failsafe once deployment succeeds. Then watches for `LANDING_CONSECUTIVE` stable readings (low velocity, acceleration near 1 g) to declare landing.

**Landed** — Terminal. Flushes and closes telemetry.

### Timer failsafe

Armed at launch detection. If `FAILSAFE_TIMEOUT_MS` elapses without a successful deployment, it fires the recovery charge unconditionally, independent of the state machine. This is the backstop for any sensor or integrator failure that leaves the state machine stuck — see the known issues below for why it currently matters more than it should.

---

## Altitude estimation

`DistanceAlgorithm` is a double integrator over the vertical accelerometer axis, with an exponential moving average as a pre-filter:

```
filteredAccel = ALPHA * accel + (1 - ALPHA) * filteredAccel
d += v*dt + ½*a*dt²
v += a*dt
```

There is no barometer in this design, so this is dead reckoning — it accumulates drift over the flight and the estimate is only as good as the reset points. That's why the integrator is re-zeroed on entry to Coast, Apogee, and Descent, and why deployment keys off *distance fallen since apogee* rather than absolute altitude. A relative measurement over a few seconds is far more trustworthy than an absolute one over the whole flight.

Loop rate is held at 100 Hz by `tick()`, which busy-waits the remainder of each 10 ms window.

---

## UART protocol

Nano 2 → Nano 1, one-way, 57600 baud on hardware serial (pins 0/1).

```
[0xAA] [MSG_TYPE] [PAYLOAD...] [CRC8]
```

CRC8 uses polynomial `0x07`. The receiver on Nano 1 is a four-state machine (`WAIT_SYNC` → `WAIT_TYPE` → `WAIT_PAYLOAD` → `WAIT_CRC`) that drops any frame failing CRC and resyncs on the next sync byte. Frames are never retried — the link is best-effort by design, since a retry queue on the control board would be exactly the kind of unbounded work this architecture exists to avoid.

**Message types**

| Type | Code | Payload | Contents |
|---|---|---|---|
| `MSG_STATE_CHANGE` | `0x01` | 4 B | new state, previous state, timestamp (low 16 bits) |
| `MSG_EVENT` | `0x02` | 6 B | event code, current state, timestamp, optional int16 data |
| `MSG_HEARTBEAT` | `0x03` | 4 B | current state, status flags, timestamp |

Timestamps are the low 16 bits of `millis()` — they wrap every ~65 s, which is fine because Nano 1 stamps its own recording and uses these only for correlation.

Heartbeat flags: bit 0 `collecting`, bit 1 `failsafeArmed`, bit 2 `failsafeTriggered`.

**Event codes**: launch detected, failsafe armed, motor burnout, stage 2 ignition start/end, apogee detected, parachute deploy, failsafe triggered, landing detected.

---

## Data format and ground processing

Telemetry is buffered in RAM as 100 × `TelemSample` and flushed to SD as raw binary when the buffer fills. Writing binary rather than CSV was deliberate: formatting floats to ASCII on an AVR is expensive enough to matter at 100 Hz, and the file is roughly a third the size.

```c
struct TelemSample {
    float xA, yA, zA;        // accelerometer, g
    float xG, yG, zG;        // gyroscope, dps
    unsigned long timestamp; // millis()
};                           // 28 bytes on AVR
```

Both boards auto-increment filenames (`data.txt`, `data1.txt`, …) so a previous flight is never overwritten.

Decode on the ground:

```bash
python3 parser.py
```

Set `DATA_DIR` at the top of the script to your SD card mount point. It finds the highest-numbered data file, unpacks it with `struct` format `6fI`, and writes `parsed_data.csv` alongside it. `log.txt` is plain text and readable as-is.

---

## Hardware

- 2 × Arduino Nano with onboard LSM6DS3 IMU (uses the `Arduino_LSM6DS3` library)
- microSD breakout on SPI, CS on D10 (Nano 1 only)
- 4 status LEDs per board
- 2 arming buttons (Nano 2)
- Pyro channels: recovery on D5, stage 2 on D9

Status LEDs on Nano 1: **LED0** SD card healthy, **LED1** IMU initialised, **LED2** loop heartbeat, **LED3** telemetry actively collecting. A dark LED0 or LED1 at boot means initialisation failed and the board has halted — check before arming.

### Build

Open each sketch folder in the Arduino IDE and flash separately. Dependencies: `SD` (bundled) and `Arduino_LSM6DS3`.

Wire Nano 2's TX to Nano 1's RX, and common the grounds. Note that both boards use hardware serial for the link, so the USB serial monitor conflicts with it while connected — `nano2_control/pins.hpp` has commented-out defines for moving the link to SoftwareSerial on D10/D11 if you need the console during bench testing.

---

## Configuration

All tuning lives in `flight_config.hpp`. Current values:

| Constant | Value | Meaning |
|---|---|---|
| `LAUNCH_ACCEL_THRESHOLD` | 3.5 g | Upward acceleration to detect launch |
| `LAUNCH_CONSECUTIVE` | 5 | Readings required to confirm |
| `BURNOUT_ACCEL_THRESHOLD` | −0.3 g | Acceleration below this = burnout |
| `BURNOUT_CONSECUTIVE` | 5 | Readings required to confirm |
| `APOGEE_VELOCITY_THRESHOLD` | 0.25 m/s | Descent rate confirming apogee |
| `APOGEE_DWELL_MS` | 500 ms | Delay at apogee before descent phase |
| `DEPLOY_DISTANCE` | 75 m | Distance fallen before recovery deployment |
| `FAILSAFE_TIMEOUT_MS` | 10 000 ms | Hard deadline after launch to force deployment |
| `MAX_ALLOWED_TILT_DEGREES` | 45° | Tilt limit for stage 2 (see below) |
| `STAGE2_ENABLED` | 1 | Set to 0 for single-stage flights |

`DEPLOY_DISTANCE` and `FAILSAFE_TIMEOUT_MS` must be set together against your simulated flight profile. If the failsafe fires before the vehicle has fallen 75 m, it deploys the chute at high speed and will likely shred it; if it's set too long, it can't save a failed deployment before impact.

---

## Known issues

These are open and worth understanding before flying this firmware.

**The EMA filter is disabled and zeroes the integrator.** `DistanceAlgorithm::ALPHA` is `0.0f`, which reduces the filter update to `filteredAccel = filteredAccel`. Since `reset()` sets it to zero, `filteredAccel` stays zero for the whole flight, so velocity and distance never leave zero. In its current state the apogee-velocity check and the 75 m deployment trigger will not fire, and recovery depends entirely on the timer failsafe. `ALPHA` needs a real value (start around 0.1–0.3) before this flies.

**Tilt lockout is not active.** The stage 2 tilt check is commented out in `main.ino/boost_state.cpp` and absent from `nano2_control/boost_state.cpp`, so `MAX_ALLOWED_TILT_DEGREES` is currently unused and stage 2 will light regardless of attitude at first burnout. Reinstating this is the highest-priority fix in the repo — a tilt inhibit is standard practice for staged flights and most clubs require one.

---

## Safety

This firmware commands pyrotechnic outputs. Test with igniters disconnected and LEDs or a multimeter substituted on the pyro channels, and verify every state transition on the bench before connecting anything live.

Fly under an established safety code — NAR or Tripoli in the US — at a sanctioned launch site, with the certification and waivers your motor class requires. Staged flights carry their own additional restrictions; check them against your club's rules before assuming this firmware's staging logic is legal for your setup.

Use it at your own risk.
