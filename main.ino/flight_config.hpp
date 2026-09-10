#pragma once

// Second State
#define STAGE2_ENABLED   1    // Set to 0 to disable second stage
constexpr float MAX_ALLOWED_TILT_DEGREES = 45; // degrees

// Prelaunch -> Boost 
constexpr float LAUNCH_ACCEL_THRESHOLD  = 3.5f;     // g's - upward accel to detect launch
constexpr int LAUNCH_CONSECUTIVE        = 5;         // Consecutive readings above threshold to trigger coast

// Boost -> Coast 
constexpr float BURNOUT_ACCEL_THRESHOLD = -0.3f;     // g's - accel drops below this = burnout
constexpr int BURNOUT_CONSECUTIVE       = 5;         // Consecutive low readings to confirm apogee

// Coast -> Apogee 
constexpr float APOGEE_VELOCITY_THRESHOLD = 0.25f;  // m/s - velocity below this = descending
constexpr int APOGEE_CONSECUTIVE        = 5;         // Consecutive readings to confirm descent

// Apogee -> Descent 
constexpr unsigned long APOGEE_DWELL_MS = 500;      // ms to wait at apogee before descent

//  Descent 
constexpr float DEPLOY_DISTANCE         = 75.0f;     // Meters fallen before deploying chute

//  Descent -> Landed 
constexpr float LANDING_VELOCITY_THRESHOLD = 1.0f;  // m/s - velocity below this
constexpr float LANDING_ACCEL_THRESHOLD = 1.05f;    // accel near 1g
constexpr int LANDING_CONSECUTIVE       = 10;        // Consecutive stable readings

//  Time Failsafe 
constexpr unsigned long FAILSAFE_TIMEOUT_MS = 10000;  // ms after launch to force parachute deploy 


