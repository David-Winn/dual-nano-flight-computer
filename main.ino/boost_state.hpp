#pragma once
#include "flight_state.hpp"

class BoostState : public FlightState {
public:
    void enter(FlightContext& ctx) override;
    void update(FlightContext& ctx) override;
    void exit(FlightContext& ctx) override;

private:
    float startZ = 0.0f;
    int stage = 1;
    int consecutiveLowAccel = 0;
    unsigned long ignitionStartTime = 0.0f;
};
