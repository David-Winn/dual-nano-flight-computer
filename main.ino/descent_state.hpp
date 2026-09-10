#pragma once
#include "flight_state.hpp"
#include "distance_alg.hpp"

class DescentState : public FlightState {
public:
    void enter(FlightContext& ctx) override;
    void update(FlightContext& ctx) override;
    void exit(FlightContext& ctx) override;

private:
    float lastAlt = 0.0f;
    bool parachuteDeployed = false;
    int stableCount = 0;
    DistanceAlgorithm distanceAlg;
    unsigned long lastUpdateMicros = 0;
    unsigned long entryMs = 0;
    unsigned long deployStartTime = 0;
}; 