#pragma once
#include "flight_state.hpp"
#include "distance_alg.hpp"

class ApogeeState : public FlightState {
public:
    void enter(FlightContext& ctx) override;
    void update(FlightContext& ctx) override;
    void exit(FlightContext& ctx) override;

private:
    unsigned long entryMs = 0;
    DistanceAlgorithm distanceAlg;
    unsigned long lastUpdateMicros = 0;
};
