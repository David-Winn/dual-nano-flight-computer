#pragma once
#include "flight_state.hpp"

class LandedState : public FlightState {
public:
    void enter(FlightContext& ctx) override;
    void update(FlightContext& ctx) override;
    void exit(FlightContext& ctx) override;
};
