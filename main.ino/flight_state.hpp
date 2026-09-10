#pragma once
class FlightContext; // forward

class FlightState {
public:
    virtual ~FlightState() = default;
    virtual void enter(FlightContext& ctx) {}
    virtual void update(FlightContext& ctx) {}
    virtual void exit(FlightContext& ctx) {}
};
