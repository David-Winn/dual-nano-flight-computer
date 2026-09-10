#include "distance_alg.hpp"

DistanceAlgorithm::DistanceAlgorithm() {
    reset();
}

void DistanceAlgorithm::reset() {
    velocity = 0.0f;
    distance = 0.0f;
    filteredAccel = 0.0f;
}

void DistanceAlgorithm::update(float accel, float dtSeconds) {
    
    // Exponential moving average
    filteredAccel = ALPHA * accel + (1 - ALPHA) * filteredAccel;

    // Convert acceleration to downward reference frame
    float accelDown = -filteredAccel * 9.80655f;

    float prevVelocity = velocity;
    
    // Distance 
    // d = d0 + v*dt + 0.5*a*dt²
    distance += (velocity * dtSeconds) + (0.5f * accelDown * dtSeconds * dtSeconds);

    // Velocity 
    // v = v0 + a*dt
    velocity += accelDown * dtSeconds;

    // Clamp distance to non-negative 
    if (distance < 0.0f) {
        distance = 0.0f;
    }
}

float DistanceAlgorithm::getDistance() const {
    return distance;
}

float DistanceAlgorithm::getVelocity() const {
    return velocity;
}
