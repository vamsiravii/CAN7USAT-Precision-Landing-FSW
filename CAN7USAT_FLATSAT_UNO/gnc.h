#pragma once

#include <stdint.h>
#include "nav_types.h"
#include "config.h"
#include "navigation.h"
#include "guidance.h"
#include "flight_sm.h"

// ============================================================================
// GncCore: one 10 Hz Guidance-Navigation-Control step.
// Hardware-free on purpose: the Arduino sketch feeds it real sensors, the
// software-in-the-loop harness (../sil) feeds it a simulated parafoil, and
// both run exactly this code.
// ============================================================================
class GncCore {
public:
    GncCore() : targetX(TARGET_DEFAULT_EAST), targetY(TARGET_DEFAULT_NORTH) {}
    void reset(uint32_t now_ms);
    void setTarget(float x, float y);
    GncOutputs step(const GncSensors& s);
    void forceState(FlightState st, uint32_t now_ms);

    // Telecommand overrides (owned by the sketch)
    bool manualStateHold;
    bool manualSteering;
    float manualSteeringValue;

    NavFilter nav;
    AirDataEstimator air;
    FlightStateMachine sm;
    PrecisionGuidance guidance;

    float steering;
    float targetX, targetY;

private:
    GuidanceInputs makeGuidanceInputs(const GncSensors& s, float dt) const;
    uint32_t last_ms;
    bool started;
    FlightState last_state;
};
