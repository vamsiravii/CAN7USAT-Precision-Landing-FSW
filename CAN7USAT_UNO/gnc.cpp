#include "gnc.h"
#include "nav_math.h"

void GncCore::reset(uint32_t now_ms) {
    nav.reset();
    air.reset();
    sm.reset();
    guidance.reset();
    guidance.setTarget(targetX, targetY);
    manualStateHold = false;
    manualSteering = false;
    manualSteeringValue = 0.0f;
    steering = 0.0f;
    last_ms = now_ms;
    started = false;
    last_state = FlightState::BOOT;
}

void GncCore::setTarget(float x, float y) {
    targetX = x;
    targetY = y;
    guidance.setTarget(x, y);
}

void GncCore::forceState(FlightState st, uint32_t now_ms) {
    sm.forceState(st, now_ms);
}

GuidanceInputs GncCore::makeGuidanceInputs(const GncSensors& s, float dt) const {
    GuidanceInputs in;
    in.x = nav.x;
    in.y = nav.y;
    in.alt = nav.alt;
    in.ve = nav.ve;
    in.vn = nav.vn;
    in.heading_deg = nav.headingDeg();
    in.heading_valid = nav.headingValid();
    in.yaw_rate_dps = s.gyro_valid ? s.yaw_rate_dps : 0.0f;
    in.nav_valid = nav.horizValid();
    in.wind_e = air.windE;
    in.wind_n = air.windN;
    in.airspeed = air.airspeed;
    in.sink_straight = air.sinkStraight;
    in.sink_turn = air.sinkTurn;
    in.wind_known = air.fitValid || air.windSeeded;
    in.wind_fitted = air.fitValid;
    in.dt = dt;
    in.now_ms = s.now_ms;
    return in;
}

__attribute__((noinline))
GncOutputs GncCore::step(const GncSensors& s) {
    float dt = started ? (float)(uint32_t)(s.now_ms - last_ms) * 0.001f : 0.1f;
    if (dt < 0.02f) dt = 0.02f;
    if (dt > 0.5f) dt = 0.5f;
    last_ms = s.now_ms;
    started = true;

    FlightState st = sm.getState();

    // 1. Navigation
    nav.update(s, dt, air.windE, air.windN);

    // 2. Air data: drogue drift seeds the wind; canopy flight refines wind,
    //    airspeed and sink rate.
    bool fresh_fix = s.gnss_new && s.gnss_valid;
    if (st == FlightState::DROGUE_DESCENT && fresh_fix) {
        air.seedWindFromDrift(s.gnss_ve, s.gnss_vn);
    }
    if (sm.isGuidanceState()) {
        if (fresh_fix) air.addVelocitySample(s.gnss_ve, s.gnss_vn);
        float sink = nav.gnss_climb_valid ? -nav.gnss_climb : -nav.climb;
        air.addSinkSample(sink, s.gyro_valid ? s.yaw_rate_dps : 0.0f);
    }

    // 3. Mission sequencing
    if (!manualStateHold) {
        sm.update(nav.alt, nav.climb, s.now_ms);
    } else {
        sm.tickTimers(s.now_ms);
    }
    st = sm.getState();

    // 4. Guidance & control
    if (sm.isSteeringEnabled()) {
        GuidanceInputs in = makeGuidanceInputs(s, dt);
        if (st != last_state) guidance.syncToFlightState(st, in);
        float u = guidance.update(in);
        steering = manualSteering ? manualSteeringValue : u;
        if (!manualStateHold) {
            sm.requestGuidanceState(guidance.flightState(), s.now_ms);
            st = sm.getState();
        }
    } else {
        steering = manualSteering ? manualSteeringValue : 0.0f;
    }
    last_state = st;

    GncOutputs out;
    out.steering = steering;
    out.steering_enabled = sm.isSteeringEnabled();
    out.burnwire = sm.isBurnwireActive();
    out.flywheel = sm.isFlywheelActive();
    out.buzzer = sm.isBuzzerActive();
    return out;
}
