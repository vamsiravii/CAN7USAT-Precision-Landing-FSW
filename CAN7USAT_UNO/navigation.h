#pragma once

#include <stdint.h>
#include "nav_types.h"
#include "config.h"

// ============================================================================
// NavFilter
// ----------------------------------------------------------------------------
// Horizontal: GNSS position + GNSS ground velocity (5 Hz).  Between fixes the
// position is dead-reckoned with the ground velocity, and the AIR-relative
// part of that velocity is rotated by the gyro yaw rate (turns show up
// immediately, wind does not rotate).  Gyro bias only matters over 0.2 s, so
// heading drift - the main weakness of the old integrated-gyro EKF - is gone.
//
// Heading: direction of (ground velocity - wind estimate), i.e. where the
// canopy points through the air, propagated with the gyro between fixes and
// pulled toward the GNSS value with a ~2 s time constant (filters gusts,
// removes gyro bias).
//
// Vertical: alpha-beta filter on the barometric altitude.
// ============================================================================
class NavFilter {
public:
    void reset();
    void update(const GncSensors& s, float dt, float wind_e, float wind_n);

    bool horizValid() const { return have_fix && !stale; }
    float courseDeg() const;                 // ground track [0, 360)
    float groundSpeed() const;               // [m/s]
    float headingDeg() const { return psi; } // air-relative heading estimate [0, 360)
    bool headingValid() const { return psi_valid; }

    float x, y;          // East / North of launch origin [m]
    float ve, vn;        // ground velocity [m/s]
    float alt;           // altitude above launch pad [m]
    float climb;         // vertical speed, + = up [m/s]
    float gnss_climb;    // -velD of the last GNSS fix [m/s]
    bool gnss_climb_valid;

private:
    void propagate(float tau, float yaw_rate_dps, float wind_e, float wind_n);

    uint32_t last_fix_ms;
    bool have_fix;
    bool stale;
    bool alt_init;
    float psi;
    bool psi_valid;
};

// ============================================================================
// AirDataEstimator
// ----------------------------------------------------------------------------
// While the canopy turns, GNSS ground-velocity samples lie on a circle whose
// centre is the wind vector and whose radius is the airspeed (vg = Va*u + W).
// A least-squares (Kasa) circle fit with exponential forgetting therefore
// measures wind AND true airspeed without any heading sensor.  Under the
// drogue the horizontal drift seeds the wind.  Sink rate is measured
// separately for straight and turning flight.
// ============================================================================
class AirDataEstimator {
public:
    void reset();
    void seedWindFromDrift(float ve, float vn);
    void addVelocitySample(float ve, float vn);
    void addSinkSample(float sink, float yaw_rate_dps);

    float windE, windN;      // wind the air moves TOWARD [m/s]
    float airspeed;          // horizontal airspeed [m/s]
    float sinkStraight;      // [m/s]
    float sinkTurn;          // [m/s]
    bool fitValid;           // true once the circle fit has converged
    bool windSeeded;
    float spread;            // smallest velocity-cloud variance in the fit window [m^2/s^2]

private:
    float s1, sx, sy, sxx, syy, sxy, sxz, syz, sz;
    uint16_t straightSamples;
    uint16_t turnSamples;
};
