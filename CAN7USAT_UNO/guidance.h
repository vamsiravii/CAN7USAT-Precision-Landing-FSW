#pragma once

#include <stdint.h>
#include "nav_types.h"
#include "config.h"

// ============================================================================
// Guidance sub-modes (telemetered through the FlightState they map to):
//   HOMING   -> PRECISION_GUIDANCE  fly the ground track straight at the target
//   LOITER   -> TERMINAL_GUIDANCE   circle the target to burn excess height and
//                                   measure wind/airspeed (circle fit)
//   DOWNWIND -> TERMINAL_GUIDANCE   leave the circle downwind; turn back when the
//                                   predicted arrival height drops to the bias
//   FINAL    -> FINAL_APPROACH      fly into the wind to the target; S-turns
//                                   burn any remaining excess height
// ============================================================================
enum class GuidanceMode : uint8_t {
    HOMING = 0,
    LOITER = 1,
    DOWNWIND = 2,
    FINAL = 3
};

struct GuidanceInputs {
    float x, y;              // East / North of launch origin [m]
    float alt;               // above launch pad [m]
    float ve, vn;            // ground velocity [m/s]
    float heading_deg;       // air-relative heading estimate [deg]
    bool heading_valid;
    float yaw_rate_dps;      // + = clockwise
    bool nav_valid;          // GNSS fresh
    float wind_e, wind_n;    // wind the air moves toward [m/s]
    float airspeed;          // horizontal airspeed [m/s]
    float sink_straight;     // [m/s]
    float sink_turn;         // [m/s]
    bool wind_known;         // circle fit converged or drogue drift seeded
    bool wind_fitted;        // wind/airspeed come from the circle fit (trusted)
    float dt;
    uint32_t now_ms;
};

// ----------------------------------------------------------------------------
// Heading controller: heading error -> commanded turn rate (limited to
// TURN_RATE_CMD_MAX so turns are predictable) -> steering.  The inner
// turn-rate loop uses the gyro (P + I); the integral absorbs rigging
// asymmetry, brake-line slack and steering-authority error.
// ----------------------------------------------------------------------------
class HeadingController {
public:
    void reset();
    // forced_dir: 0 = turn the short way; +1/-1 = keep turning right/left while |error| > 90 deg
    float update(float desired_heading_deg, float heading_deg, float yaw_rate_dps,
                 float rate_ff_dps, int8_t forced_dir, float dt);
    float lastError() const { return last_err; }
    float rateIntegral() const { return integ; }

private:
    float integ;
    float last_err;
};

class PrecisionGuidance {
public:
    void reset();
    void setTarget(float x, float y);
    float update(const GuidanceInputs& in);      // returns steering [-1, +1]
    void syncToFlightState(FlightState st, const GuidanceInputs& in);

    GuidanceMode mode() const { return mode_; }
    FlightState flightState() const;
    float turnRatePlanDps() const;

    // Logged for post-flight analysis
    float desiredCourse;
    float desiredHeading;
    float predictedArrivalAlt;
    float distToTarget;

private:
    void enterFinal(const GuidanceInputs& in, int8_t turn_dir);
    void enterDownwind(const GuidanceInputs& in, float leg_course);
    float straightArrivalAlt(const GuidanceInputs& in) const;
    float predictTurnThenFinal(const GuidanceInputs& in, int8_t dir) const;

    LocalPoint target;
    GuidanceMode mode_;
    HeadingController ctrl;
    int8_t forced_dir;
    float crs_trim;          // heading offset learned from the actual ground track [deg]
    bool had_fit;            // circle-fit wind seen before (reset the trim when it first arrives)

    uint32_t last_pass_ms;
    float last_pass_alt;
    bool have_pass;

    float leg_course;
    float leg_x0, leg_y0;

    float final_axis;
    int8_t weave_sign;
};
