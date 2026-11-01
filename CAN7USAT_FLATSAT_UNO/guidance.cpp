#include "guidance.h"
#include "nav_math.h"

// ============================================================================
// HeadingController
// ============================================================================
void HeadingController::reset() {
    integ = 0.0f;
    last_err = 0.0f;
}

__attribute__((noinline))
float HeadingController::update(float desired_heading_deg, float heading_deg, float yaw_rate_dps,
                                float rate_ff_dps, int8_t forced_dir, float dt) {
    float err = wrap_180(desired_heading_deg - heading_deg);
    if (forced_dir != 0 && fabsf(err) > 90.0f) {
        err = (forced_dir > 0) ? fabsf(err) : -fabsf(err);
    }
    last_err = err;

    float r_cmd = clampf(HDG_K * err + rate_ff_dps, -TURN_RATE_CMD_MAX, TURN_RATE_CMD_MAX);
    float r_err = r_cmd - yaw_rate_dps;
    integ = clampf(integ + RATE_KI * r_err * dt / TURN_AUTH_NOM, -RATE_I_LIMIT, RATE_I_LIMIT);
    float u = (r_cmd + RATE_K * r_err) / TURN_AUTH_NOM + integ;
    return clampf(u, -1.0f, 1.0f);
}

// ============================================================================
// PrecisionGuidance
// ============================================================================
void PrecisionGuidance::reset() {
    target = LocalPoint(TARGET_DEFAULT_EAST, TARGET_DEFAULT_NORTH);
    mode_ = GuidanceMode::HOMING;
    ctrl.reset();
    forced_dir = 0;
    crs_trim = 0.0f;
    had_fit = false;
    last_pass_ms = 0;
    last_pass_alt = 0.0f;
    have_pass = false;
    leg_course = 0.0f;
    leg_x0 = leg_y0 = 0.0f;
    final_axis = 0.0f;
    weave_sign = 1;
    desiredCourse = 0.0f;
    desiredHeading = 0.0f;
    predictedArrivalAlt = 0.0f;
    distToTarget = 0.0f;
}

void PrecisionGuidance::setTarget(float x, float y) {
    target = LocalPoint(x, y);
}

FlightState PrecisionGuidance::flightState() const {
    switch (mode_) {
        case GuidanceMode::HOMING:   return FlightState::PRECISION_GUIDANCE;
        case GuidanceMode::FINAL:    return FlightState::FINAL_APPROACH;
        default:                     return FlightState::TERMINAL_GUIDANCE;
    }
}

float PrecisionGuidance::turnRatePlanDps() const {
    // The rate loop holds TURN_RATE_CMD_MAX in long turns; 0.9 covers roll-in/out lag.
    return 0.9f * TURN_RATE_CMD_MAX;
}

void PrecisionGuidance::syncToFlightState(FlightState st, const GuidanceInputs& in) {
    if (st == flightState()) return;
    if (st == FlightState::PRECISION_GUIDANCE) {
        mode_ = GuidanceMode::HOMING;
    } else if (st == FlightState::TERMINAL_GUIDANCE) {
        mode_ = GuidanceMode::LOITER;
        have_pass = false;
    } else if (st == FlightState::FINAL_APPROACH) {
        enterFinal(in, 0);
    }
}

void PrecisionGuidance::enterFinal(const GuidanceInputs& in, int8_t turn_dir) {
    mode_ = GuidanceMode::FINAL;
    forced_dir = turn_dir;
    final_axis = bearing(LocalPoint(in.x, in.y), target);
    float a = deg2rad(final_axis);
    float xt = (in.x - target.x) * cosf(a) - (in.y - target.y) * sinf(a);
    weave_sign = (xt > 0.0f) ? -1 : 1;
}

void PrecisionGuidance::enterDownwind(const GuidanceInputs& in, float course) {
    mode_ = GuidanceMode::DOWNWIND;
    leg_course = course;
    leg_x0 = in.x;
    leg_y0 = in.y;
    forced_dir = 0;
}

// Height left over at the target if we fly straight there from here.
float PrecisionGuidance::straightArrivalAlt(const GuidanceInputs& in) const {
    LocalPoint p(in.x, in.y);
    float d = distance_xy(p, target);
    float vg = ground_speed_on_course(bearing(p, target), in.airspeed, in.wind_e, in.wind_n);
    return in.alt - in.sink_straight * d / vg;
}

// Height left over at the target if we start a turn in direction `dir` now
// (at the planned turn rate), roll out pointing at the target, then fly straight.
__attribute__((noinline))
float PrecisionGuidance::predictTurnThenFinal(const GuidanceInputs& in, int8_t dir) const {
    const float two_pi = 2.0f * (float)M_PI;
    float va = in.airspeed;
    float w = deg2rad(turnRatePlanDps());
    float rt = va / w;
    float psi0 = in.heading_valid ? deg2rad(in.heading_deg)
                                  : atan2f(in.ve - in.wind_e, in.vn - in.wind_n);
    float s = (float)dir;
    float turn = (float)M_PI;
    float ex = in.x, ey = in.y, tt = 0.0f;
    for (uint8_t k = 0; k < 3; k++) {
        float psi1 = psi0 + s * turn;
        tt = turn / w;
        ex = in.x + s * rt * (cosf(psi0) - cosf(psi1)) + in.wind_e * tt;
        ey = in.y - s * rt * (sinf(psi0) - sinf(psi1)) + in.wind_n * tt;
        if (k == 2) break;
        float chi_f = bearing(LocalPoint(ex, ey), target);
        float psi_f = deg2rad(calculate_wind_corrected_heading(chi_f, va, in.wind_e, in.wind_n).heading_deg);
        float d = s * (psi_f - psi0);
        while (d < 0.0f) d += two_pi;
        while (d >= two_pi) d -= two_pi;
        turn = d;
    }
    LocalPoint e(ex, ey);
    float df = distance_xy(e, target);
    float vg = ground_speed_on_course(bearing(e, target), va, in.wind_e, in.wind_n);
    return in.alt - in.sink_turn * tt - in.sink_straight * df / vg;
}

__attribute__((noinline))
float PrecisionGuidance::update(const GuidanceInputs& in_raw) {
    // An un-fitted (drogue drift) wind can be badly off; never let it exceed
    // the airspeed, or the wind triangle would point us the wrong way.
    GuidanceInputs in = in_raw;
    if (in.wind_fitted && !had_fit) {
        crs_trim = 0.0f;     // the trim was compensating the rough drogue-drift wind
        had_fit = true;
    }
    if (!in.wind_fitted) {
        float w = sqrtf(in.wind_e * in.wind_e + in.wind_n * in.wind_n);
        float wmax = SEED_WIND_MAX_FRAC * in.airspeed;
        if (w > wmax) {
            in.wind_e *= wmax / w;
            in.wind_n *= wmax / w;
        }
    }
    LocalPoint p(in.x, in.y);
    distToTarget = distance_xy(p, target);
    float gs = sqrtf(in.ve * in.ve + in.vn * in.vn);
    float crs = wrap_360(rad2deg(atan2f(in.ve, in.vn)));
    const int8_t dir = (LOITER_DIRECTION >= 0) ? 1 : -1;

    if (!in.nav_valid) {
        // GNSS lost: gentle constant turn keeps us near the area until it returns.
        ctrl.reset();
        return NAV_LOST_STEERING * (float)dir;
    }

    float wind = sqrtf(in.wind_e * in.wind_e + in.wind_n * in.wind_n);
    bool calm = (!in.wind_known) || (wind < CALM_WIND_SPEED);
    float downwind = calm ? crs : wrap_360(rad2deg(atan2f(in.wind_e, in.wind_n)));
    // "Cannot fly upwind" is only believed from a fitted wind.
    bool strong_wind = in.wind_fitted && (in.airspeed - wind) < 1.0f;

    // ---------------- mode transitions ----------------
    switch (mode_) {
        case GuidanceMode::HOMING: {
            predictedArrivalAlt = straightArrivalAlt(in);
            if (distToTarget <= LOITER_RADIUS + LOITER_ENTRY_MARGIN) {
                if (!strong_wind && in.alt > LOITER_FLOOR_ALT &&
                    predictedArrivalAlt > ARRIVAL_ALT_BIAS + LOITER_MIN_EXCESS_ALT) {
                    mode_ = GuidanceMode::LOITER;
                    have_pass = false;
                } else {
                    enterFinal(in, 0);
                }
            } else if (predictedArrivalAlt <= ARRIVAL_ALT_BIAS + 3.0f) {
                enterFinal(in, 0);
            }
            break;
        }
        case GuidanceMode::LOITER: {
            if (in.alt < LOITER_FLOOR_ALT || strong_wind) {
                enterFinal(in, dir);
                break;
            }
            bool aligned = calm || fabsf(wrap_180(crs - downwind)) < DOWNWIND_ALIGN_DEG;
            bool new_pass = calm || !have_pass || (uint32_t)(in.now_ms - last_pass_ms) > 10000UL;
            if (aligned && new_pass) {
                float va = in.airspeed;
                float wr = wind / va;
                float dh = in.sink_turn * (2.0f * (float)M_PI * LOITER_RADIUS / va) * (1.0f + 0.5f * wr * wr);
                if (!calm) {
                    if (have_pass && (last_pass_alt - in.alt) > 5.0f) dh = last_pass_alt - in.alt;
                    last_pass_alt = in.alt;
                    last_pass_ms = in.now_ms;
                    have_pass = true;
                }
                predictedArrivalAlt = predictTurnThenFinal(in, dir);
                if (predictedArrivalAlt - 1.1f * dh < ARRIVAL_ALT_BIAS) {
                    enterDownwind(in, downwind);
                }
            }
            break;
        }
        case GuidanceMode::DOWNWIND: {
            predictedArrivalAlt = predictTurnThenFinal(in, dir);
            if (predictedArrivalAlt <= ARRIVAL_ALT_BIAS || in.alt < 0.8f * LOITER_FLOOR_ALT) {
                enterFinal(in, dir);
            }
            break;
        }
        case GuidanceMode::FINAL:
        default:
            // Entered FINAL with lots of height to spare (e.g. by telecommand or
            // after a GNSS outage): go back to burning it off in the loiter.
            if (!strong_wind && forced_dir == 0 &&
                distToTarget <= LOITER_RADIUS + LOITER_ENTRY_MARGIN &&
                in.alt > LOITER_FLOOR_ALT + 20.0f &&
                straightArrivalAlt(in) > ARRIVAL_ALT_BIAS + LOITER_MIN_EXCESS_ALT + 20.0f) {
                mode_ = GuidanceMode::LOITER;
                have_pass = false;
            }
            break;
    }

    // ---------------- desired course for the active mode ----------------
    float des = crs;
    float rate_ff = 0.0f;
    switch (mode_) {
        case GuidanceMode::HOMING:
            des = bearing(p, target);
            break;

        case GuidanceMode::LOITER: {
            float d = distToTarget;
            float th = (d > 1.0f) ? bearing(target, p) : wrap_360(crs - 90.0f * dir);
            float delta = rad2deg(atanf((d - LOITER_RADIUS) / LOITER_LOOKAHEAD));
            des = wrap_360(th + 90.0f * dir + dir * delta);
            if (fabsf(d - LOITER_RADIUS) < 15.0f) rate_ff = dir * rad2deg(gs / LOITER_RADIUS);
            break;
        }

        case GuidanceMode::DOWNWIND: {
            float a = deg2rad(leg_course);
            float xt = (in.x - leg_x0) * cosf(a) - (in.y - leg_y0) * sinf(a);
            des = wrap_360(leg_course - rad2deg(atanf(xt / LINE_LOOKAHEAD)));
            break;
        }

        case GuidanceMode::FINAL:
        default: {
            des = bearing(p, target);
            float vg = ground_speed_on_course(des, in.airspeed, in.wind_e, in.wind_n);
            float h_need = in.sink_straight * distToTarget / vg;
            predictedArrivalAlt = in.alt - h_need;
            if (forced_dir != 0 && fabsf(wrap_180(des - crs)) < 90.0f) forced_dir = 0;
            float h_avail = in.alt - 0.5f;
            if (forced_dir == 0 && distToTarget > FINAL_WEAVE_MIN_DIST && vg > 1.5f && h_avail > h_need) {
                // S-turns: fly at +/-phi off the target bearing so that the closing
                // speed (~vg*cos(phi)) matches the height we have left.
                float ratio = h_need / h_avail;
                float cmin = cosf(deg2rad(FINAL_WEAVE_MAX_DEG));
                if (ratio < cmin) ratio = cmin;
                float phi = rad2deg(acosf(ratio));
                if (phi > 3.0f) {
                    float a = deg2rad(final_axis);
                    float xt = (in.x - target.x) * cosf(a) - (in.y - target.y) * sinf(a);
                    if (weave_sign > 0 && xt > FINAL_WEAVE_XTRACK) weave_sign = -1;
                    else if (weave_sign < 0 && xt < -FINAL_WEAVE_XTRACK) weave_sign = 1;
                    des = wrap_360(des + weave_sign * phi);
                }
            }
            break;
        }
    }
    desiredCourse = des;

    // Steer the air-relative heading: desired heading = wind-triangle crab for
    // the desired ground course.  Falls back to the GNSS course if the heading
    // estimate is not available yet.
    float meas = crs;
    float cmd = des;
    if (in.heading_valid) {
        meas = in.heading_deg;
        cmd = calculate_wind_corrected_heading(des, in.airspeed, in.wind_e, in.wind_n).heading_deg;
        // Slow outer loop on the real ground track: if the wind estimate is
        // wrong the course drifts off; integrate that error into a heading trim.
        // Frozen in the loiter (it has its own position feedback) and in big turns.
        float ce = wrap_180(des - crs);
        if (mode_ != GuidanceMode::LOITER && forced_dir == 0 && gs > CRS_TRIM_MIN_GS && fabsf(ce) < 90.0f) {
            crs_trim = clampf(crs_trim + CRS_TRIM_KI * clampf(ce, -45.0f, 45.0f) * in.dt,
                              -CRS_TRIM_MAX_DEG, CRS_TRIM_MAX_DEG);
        }
        cmd = wrap_360(cmd + crs_trim);
    }
    desiredHeading = cmd;
    return ctrl.update(cmd, meas, in.yaw_rate_dps, rate_ff, forced_dir, in.dt);
}
