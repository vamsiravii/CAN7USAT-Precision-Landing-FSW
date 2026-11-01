#include "navigation.h"
#include "nav_math.h"

// Alpha-beta gains for the 10 Hz barometric altitude channel.
// (Altitude noise ~0.3 m -> vertical-speed noise ~0.1 m/s, ~0.7 s settling.)
static const float kAltAlpha = 0.2f;
static const float kAltBeta  = 0.02f;

// ============================================================================
// NavFilter
// ============================================================================
void NavFilter::reset() {
    x = y = 0.0f;
    ve = vn = 0.0f;
    alt = 0.0f;
    climb = 0.0f;
    gnss_climb = 0.0f;
    gnss_climb_valid = false;
    last_fix_ms = 0;
    have_fix = false;
    stale = true;
    alt_init = false;
    psi = 0.0f;
    psi_valid = false;
}

void NavFilter::propagate(float tau, float yaw_rate_dps, float wind_e, float wind_n) {
    // Rotate only the air-relative velocity: the canopy turns, the wind does not.
    float ae = ve - wind_e;
    float an = vn - wind_n;
    float d = deg2rad(yaw_rate_dps * tau);
    float c = cosf(d);
    float s = sinf(d);
    float ve_new = (ae * c + an * s) + wind_e;     // clockwise rotation (heading convention)
    float vn_new = (an * c - ae * s) + wind_n;
    x += 0.5f * (ve + ve_new) * tau;
    y += 0.5f * (vn + vn_new) * tau;
    ve = ve_new;
    vn = vn_new;
}

__attribute__((noinline))
void NavFilter::update(const GncSensors& s, float dt, float wind_e, float wind_n) {
    // ---- vertical channel -------------------------------------------------
    if (s.baro_valid) {
        if (!alt_init) {
            alt = s.baro_alt_agl;
            climb = 0.0f;
            alt_init = true;
        } else {
            float pred = alt + climb * dt;
            float r = s.baro_alt_agl - pred;
            alt = pred + kAltAlpha * r;
            climb += (kAltBeta / dt) * r;
        }
    } else if (alt_init) {
        alt += climb * dt;
    }

    // ---- horizontal channel -----------------------------------------------
    float r = s.gyro_valid ? s.yaw_rate_dps : 0.0f;
    if (psi_valid) psi = wrap_360(psi + r * dt);
    if (s.gnss_new && s.gnss_valid) {
        x = s.gnss_x;
        y = s.gnss_y;
        ve = s.gnss_ve;
        vn = s.gnss_vn;
        gnss_climb = -s.gnss_vd;
        gnss_climb_valid = true;
        have_fix = true;
        stale = false;
        last_fix_ms = s.now_ms;
        propagate(GNSS_LATENCY_S, r, wind_e, wind_n);   // PVT is ~0.1 s old when it arrives

        float ae = ve - wind_e;
        float an = vn - wind_n;
        if (ae * ae + an * an > PSI_MIN_AIRVEL * PSI_MIN_AIRVEL) {
            float meas = wrap_360(rad2deg(atan2f(ae, an)));
            if (!psi_valid) {
                psi = meas;
                psi_valid = true;
            } else {
                psi = wrap_360(psi + PSI_GNSS_GAIN * wrap_180(meas - psi));
            }
        }
    } else if (have_fix) {
        propagate(dt, r, wind_e, wind_n);
        if ((uint32_t)(s.now_ms - last_fix_ms) > GNSS_TIMEOUT_MS) {
            stale = true;
            gnss_climb_valid = false;
        }
    }
}

float NavFilter::courseDeg() const {
    return wrap_360(rad2deg(atan2f(ve, vn)));
}

float NavFilter::groundSpeed() const {
    return sqrtf(ve * ve + vn * vn);
}


// ============================================================================
// AirDataEstimator
// ============================================================================
void AirDataEstimator::reset() {
    s1 = sx = sy = sxx = syy = sxy = sxz = syz = sz = 0.0f;
    windE = windN = 0.0f;
    airspeed = AIRSPEED_INIT;
    sinkStraight = SINK_STRAIGHT_INIT;
    sinkTurn = SINK_TURN_INIT;
    fitValid = false;
    windSeeded = false;
    spread = 0.0f;
    straightSamples = 0;
    turnSamples = 0;
}

void AirDataEstimator::seedWindFromDrift(float ve, float vn) {
    if (fitValid) return;
    if (!windSeeded) {
        windE = ve;
        windN = vn;
        windSeeded = true;
    } else {
        windE += DROGUE_WIND_EMA_ALPHA * (ve - windE);
        windN += DROGUE_WIND_EMA_ALPHA * (vn - windN);
    }
}

__attribute__((noinline))
void AirDataEstimator::addVelocitySample(float ve, float vn) {
    const float l = WIND_FIT_FORGET;
    float z = ve * ve + vn * vn;
    s1  = l * s1  + 1.0f;
    sx  = l * sx  + ve;
    sy  = l * sy  + vn;
    sxx = l * sxx + ve * ve;
    syy = l * syy + vn * vn;
    sxy = l * sxy + ve * vn;
    sxz = l * sxz + ve * z;
    syz = l * syz + vn * z;
    sz  = l * sz  + z;

    if (s1 < 20.0f) return;

    // Centred algebraic circle fit (Bullock).  u = vx - mx, v = vy - my.
    float n = s1;
    float mx = sx / n;
    float my = sy / n;
    float suu = sxx - n * mx * mx;
    float svv = syy - n * my * my;
    float suv = sxy - n * mx * my;
    float m2 = mx * mx + my * my;
    float A = sxz - 2.0f * mx * sxx - 2.0f * my * sxy - mx * sz + 2.0f * n * mx * m2;   // sum u(u^2+v^2)
    float B = syz - 2.0f * mx * sxy - 2.0f * my * syy - my * sz + 2.0f * n * my * m2;   // sum v(u^2+v^2)
    float det = suu * svv - suv * suv;
    float tr = suu + svv;
    // Smallest eigenvalue of the velocity covariance: a real turn spreads the
    // samples around a circle of radius ~airspeed; gust noise in straight
    // flight only makes a small blob, which must not be "fitted".
    float half_tr = 0.5f * tr / n;
    float dd = 0.5f * (suu - svv) / n;
    float off = suv / n;
    spread = half_tr - sqrtf(dd * dd + off * off);
    if (spread < WIND_FIT_MIN_SPREAD * AIRSPEED_INIT * AIRSPEED_INIT || det <= 0.0f) return;

    float uc = 0.5f * (A * svv - B * suv) / det;
    float vc = 0.5f * (B * suu - A * suv) / det;
    float r2 = uc * uc + vc * vc + tr / n;
    if (!(r2 > 0.0f)) return;
    float va = sqrtf(r2);
    float we = mx + uc;
    float wn = my + vc;
    if (va < WIND_FIT_MIN_AIRSPEED || va > WIND_FIT_MAX_AIRSPEED) return;
    if (we * we + wn * wn > 15.0f * 15.0f) return;

    windE = we;
    windN = wn;
    airspeed = va;
    fitValid = true;
}

void AirDataEstimator::addSinkSample(float sink, float yaw_rate_dps) {
    if (!(sink > 0.5f && sink < 8.0f)) return;    // reject deployment transients / landing
    if (fabsf(yaw_rate_dps) < SINK_TURN_RATE_SPLIT) {
        sinkStraight += SINK_EMA_ALPHA * (sink - sinkStraight);
        if (straightSamples < 60000U) straightSamples++;
    } else {
        sinkTurn += SINK_EMA_ALPHA * (sink - sinkTurn);
        if (turnSamples < 60000U) turnSamples++;
    }
    // If we have mostly been turning, scale the straight-flight value from the
    // measured turning value instead of trusting the pre-flight guess.
    if (straightSamples < 100U && turnSamples >= 100U) {
        sinkStraight = sinkTurn * (SINK_STRAIGHT_INIT / SINK_TURN_INIT);
    }
}
