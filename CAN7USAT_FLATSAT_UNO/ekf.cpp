#include "ekf.h"
#include "nav_math.h"
#include "config.h"

ExtendedKalmanFilter::ExtendedKalmanFilter()
    : current_va(FORWARD_AIRSPEED),
      current_vz(PARAFOIL_DESCENT_RATE),
      wind_x(0.0f),
      wind_y(0.0f) {}

__attribute__((noinline))
void ExtendedKalmanFilter::reset(
    float init_x,
    float init_y,
    float init_alt,
    float init_heading_deg
) {
    x[0] = init_x;
    x[1] = init_y;
    x[2] = init_alt;
    x[3] = deg2rad(wrap_360(init_heading_deg));
    x[4] = 0.0f;

    P[0] = 25.0f;
    P[1] = 25.0f;
    P[2] = 9.0f;
    P[3] = 0.03046f; // (10 deg)^2 in rad^2
    P[4] = 0.00761f; // (5 deg/s)^2 in rad^2/s^2

    wind_x = 0.0f;
    wind_y = 0.0f;
}

void ExtendedKalmanFilter::setKinematics(float forward_speed, float descent_rate) {
    if (forward_speed >= 0.0f) current_va = forward_speed;
    if (descent_rate >= 0.0f) current_vz = descent_rate;
}

void ExtendedKalmanFilter::setWindEstimate(float wx, float wy) {
    wind_x = wx;
    wind_y = wy;
}

static bool scalarUpdate1D(float &state_val, float &var_val, float innov, float R_var, float nis_max) {
    float S = var_val + R_var;
    if (S < 1e-6f) return false;
    if ((innov * innov) > (nis_max * S)) return false;
    float K = var_val / S;
    state_val += K * innov;
    var_val = (1.0f - K) * var_val;
    if (var_val < 1e-6f) var_val = 1e-6f;
    return true;
}

__attribute__((noinline))
void ExtendedKalmanFilter::predict(float dt, float steering_cmd) {
    if (dt <= 0.0f) return;

    float u = (steering_cmd < -1.0f) ? -1.0f : ((steering_cmd > 1.0f) ? 1.0f : steering_cmd);
    float cmd_rate = u * deg2rad(MAX_TURN_RATE_DEG_S);

    float psi = x[3];
    float w   = x[4];

    x[0] += (current_va * sinf(psi) + wind_x) * dt;
    x[1] += (current_va * cosf(psi) + wind_y) * dt;
    x[2] -= current_vz * dt;
    if (x[2] < 0.0f) x[2] = 0.0f;
    x[3] = wrap_pi(psi + w * dt);
    x[4] = w + ((cmd_rate - w) / 0.1f) * dt;

    float dt2 = dt * dt;
    P[0] += (0.04f + current_va * current_va * P[3]) * dt2;
    P[1] += (0.04f + current_va * current_va * P[3]) * dt2;
    P[2] += 0.01f * dt2;
    P[3] += (P[4] + 0.0025f) * dt2;
    P[4] = 0.25f * dt2;
}

bool ExtendedKalmanFilter::correctBaro(float meas_alt) {
    if (meas_alt < -50.0f || meas_alt > 5000.0f) return false;
    return scalarUpdate1D(x[2], P[2], meas_alt - x[2], 2.25f, 9.0f);
}

bool ExtendedKalmanFilter::correctHeading(float meas_heading_deg) {
    float meas_rad = deg2rad(wrap_360(meas_heading_deg));
    float innov = wrap_pi(meas_rad - x[3]);
    bool ok = scalarUpdate1D(x[3], P[3], innov, 0.00274f, 9.0f);
    x[3] = wrap_pi(x[3]);
    return ok;
}

bool ExtendedKalmanFilter::correctGPS(
    float meas_x,
    float meas_y,
    float /*meas_vx*/,
    float /*meas_vy*/,
    bool /*has_vel*/
) {
    bool okX = scalarUpdate1D(x[0], P[0], meas_x - x[0], 6.25f, 16.0f);
    bool okY = scalarUpdate1D(x[1], P[1], meas_y - x[1], 6.25f, 16.0f);
    return okX && okY;
}

float ExtendedKalmanFilter::getHeadingDeg() const {
    return wrap_360(rad2deg(x[3]));
}
