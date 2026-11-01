// ============================================================================
// CAN-7USAT software-in-the-loop (SIL) truth model
// ----------------------------------------------------------------------------
// Independent of the flight code: a 4-DOF parafoil/payload model (position,
// altitude, heading, turn rate) with servo slew, brake-line slack, first-order
// turn response, rigging asymmetry, brake-dependent speed/sink, bank-angle
// sink increase, payload yaw/pendulum oscillation, a drogue phase, burnwire
// cut + inflation, wind with shear/veer/gusts, and sensor models (baro drift,
// gyro bias/scale, GNSS Gauss-Markov error, latency, rate).
//
// It is NOT a validated aerodynamic model.  Its parameter ranges come from the
// first-order physics budget (canopy 0.86 m^2, 1.08 kg, drogue CdS ~0.075).
// Use flight/drop-test data to narrow them.
// ============================================================================
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <random>
#include <vector>

namespace sil {

constexpr double kG = 9.81;
constexpr double kD2R = M_PI / 180.0;
constexpr double kR2D = 180.0 / M_PI;

inline double wrap360(double d) { d = std::fmod(d, 360.0); return d < 0 ? d + 360.0 : d; }
inline double wrap180(double d) { return wrap360(d + 180.0) - 180.0; }

struct Rng {
    std::mt19937_64 eng;
    explicit Rng(uint64_t seed) : eng(seed) {}
    double uni(double a, double b) { return std::uniform_real_distribution<double>(a, b)(eng); }
    double nrm(double m, double s) { return std::normal_distribution<double>(m, s)(eng); }
};

struct GaussMarkov {
    double x = 0.0, sigma = 0.0, tau = 1.0;
    void init(double s, double t, Rng& r) { sigma = s; tau = t; x = r.nrm(0.0, s); }
    void step(double dt, Rng& r) {
        double a = std::exp(-dt / tau);
        x = a * x + sigma * std::sqrt(1.0 - a * a) * r.nrm(0.0, 1.0);
    }
};

// ----------------------------------------------------------------------------
// One randomised flight.
// ----------------------------------------------------------------------------
struct Scenario {
    // Environment
    double wind_ref = 3.0;        // wind speed at 600 m AGL [m/s]
    double wind_dir = 90.0;       // direction the air moves TOWARD at 600 m [deg]
    double shear_alpha = 0.2;     // power-law exponent
    double veer = 0.0;            // direction change from 600 m to the ground [deg]
    double gust_sigma = 0.6;      // horizontal gust 1-sigma [m/s]
    double gust_tau = 6.0;        // [s]
    double vgust_sigma = 0.2;     // vertical gust 1-sigma [m/s]
    // Vehicle
    double va = 6.0;              // horizontal airspeed, brakes off [m/s]
    double sink = 2.8;            // straight-flight sink, brakes off [m/s]
    double turn_auth = 35.0;      // steady turn rate at full one-side brake [deg/s]
    double turn_tau = 0.9;        // turn-rate response time constant [s]
    double trim_bias = 0.0;       // rigging asymmetry turn rate [deg/s]
    double slack = 0.05;          // fraction of servo travel that only takes up line slack
    double servo_full_s = 0.35;   // time for full brake travel [s]
    double drogue_rate = 15.8;    // [m/s]
    double burn_time = 1.5;       // burnwire ON time needed to part the line [s]
    double inflate_time = 2.5;    // [s]
    double yaw_osc_amp = 2.0;     // payload twist oscillation amplitude [deg]
    double yaw_osc_hz = 0.6;
    double drogue_spin = 60.0;    // can spin rate under the drogue [deg/s]
    double twist_unwind = 0.0;    // can rotation while the canopy opens [deg]
    // Mission
    double release_alt = 1000.0;  // AGL [m]
    double release_x = 0.0, release_y = 0.0;
    double target_x = 0.0, target_y = 0.0;          // TRUE target
    double target_survey_err_x = 0.0, target_survey_err_y = 0.0;   // uploaded - true
    double pad_wait_s = 60.0;
    double climb_rate = 4.0;      // drone climb [m/s]
    double hover_s = 20.0;
    // Sensors
    double baro_drift_end = 0.0;  // baro error accumulated by the end of the flight [m]
    double baro_noise = 0.25;     // [m]
    double gyro_bias = 0.3;       // [deg/s]
    double gyro_scale = 0.0;      // fractional
    double gyro_noise = 0.3;      // [deg/s]
    double gnss_gm_sigma = 1.2;   // slowly varying position error per axis [m]
    double gnss_gm_tau = 60.0;    // [s]
    double gnss_white = 0.3;      // [m]
    double gnss_vel_noise = 0.08; // [m/s]
    double gnss_latency = 0.10;   // [s]
    double gnss_rate_hz = 5.0;
};

struct ScenarioRanges {
    double wind_min = 0.0, wind_max = 6.0;
    double turn_auth_min = 25.0, turn_auth_max = 50.0;   // with a proper brake drum
    double slack_max = 0.15;
    double gnss_rate_hz = 5.0;
    double release_radius = 100.0;    // drone release point within this of the target [m]
    double gyro_bias_sigma = 0.7;     // stale EEPROM calibration + temperature
    double burn_sigma = -1.0;         // >0: burn time ~N(1.5 s, sigma) i.e. measured on the bench
};

inline Scenario randomScenario(Rng& r, const ScenarioRanges& g) {
    Scenario s;
    s.wind_ref = r.uni(g.wind_min, g.wind_max);
    s.wind_dir = r.uni(0.0, 360.0);
    s.shear_alpha = r.uni(0.10, 0.30);
    s.veer = r.uni(-30.0, 30.0);
    s.gust_sigma = 0.2 + r.uni(0.15, 0.25) * s.wind_ref;
    s.gust_tau = r.uni(3.0, 10.0);
    s.vgust_sigma = r.uni(0.1, 0.3);
    s.va = std::max(5.0, std::min(7.2, r.nrm(6.0, 0.4)));
    s.sink = std::max(2.2, std::min(3.6, r.nrm(2.8, 0.3)));
    s.turn_auth = r.uni(g.turn_auth_min, g.turn_auth_max);
    s.turn_tau = r.uni(0.6, 1.2);
    s.trim_bias = r.nrm(0.0, 3.0);
    s.slack = r.uni(0.0, g.slack_max);
    s.drogue_rate = r.nrm(15.8, 1.2);
    s.burn_time = (g.burn_sigma > 0.0) ? std::max(0.3, r.nrm(1.5, g.burn_sigma)) : r.uni(0.8, 2.5);
    s.inflate_time = r.uni(1.5, 3.5);
    s.yaw_osc_amp = r.uni(1.0, 4.0);
    s.yaw_osc_hz = r.uni(0.4, 0.8);
    s.drogue_spin = r.nrm(0.0, 90.0);
    s.twist_unwind = r.nrm(0.0, 360.0);
    s.release_alt = r.uni(950.0, 1050.0);
    double rr = g.release_radius * std::sqrt(r.uni(0.0, 1.0));
    double ra = r.uni(0.0, 2.0 * M_PI);
    s.target_x = r.uni(-30.0, 30.0);
    s.target_y = r.uni(-30.0, 30.0);
    s.release_x = s.target_x + rr * std::sin(ra);
    s.release_y = s.target_y + rr * std::cos(ra);
    s.target_survey_err_x = r.nrm(0.0, 1.0);
    s.target_survey_err_y = r.nrm(0.0, 1.0);
    s.pad_wait_s = r.uni(30.0, 300.0);
    s.climb_rate = r.uni(3.0, 5.0);
    s.hover_s = r.uni(10.0, 60.0);
    s.baro_drift_end = r.nrm(0.0, 1.5);
    s.gyro_bias = r.nrm(0.0, g.gyro_bias_sigma);
    s.gyro_scale = r.nrm(0.0, 0.02);
    s.gnss_rate_hz = g.gnss_rate_hz;
    return s;
}

// ----------------------------------------------------------------------------
// Sensor sample delivered to the flight code at each 10 Hz tick.
// ----------------------------------------------------------------------------
struct SensorTick {
    double t;
    double baro_alt;      // raw pressure altitude AGL (sketch applies its EMA)
    double gyro_rate;     // mean measured yaw rate over the tick [deg/s], + clockwise
    double gyro_angle;    // integrated measured yaw (for the old firmware) [deg]
    bool gnss_new;
    double gx, gy;        // measured position [m]
    double gve, gvn, gvd; // measured velocity [m/s]
};

class World {
public:
    enum Phase { PAD, ASCENT, HOVER, DROGUE, INFLATING, CANOPY, LANDED };

    World(const Scenario& sc, uint64_t seed) : s(sc), rng(seed) {
        x = s.release_x; y = s.release_y; h = 0.0;
        gust_e.init(s.gust_sigma, s.gust_tau, rng);
        gust_n.init(s.gust_sigma, s.gust_tau, rng);
        gust_v.init(s.vgust_sigma, 3.0, rng);
        gnss_ex.init(s.gnss_gm_sigma, s.gnss_gm_tau, rng);
        gnss_ey.init(s.gnss_gm_sigma, s.gnss_gm_tau, rng);
        canopy_psi = rng.uni(0.0, 360.0);
        can_yaw = rng.uni(0.0, 360.0);
        drone_yaw_rate = 0.0;
        osc_phase = rng.uni(0.0, 2.0 * M_PI);
        double total_guess = s.pad_wait_s + s.release_alt / s.climb_rate + s.hover_s + 400.0;
        baro_drift_rate = s.baro_drift_end / total_guess;
        next_gnss_t = rng.uni(0.0, 1.0 / s.gnss_rate_hz);
    }

    // ---- wind field -------------------------------------------------------
    void wind(double alt, double& we, double& wn) const {
        double hh = std::max(alt, 3.0);
        double spd = s.wind_ref * std::pow(hh / 600.0, s.shear_alpha);
        double frac = std::min(std::max(alt, 0.0), 600.0) / 600.0;
        double dir = (s.wind_dir + s.veer * (1.0 - frac)) * kD2R;
        we = spd * std::sin(dir) + gust_e.x;
        wn = spd * std::cos(dir) + gust_n.x;
    }

    // ---- one physics step ---------------------------------------------------
    // steering: [-1, 1] (+ = right brake); burnwire: MOSFET state.
    void step(double dt, double steering, bool steering_enabled, bool burnwire) {
        if (phase == LANDED) return;
        t += dt;
        gust_e.step(dt, rng); gust_n.step(dt, rng); gust_v.step(dt, rng);
        gnss_ex.step(dt, rng); gnss_ey.step(dt, rng);

        // servo slew toward commanded brake positions
        double cmdR = steering_enabled ? std::max(0.0, steering) : 0.0;
        double cmdL = steering_enabled ? std::max(0.0, -steering) : 0.0;
        if (servo_override) { cmdR = servo_cmd_r; cmdL = servo_cmd_l; }
        double maxd = dt / s.servo_full_s;
        brakeR += std::max(-maxd, std::min(maxd, cmdR - brakeR));
        brakeL += std::max(-maxd, std::min(maxd, cmdL - brakeL));

        double we, wn;
        wind(h, we, wn);
        double prev_can_yaw = can_yaw;

        switch (phase) {
            case PAD:
                if (t >= s.pad_wait_s) phase = ASCENT;
                break;
            case ASCENT:
                h += s.climb_rate * dt;
                if (std::fmod(t, 20.0) < dt) drone_yaw_rate = rng.nrm(0.0, 4.0);
                can_yaw += drone_yaw_rate * dt;
                if (h >= s.release_alt) { h = s.release_alt; phase = HOVER; hover_end = t + s.hover_s; }
                break;
            case HOVER:
                can_yaw += drone_yaw_rate * dt;
                if (t >= hover_end) { phase = DROGUE; vz = 0.0; }
                break;
            case DROGUE: {
                double vd = -vz;   // down speed
                vd += kG * (1.0 - (vd / s.drogue_rate) * std::fabs(vd / s.drogue_rate)) * dt;
                vz = -vd;
                h += vz * dt;
                double a = dt / 1.0;
                vx += a * (we - vx);
                vy += a * (wn - vy);
                x += vx * dt; y += vy * dt;
                can_yaw += s.drogue_spin * dt;
                if (burnwire) burn_accum += dt;
                if (burn_accum >= s.burn_time && !cut) {
                    cut = true; cut_alt = h; cut_t = t;
                    phase = INFLATING; infl_t0 = t;
                    // the can ends up aligned with the canopy (mod full line twists)
                    infl_yaw0 = can_yaw;
                    double k = std::round((can_yaw - canopy_psi + s.twist_unwind) / 360.0);
                    infl_yaw1 = canopy_psi + 360.0 * k;
                }
                break;
            }
            case INFLATING: {
                double f = std::min(1.0, (t - infl_t0) / s.inflate_time);
                double vd = -vz;
                double tau = s.inflate_time / 2.5;
                vd += (s.sink - vd) * (dt / tau);
                vz = -vd;
                h += vz * dt;
                double va_now = s.va * f;
                vx = va_now * std::sin(canopy_psi * kD2R) + we;
                vy = va_now * std::cos(canopy_psi * kD2R) + wn;
                x += vx * dt; y += vy * dt;
                can_yaw = infl_yaw0 + (infl_yaw1 - infl_yaw0) * f;
                if (f >= 1.0) { phase = CANOPY; canopy_t0 = t; }
                break;
            }
            case CANOPY: {
                auto eff = [&](double b) { return std::max(0.0, b - s.slack) / (1.0 - s.slack); };
                double eR = eff(brakeR), eL = eff(brakeL);
                double da = eR - eL;
                double ds = std::min(eR, eL);
                double r_ss = s.turn_auth * da + s.trim_bias;
                double lim = s.turn_auth * 1.3;
                r_ss = std::max(-lim, std::min(lim, r_ss));
                turn_rate += (r_ss - turn_rate) * (dt / s.turn_tau);
                canopy_psi = wrap360(canopy_psi + turn_rate * dt);
                double va_now = s.va * (1.0 - 0.08 * std::fabs(da) - 0.20 * ds);
                double phi = std::atan(va_now * turn_rate * kD2R / kG);
                double n = 1.0 / std::cos(phi);
                double sink_now = s.sink * (1.0 + 0.10 * std::fabs(da) + 0.35 * ds) * std::pow(n, 1.5);
                vz = -sink_now + gust_v.x;
                h += vz * dt;
                double swing = 0.25 * std::sin(2.0 * M_PI * 0.42 * t + osc_phase);
                vx = (va_now + swing) * std::sin(canopy_psi * kD2R) + we;
                vy = (va_now + swing) * std::cos(canopy_psi * kD2R) + wn;
                x += vx * dt; y += vy * dt;
                // can yaw = canopy heading + lightly damped twist oscillation
                double ramp = std::min(1.0, (t - canopy_t0) / 2.0);
                double osc = ramp * s.yaw_osc_amp * std::sin(2.0 * M_PI * s.yaw_osc_hz * t + osc_phase);
                can_yaw = canopy_psi + osc;
                sink_sum += sink_now; sink_n += 1;
                max_rate = std::max(max_rate, std::fabs(turn_rate));
                break;
            }
            case LANDED:
                break;
        }

        // integrate the measured gyro (yaw rate of the CAN, not the canopy)
        double true_rate = wrap180(can_yaw - prev_can_yaw) / dt;
        if (phase == DROGUE && std::fabs(true_rate) > 500.0) true_rate = 500.0 * (true_rate > 0 ? 1 : -1);
        double meas = true_rate * (1.0 + s.gyro_scale) + s.gyro_bias + rng.nrm(0.0, s.gyro_noise);
        gyro_angle_acc += meas * dt;
        gyro_tick_acc += meas * dt;

        if (phase >= DROGUE && h <= 0.0) {
            phase = LANDED;
            land_x = x; land_y = y; land_t = t;
        }

        // store history for GNSS latency
        hist.push_back({t, x, y, vx, vy, vz});
        if (hist.size() > 64) hist.erase(hist.begin());
    }

    // ---- sensors at a 10 Hz tick ---------------------------------------------
    SensorTick sense(double tick_dt) {
        SensorTick o;
        o.t = t;
        o.baro_alt = h + baro_drift_rate * t + rng.nrm(0.0, s.baro_noise);
        o.gyro_rate = gyro_tick_acc / tick_dt;
        gyro_tick_acc = 0.0;
        o.gyro_angle = gyro_angle_acc;
        o.gnss_new = false;
        o.gx = o.gy = o.gve = o.gvn = o.gvd = 0.0;
        if (t >= next_gnss_t) {
            next_gnss_t += 1.0 / s.gnss_rate_hz;
            double tm = t - s.gnss_latency;
            const HistPt* hp = &hist.front();
            for (const auto& p : hist) { if (p.t <= tm) hp = &p; }
            o.gnss_new = true;
            o.gx = hp->x + gnss_ex.x + rng.nrm(0.0, s.gnss_white);
            o.gy = hp->y + gnss_ey.x + rng.nrm(0.0, s.gnss_white);
            o.gve = hp->vx + rng.nrm(0.0, s.gnss_vel_noise);
            o.gvn = hp->vy + rng.nrm(0.0, s.gnss_vel_noise);
            o.gvd = -hp->vz + rng.nrm(0.0, 1.5 * s.gnss_vel_noise);
        }
        return o;
    }

    double landingError() const {
        return std::hypot(land_x - s.target_x, land_y - s.target_y);
    }
    double meanCanopySink() const { return sink_n ? sink_sum / sink_n : 0.0; }

    // truth state
    Scenario s;
    Rng rng;
    Phase phase = PAD;
    double t = 0.0, x = 0.0, y = 0.0, h = 0.0;
    double vx = 0.0, vy = 0.0, vz = 0.0;
    double canopy_psi = 0.0, turn_rate = 0.0, can_yaw = 0.0;
    double infl_yaw0 = 0.0, infl_yaw1 = 0.0;
    double brakeR = 0.0, brakeL = 0.0;
    bool servo_override = false;
    double servo_cmd_r = 0.0, servo_cmd_l = 0.0;
    bool cut = false;
    double cut_alt = 0.0, cut_t = 0.0, infl_t0 = 0.0, canopy_t0 = 0.0, hover_end = 0.0;
    double burn_accum = 0.0;
    double land_x = 0.0, land_y = 0.0, land_t = 0.0;
    double sink_sum = 0.0; long sink_n = 0;
    double max_rate = 0.0;

private:
    struct HistPt { double t, x, y, vx, vy, vz; };
    std::vector<HistPt> hist;
    GaussMarkov gust_e, gust_n, gust_v, gnss_ex, gnss_ey;
    double drone_yaw_rate = 0.0;
    double osc_phase = 0.0;
    double baro_drift_rate = 0.0;
    double next_gnss_t = 0.0;
    double gyro_angle_acc = 0.0, gyro_tick_acc = 0.0;
};

// ----------------------------------------------------------------------------
// Statistics helpers
// ----------------------------------------------------------------------------
inline double percentile(std::vector<double> v, double p) {
    if (v.empty()) return NAN;
    std::sort(v.begin(), v.end());
    double idx = p * (v.size() - 1);
    size_t i = (size_t)idx;
    double f = idx - i;
    return (i + 1 < v.size()) ? v[i] * (1 - f) + v[i + 1] * f : v[i];
}

}  // namespace sil
