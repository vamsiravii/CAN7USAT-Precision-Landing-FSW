// ============================================================================
// CAN-7USAT SIL Monte Carlo: flies the REAL flight code (../CAN7USAT_UNO
// navigation/guidance/flight_sm/gnc .cpp, compiled natively) against the
// sim_world.h truth model, from pad -> drone climb -> release -> landing.
//
// Build:  make            (or: g++ -O2 -std=c++17 -I../CAN7USAT_UNO sil_main.cpp
//                               ../CAN7USAT_UNO/{navigation,guidance,flight_sm,gnc}.cpp -o sil)
// Run:    ./sil --n 1000                      Monte Carlo summary
//         ./sil --trace run.csv --seed 7      one flight, 10 Hz CSV trace
//         ./sil --n 1000 --runs-csv runs.csv   per-flight results
//         ./sil --trace f.csv --index 42      replay flight 42 of the Monte Carlo set
// Options: --wind-min/--wind-max [m/s]  --auth-min/--auth-max [deg/s]
//          --slack-max [0..1]  --release-radius [m]  --burn-sigma [s]  --gnss-hz [Hz]  --seed N
// ============================================================================
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "sim_world.h"
#include "gnc.h"
#include "nav_math.h"

struct RunResult {
    double err = 0.0;
    double wind = 0.0;
    double cut_alt = 0.0;
    double mean_sink = 0.0;
    double flight_t = 0.0;
    bool landed = false;
};

static RunResult runFlight(const sil::Scenario& sc, uint64_t seed, FILE* trace) {
    sil::World w(sc, seed);
    GncCore gnc;
    gnc.setTarget((float)(sc.target_x + sc.target_survey_err_x), (float)(sc.target_y + sc.target_survey_err_y));
    gnc.reset(0);
    gnc.forceState(FlightState::LAUNCH_PAD, 0);

    GncOutputs out = {0.0f, false, false, false, false};
    double baro_ema = 0.0;
    bool ema_init = false;
    uint32_t now_ms = 0;
    if (trace) {
        std::fprintf(trace, "t,phase,state,mode,x,y,h,nav_x,nav_y,nav_alt,course,des_course,steer,"
                            "brakeR,brakeL,turn_rate,wind_e,wind_n,wind_e_est,wind_n_est,va_est,"
                            "sink_s_est,sink_t_est,arrival_alt,dist,hdg_est,des_hdg,hdg_true\n");
    }
    while (w.phase != sil::World::LANDED && w.t < 4000.0) {
        for (int k = 0; k < 10; k++) w.step(0.01, out.steering, out.steering_enabled, out.burnwire);
        now_ms += 100;
        sil::SensorTick st = w.sense(0.1);
        // The sketch smooths pressure with an EMA (alpha 0.4) before computing altitude.
        baro_ema = ema_init ? 0.4 * st.baro_alt + 0.6 * baro_ema : st.baro_alt;
        ema_init = true;

        GncSensors s;
        s.now_ms = now_ms;
        s.baro_valid = true;
        s.baro_alt_agl = (float)baro_ema;
        s.gyro_valid = true;
        s.yaw_rate_dps = (float)st.gyro_rate;
        s.gnss_new = st.gnss_new;
        s.gnss_valid = true;
        s.gnss_x = (float)st.gx;
        s.gnss_y = (float)st.gy;
        s.gnss_ve = (float)st.gve;
        s.gnss_vn = (float)st.gvn;
        s.gnss_vd = (float)st.gvd;
        out = gnc.step(s);

        if (trace) {
            double we, wn;
            w.wind(w.h, we, wn);
            std::fprintf(trace, "%.1f,%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%.3f,%.3f,%.3f,%.2f,"
                                "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%.1f\n",
                         w.t, (int)w.phase, (int)gnc.sm.getState(), (int)gnc.guidance.mode(),
                         w.x, w.y, w.h, gnc.nav.x, gnc.nav.y, gnc.nav.alt, gnc.nav.courseDeg(),
                         gnc.guidance.desiredCourse, out.steering, w.brakeR, w.brakeL, w.turn_rate,
                         we, wn, gnc.air.windE, gnc.air.windN, gnc.air.airspeed, gnc.air.sinkStraight,
                         gnc.air.sinkTurn, gnc.guidance.predictedArrivalAlt, gnc.guidance.distToTarget,
                         gnc.nav.headingDeg(), gnc.guidance.desiredHeading, w.canopy_psi);
        }
    }
    RunResult r;
    r.landed = (w.phase == sil::World::LANDED);
    r.err = r.landed ? w.landingError() : 1e9;
    r.wind = sc.wind_ref;
    r.cut_alt = w.cut ? w.cut_alt : -1.0;
    r.mean_sink = w.meanCanopySink();
    r.flight_t = w.land_t;
    return r;
}

static void summarize(const char* label, const std::vector<RunResult>& rs) {
    std::vector<double> e;
    int n10 = 0, n25 = 0, n50 = 0;
    for (const auto& r : rs) {
        e.push_back(r.err);
        n10 += r.err <= 10.0; n25 += r.err <= 25.0; n50 += r.err <= 50.0;
    }
    if (e.empty()) return;
    double n = (double)e.size();
    std::printf("  %-18s n=%4zu  median(CEP50)=%6.1f m  P90=%6.1f m  P95=%6.1f m  "
                "<=10m %3.0f%%  <=25m %3.0f%%  <=50m %3.0f%%\n",
                label, e.size(), sil::percentile(e, 0.5), sil::percentile(e, 0.9),
                sil::percentile(e, 0.95), 100 * n10 / n, 100 * n25 / n, 100 * n50 / n);
}

int main(int argc, char** argv) {
    int n = 500;
    uint64_t seed = 1;
    const char* trace_path = nullptr;
    const char* runs_path = nullptr;
    int index = -1;
    sil::ScenarioRanges g;
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        auto next = [&]() { return (i + 1 < argc) ? argv[++i] : (char*)"0"; };
        if (a == "--n") n = std::atoi(next());
        else if (a == "--seed") seed = std::strtoull(next(), nullptr, 10);
        else if (a == "--trace") trace_path = next();
        else if (a == "--runs-csv") runs_path = next();
        else if (a == "--index") index = std::atoi(next());
        else if (a == "--wind-min") g.wind_min = std::atof(next());
        else if (a == "--wind-max") g.wind_max = std::atof(next());
        else if (a == "--auth-min") g.turn_auth_min = std::atof(next());
        else if (a == "--auth-max") g.turn_auth_max = std::atof(next());
        else if (a == "--slack-max") g.slack_max = std::atof(next());
        else if (a == "--release-radius") g.release_radius = std::atof(next());
        else if (a == "--burn-sigma") g.burn_sigma = std::atof(next());
        else if (a == "--gnss-hz") g.gnss_rate_hz = std::atof(next());
        else { std::fprintf(stderr, "unknown option %s\n", a.c_str()); return 2; }
    }

    if (trace_path) {
        // --index I replays flight I of the Monte Carlo run with the same --seed
        sil::Rng r(seed);
        sil::Scenario sc = sil::randomScenario(r, g);
        for (int k = 0; k < index; k++) sc = sil::randomScenario(r, g);
        uint64_t run_seed = (index >= 0) ? seed * 1000003ULL + (uint64_t)index : seed * 7919 + 13;
        FILE* f = std::fopen(trace_path, "w");
        RunResult res = runFlight(sc, run_seed, f);
        std::fclose(f);
        std::printf("seed %llu: wind %.1f m/s toward %.0f deg, Va %.2f, sink %.2f, auth %.0f deg/s, "
                    "trim %.1f deg/s -> landing error %.1f m, line cut at %.0f m, mean sink %.2f m/s\n",
                    (unsigned long long)seed, sc.wind_ref, sc.wind_dir, sc.va, sc.sink, sc.turn_auth,
                    sc.trim_bias, res.err, res.cut_alt, res.mean_sink);
        return 0;
    }

    std::vector<RunResult> all;
    sil::Rng r(seed);
    FILE* rf = runs_path ? std::fopen(runs_path, "w") : nullptr;
    if (rf) std::fprintf(rf, "i,err,wind,dir,alpha,veer,gust,va,sink,auth,tau,trim,slack,burn,rel_r,cut_alt,mean_sink\n");
    for (int i = 0; i < n; i++) {
        sil::Scenario sc = sil::randomScenario(r, g);
        all.push_back(runFlight(sc, seed * 1000003ULL + (uint64_t)i, nullptr));
        if (rf) {
            const RunResult& x = all.back();
            std::fprintf(rf, "%d,%.2f,%.2f,%.0f,%.2f,%.1f,%.2f,%.2f,%.2f,%.1f,%.2f,%.2f,%.3f,%.2f,%.1f,%.1f,%.2f\n",
                         i, x.err, sc.wind_ref, sc.wind_dir, sc.shear_alpha, sc.veer, sc.gust_sigma, sc.va,
                         sc.sink, sc.turn_auth, sc.turn_tau, sc.trim_bias, sc.slack, sc.burn_time,
                         std::hypot(sc.release_x - sc.target_x, sc.release_y - sc.target_y), x.cut_alt, x.mean_sink);
        }
    }
    if (rf) std::fclose(rf);

    std::printf("Corrected flight code, %d Monte Carlo flights (wind %.1f-%.1f m/s at 600 m, "
                "turn authority %.0f-%.0f deg/s)\n", n, g.wind_min, g.wind_max, g.turn_auth_min, g.turn_auth_max);
    summarize("all", all);
    const double bins[][2] = {{0, 2}, {2, 4}, {4, 5}, {5, 6}, {6, 99}};
    for (const auto& b : bins) {
        std::vector<RunResult> sub;
        for (const auto& x : all) if (x.wind >= b[0] && x.wind < b[1]) sub.push_back(x);
        char lbl[32];
        std::snprintf(lbl, sizeof lbl, "wind %.0f-%.0f m/s", b[0], b[1] > 50 ? 99.0 : b[1]);
        if (!sub.empty()) summarize(lbl, sub);
    }
    std::vector<double> cut, sink;
    int in_tol = 0, sink_ok = 0;
    for (const auto& x : all) {
        cut.push_back(x.cut_alt);
        sink.push_back(x.mean_sink);
        in_tol += std::fabs(x.cut_alt - 600.0) <= 10.0;
        sink_ok += (x.mean_sink >= 1.0 && x.mean_sink <= 3.0);
    }
    std::printf("  line-cut altitude (true): median %.0f m, 5-95%% %.0f-%.0f m, within 600+/-10 m: %.0f%%\n",
                sil::percentile(cut, 0.5), sil::percentile(cut, 0.05), sil::percentile(cut, 0.95),
                100.0 * in_tol / all.size());
    std::printf("  mean canopy sink: median %.2f m/s, 5-95%% %.2f-%.2f m/s, within 1-3 m/s: %.0f%%\n",
                sil::percentile(sink, 0.5), sil::percentile(sink, 0.05), sil::percentile(sink, 0.95),
                100.0 * sink_ok / all.size());
    return 0;
}
