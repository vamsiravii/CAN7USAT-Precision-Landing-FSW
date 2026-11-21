# CAN-7USAT Precision Landing Guidance — Milestone Status Report

**Milestone:** Phase 5.2b — Statistical Gating Calibration and Fault-Recovery Audit
**Date:** October 12, 2026
**Status:** Validated Statistical Gating, Kinematic Alignment, and Outage Recovery
**Milestone Reference Tag:** `phase-4.3-actuator-dynamics` points to `c2b93b097da00dbf112b8d00cdb719f39801a234` (verified intact)
**Production Baseline:** Candidate A Retained (Shear Terminal Orbit Planning; Loiter Center Planner disabled by default)

---

## 1. System Architecture

The CAN-7USAT guidance, navigation, and control system governs autonomous steerable parafoil recovery for high-precision touchdown.

```
+--------------------------------------------------------------------------+
|                            MISSION FLIGHT PHASES                         |
|                                                                          |
|  DROGUE_DESCENT (1000m -> 600m)                                          |
|         |                                                                |
|         v                                                                |
|  PARAFOIL_DEPLOYMENT (600m, 2s inflation)                                |
|         |                                                                |
|         v                                                                |
|  PRECISION_GUIDANCE (Waypoint progression to target area)                |
|         |                                                                |
|         v                                                                |
|  TERMINAL_GUIDANCE (Autonomous 30m loiter holding orbit over target)     |
|         |                                                                |
|         v                                                                |
|  FINAL_APPROACH (Breakout at dynamic energy altitude, direct approach)   |
|         |                                                                |
|         v                                                                |
|  LANDING (Ground contact, steering disabled)                             |
+--------------------------------------------------------------------------+
```

### Guidance & Actuation Pipeline
1. **Shared Wind Estimator:** Flight software receives local wind velocity queries and updates an Exponential Moving Average (EMA) estimator ($\alpha = 0.05$).
2. **Guidance Layer:** Calculates desired ground track vector. Computes crab angle compensation using estimated wind and 8.0 m/s nominal airspeed.
3. **Control Layer:** Calculates heading error in degrees, normalizes internally to radians ($\text{error}_{\text{rad}} = \text{error}_{\text{deg}} \times \frac{\pi}{180}$), and evaluates a bounded PID controller yielding commanded steering $\delta_{\text{cmd}} \in [-1.0, +1.0]$.
4. **Physical Actuator Dynamics (`control/actuator.py`):**
   - Transport Delay buffer (FIFO queue of length $T_d / \Delta t$)
   - Asymmetric deadband with linear rescaling
   - Finite slew-rate limiting (max rate of position change)
   - Output command saturation clamping $[-1.0, +1.0]$
   - Pass-through configuration preserves baseline identically when unconfigured (`actuator_model=None`).
5. **Flight Dynamics Layer:** Normalized actual steering command $[-1.0, +1.0]$ maps to parafoil heading rate bounded strictly at $\le 30.0^\circ/\text{s}$.

---

## 2. Frozen Controller Configuration

The controller baseline is frozen with the following parameters:

| Parameter | Value | Unit / Range | Rationale |
| :--- | :---: | :---: | :---: |
| **PID Angular Input** | Radians | $[-\pi, +\pi]$ | Unit-consistent error input eliminating artificial degree-scale chatter |
| **Proportional Gain ($K_p$)** | 10.0 | dimensionless | Validated radian-domain tracking authority |
| **Integral Gain ($K_i$)** | 0.1 | $s^{-1}$ | Slow bias rejection without windup overshoot |
| **Derivative Gain ($K_d$)** | 0.5 | $s$ | Heading rate damping without derivative kick |
| **Integral Limits** | $[-10.0, +10.0]$ | rad$\cdot$s | Clamped anti-windup bounds |
| **Steering Output Limits** | $[-1.0, +1.0]$ | normalized | Full physical steering range |
| **Wind Estimator Filter ($\alpha$)** | 0.05 | dimensionless | Smooth disturbance rejection, immune to high-frequency gusts |
| **Maximum Parafoil Turn Rate** | 30.0 | deg/s | Physical aerodynamic turn rate limit |
| **Airspeed ($V_a$)** | 8.0 | m/s | Nominal forward airspeed |
| **Descent Rate ($V_z$)** | 2.5 | m/s | Nominal glide descent rate |
| **Terminal Loiter Radius ($R$)** | 30.0 | m | Target energy dissipation orbit radius |

---

## 3. Phase 4.4c Comparative Benchmark Findings

Phase 4.4c conducted an exhaustive evaluation of four architectural approaches:
- **Candidate A:** Unbiased Baseline (Production `ShearTerminalPlan` orbit center, `loiter_center_planner` disabled)
- **Candidate B:** Phase 4.4 Isotropic Upwind Heuristic ($\Delta \vec{r} = -k \cdot \vec{W}_{\text{est}}$)
- **Candidate C:** Breakout-Aware Geometric Planner (Along-track clamping in tailwinds, tangent cross-track offset)
- **Candidate D:** Predictive Candidate Evaluator (Flight software analytical forward scoring)

### 3.1 8-Point Compass Sweep (4.0 m/s)

| Wind Direction (4 m/s) | Cand A (Base) [m] | Cand B (Heur) [m] | Cand C (Geom) [m] | Cand D (Pred) [m] | Outcome |
| :--- | :---: | :---: | :---: | :---: | :--- |
| **Zero Wind (0 m/s)** | **4.35** | **4.35** | **4.35** | **4.35** | Parity |
| **North (+Y, 0°)** | 9.20 | 6.63 | **5.23** | 6.63 | C Best |
| **NE (45°)** | **5.55** | 22.97 | 12.33 | 12.33 | **A Decisively Best** (+17.4m reg in B) |
| **East (+X, 90°, Tailwind)**| **2.88** | 16.75 | 16.78 | 16.78 | **A Decisively Best** (+13.9m reg in B) |
| **SE (135°)** | 24.19 | **3.51** | 12.29 | 12.29 | B Best |
| **South (-Y, 180°)** | **7.66** | 17.48 | 17.39 | 17.48 | **A Decisively Best** (+9.8m reg in B) |
| **SW (225°)** | 209.49 | 209.49 | 209.49 | 209.49 | Reachability Shortfall |
| **West (-X, 270°, Headwind)**| 362.05 | 362.05 | 362.05 | 362.05 | Reachability Shortfall |
| **NW (315°)** | 246.81 | 246.81 | 246.81 | 246.81 | Reachability Shortfall |
| **Reachable Mean (6 Cases)**| **8.97 m** | **11.95 m** | **11.39 m** | **11.64 m** | **Cand A Superior Overall** |

### 3.2 Wind Speed Sweep (0 to 6 m/s)

| Wind Scenario | Cand A (Base) [m] | Cand B (Heur) [m] | Delta (B - A) [m] | Classification |
| :--- | :---: | :---: | :---: | :--- |
| **Zero Wind (0 m/s)** | **4.35** | **4.35** | $\pm 0.00$ | Baseline Parity |
| **+Y 2 m/s** | **0.48** | 7.33 | **+6.85** | Severe Regression in B |
| **+Y 4 m/s** | 9.20 | **6.63** | **-2.56** | Moderate Improvement in B |
| **+Y 6 m/s** | 484.67 | 484.67 | $\pm 0.00$ | Reachability Shortfall |
| **-Y 2 m/s** | **8.63** | 11.01 | **+2.39** | Regression in B |
| **-Y 4 m/s** | **7.66** | 17.48 | **+9.82** | Severe Regression in B |
| **-Y 6 m/s** | 362.04 | 362.04 | $\pm 0.00$ | Reachability Shortfall |
| **+X 2 m/s (Tailwind)** | 14.21 | **12.40** | **-1.81** | Minor Improvement in B |
| **+X 4 m/s (Tailwind)** | **2.88** | 16.75 | **+13.87** | Severe Regression in B |
| **+X 6 m/s (Tailwind)** | **3.54** | 3.55 | $\pm 0.00$ | Strict Float A < B (-0.0035m) |

**Speed Sweep Case-Count & Denominator Breakdown:**
- **Across all reported conditions ($N = 10$):** Candidate A matches or beats Candidate B in **8 out of 10 conditions (80.0%)** (5 wins, 3 ties/matches, 2 losses).
- **Across reachable conditions including zero wind ($N = 8$):** Candidate A matches or beats Candidate B in **6 out of 8 reachable conditions (75.0%)** (5 wins, 1 tie, 2 losses; excluding +Y 6 m/s and -Y 6 m/s reachability shortfalls).
- **Across nonzero reachable conditions ($N = 7$):** Candidate A matches or beats Candidate B in **5 out of 7 conditions (71.4%)** (5 strict wins or 4 wins + 1 practical tie, 2 losses).

### 3.3 Terminal-Overlapping Deterministic Gust ($t \in [140, 180]\text{ s}$)

- **North Gust (+Y 3 m/s on +Y 2 m/s base):** Cand A = 9.74 m | Cand B = 1.86 m (-7.88 m)
- **South Gust (-Y 3 m/s on -Y 2 m/s base):** Cand A = **6.92 m** | Cand B = 12.00 m (+5.08 m regression)
- **Conclusion:** Candidate A exhibits balanced bidirectional gust rejection. Candidate B exhibits severe directional volatility.

### 3.4 Multi-Seed Stochastic Monte Carlo ($N=20$ Seeds)

- **High-Frequency ($\tau=1.0\text{ s}$):**
  - Candidate A: Mean = **18.30 m**, Median = **17.13 m**, $\le 10$ m: **15.0%**
  - Candidate B: Mean = 27.18 m, Median = 28.22 m, $\le 10$ m: 0.0%
  - **Candidate A won 16/20 seeds (80.0% superiority).**
- **Low-Frequency ($\tau=20.0\text{ s}$):**
  - Candidate A: Mean = **11.02 m**, Median = **9.46 m**, $\le 10$ m: **55.0%**
  - Candidate B: Mean = 11.98 m, Median = 10.83 m, $\le 10$ m: 40.0%
  - **Candidate A won 12/20 seeds (60.0% superiority).**
- **Reconciliation of Phase 4.4b vs Phase 4.4c:** Baseline figures (18.30 m / 11.02 m mean) match identically with 100% precision. In Phase 4.4b, `wf.reset()` was omitted between baseline and upwind runs, causing `StochasticWind` time-caching (`time <= last_time`) to evaluate the upwind controller against a frozen constant snapshot. In Phase 4.4c, `wf.reset()` was explicitly called before each run, exposing both controllers to identical dynamic turbulence realizations where Candidate A's superior robustness is definitively confirmed.

---

## 4. Architectural Decision & Production Status

1. **Defensible Negative Result Confirmed:**
   The engineering hypothesis that shifting the terminal holding center provides robust wind compensation is refuted. Any center offset relative to the shear-planned tangent center introduces non-zero crosswinds on breakout, disrupting direct-bearing approach and causing severe regressions in tailwinds, south winds, and stochastic turbulence.
2. **Production Default Retained:**
   `LoiterCenterPlannerConfig.enabled` remains **`False` by default**. `ParafoilSimulator` utilizes the validated, robust `ShearTerminalPlan` orbit center.
3. **Target Invariance Preserved:**
   Geographic target $P_{\text{target}} = (1200.0, 0.0)$ remains strictly invariant across all phases.

---

## 5. Test Suite Verification

- **Total Test Cases:** **343 collected and executed**
- **Pass Rate:** **100% (343 passed, 0 failed, 0 skipped)**
- **Execution Time:** ~55.8s
- **New Test Modules & Tests:**
  - `tests/test_breakout_aware_planner.py` (6 unit and regression tests)
  - `tests/test_ekf_state_estimation.py` (21 unit and regression tests for sensor models, phase-aware kinematics, initial t=0 seeding, transition state isolation, EKF prediction, correction, Joseph form covariance updates, and heading wrapping)
  - `tests/test_sensor_faults_and_robustness.py` (18 unit and regression tests including Phase 5.2b statistical gating calibration, outage covariance inflation, persistent outlier rejection, and single-sensor vertical unobservability)
- **Complete Test Coverage:** Navigation, kinematics, PID control, actuator dynamics, wind fields, stochastic turbulence, terminal loiter guidance, dynamic breakout, sensor noise generation, phase-aware EKF state estimation, synthetic sensor fault injection, robust NIS statistical gating, outage re-acquisition, and simulator integration.

---

## 6. Phase 5.1 / Phase 5.1b — Sensor Noise and EKF State Estimation Status

1. **Modular Architecture:**
   - [`sensors/sensor_model.py`](../sensors/sensor_model.py) implements configurable synthetic GNSS, Barometer, and Heading sensors with discrete sampling intervals and Gaussian noise.
   - [`estimation/ekf.py`](../estimation/ekf.py) implements a 5-state Extended Kalman Filter ($\mathbf{x} = [x, y, z, \psi, \omega]^T$) with flight-phase kinematics, Joseph form covariance updates, and rigorous angle wrapping.
2. **Strict Mode Separation & Ground-Truth Isolation:**
   - `"ideal"` (Default): Simulator ground truth passed to guidance and state machine transitions. Preserves 100% numerical parity with all historical baselines.
   - `"noisy_direct"`: Raw noisy sensor measurements fed directly to guidance and state machine transitions.
   - `"noisy_ekf"`: Filtered EKF state estimates fed to guidance and state machine transitions.
3. **Validated Audited Findings (Phase 5.1b):**
   - **Phase-Aware Kinematics:** Drogue descent ($V_a = 0, V_z = 17\text{ m/s}$) propagated without artificial forward flight, dropping Position RMSE from ~13 m to **2.82 – 2.92 m**.
   - **Independent Wind Estimation:** Flight software derives wind directly from GNSS velocity and heading without circular self-reference.
   - **Decisive Accuracy Gains:**
     - Zero Wind: Noisy EKF **3.02 m** (vs Noisy Direct 3.37 m, Ideal 4.35 m).
     - Crosswind (+Y 2 m/s): Noisy EKF **4.13 m** (vs Noisy Direct 9.01 m).
     - Crosswind (+Y 4 m/s): Noisy EKF **14.63 m** (vs Noisy Direct 21.87 m).
     - Low-Frequency Turbulence ($\tau=20\text{ s}$): Noisy EKF mean **10.80 m** (median **8.02 m**, 55% $\le 10\text{ m}$), matching Ideal (11.02 m) and beating Noisy Direct (14.38 m).
     - High Sensor Noise: Noisy EKF achieves **2.61 m** vs **18.64 m** for Noisy Direct.

---

## 7. Phase 5.2 — Sensor Faults, Dropouts, and Robust State Estimation Status

1. **Synthetic Sensor Fault Framework:**
   - [`sensors/fault_model.py`](../sensors/fault_model.py) implements comprehensive degradation modes: GNSS outages, frozen/stale streams, impulsive vector outliers, barometric reference bias and linear drift, heading sensor spikes/biases, and state-triggered activation anchored to flight phase transitions.
2. **Robust Statistical Gating (NIS) & Fault Protection:**
   - [`estimation/ekf.py`](../estimation/ekf.py) incorporates Normalized Innovation Squared (NIS / Mahalanobis distance) gating with $\chi^2$ thresholds ($4\sigma$ for GNSS pos/vel, $3\sigma$ for baro and heading).
   - Timestamp staleness rejection skips frozen packets with $\Delta t \le 0$.
   - Dead-reckoning covariance inflation automatically scales process noise covariance during sustained GNSS dropouts ($t_{\text{outage}} > 1.0\text{ s}$, scale = 2.5).
   - Consecutive rejection tracking prevents filter lockout cascades during signal recovery after extended outages.
3. **Validated Benchmark Findings (10 Scenarios):**
   - **Extended GNSS Outage (15s):** Robust EKF achieved **4.02 m** touchdown error and **1.76°** heading RMSE, outperforming reference EKF (4.05 m TD err, 2.65° hdg RMSE) and far exceeding noisy-direct guidance (12.54 m).
   - **GNSS Outlier Burst:** 100% of impulsive $+40\text{ m}, -50\text{ m}, +60\text{ m}$ position outliers successfully rejected without trajectory perturbations.
   - **Stale GNSS Freeze:** Frozen data safely rejected; EKF achieved 3.77 m touchdown vs 6.86 m for noisy-direct.
   - **Barometer Bias & Drift:** Statistical gating rejected 1,813 corrupted barometric samples, cutting altitude RMSE from **38.17 m** down to **23.53 m** (38.4% improvement).
   - **Severe Compound Stress Test:** Robust EKF attained lowest touchdown error (**26.69 m** vs 27.73 m direct and 29.58 m reference EKF) and lowest heading RMSE (**1.57°**).
4. **Milestone Report:**
   - Comprehensive documentation delivered in [`phase_5_2_sensor_fault_robustness_report.md`](phase_5_2_sensor_fault_robustness_report.md).

---

## 8. Phase 5.2b — Statistical Gating Calibration and Fault-Recovery Audit Status

1. **Resolution of Nominal GPS False Rejections (Scenario S1):**
   - Root cause identified: `current_va = 0.0` during `PARAFOIL_DEPLOYMENT` created an 8 m/s airspeed prediction mismatch against the physical model propagating at 8 m/s North.
   - Kinematic alignment in `simulation/simulator.py` dropped nominal GPS false rejections from **46 to exactly 0**.
   - Position RMSE reduced from **2.85 m to 1.14 m**, peak error reduced from **15.62 m to 1.87 m**.
2. **Statistical Validation of 1-DOF Nominal Rejections:**
   - Evaluated 2,450 samples each for Barometer and Heading over 245s flight.
   - Observed empirical rejection rates (Baro: $0.245\%$; Heading: $0.326\%$) match theoretical $\chi^2(1) \ge 9.0$ tail probability ($0.270\%$). Confirmed rejections represent expected Gaussian $3\sigma$ bilateral tails.
3. **Diagnosis of Scenario S9 Crosswind Disparity:**
   - Touchdown error difference (Reference EKF 8.85m vs Robust EKF 19.22m) was diagnosed as an angular branch-cut singularity ($180^\circ$ inversion) at loiter breakout ($101.4^\circ$ bearing vs $282.0^\circ$ heading).
   - Robust EKF heading RMSE was actually superior ($2.18^\circ$ vs $2.36^\circ$). A $1.5^\circ$ heading difference flipped turn direction (into-wind vs downwind), causing the trajectory difference.
4. **Single-Sensor Vertical Observability & Barometer Drift (S7 Audit):**
   - Verified that unmodeled gradual drift ($0.2\text{ m/s}$) on the single vertical sensor is mathematically unobservable without independent vertical reference (GPS altitude, lidar, or terrain reference).
   - Documented the lockout mechanism when truth resumes ($D^2 \approx 270 \gg 9.0$) and established the requirement for vertical covariance growth or adaptive re-acquisition monitors.
5. **Principled Outage Recovery:**
   - Replaced arbitrary force-acceptance counter bypass (`max_consecutive_rejections = 10`) with calibrated dead-reckoning covariance inflation (`dead_reckoning_q_inflation = 10.0`, default `max_consecutive_rejections = 0`).
   - Verified 15s GNSS outage recovers naturally without accepting persistent outliers.
6. **Milestone Report:**
   - Comprehensive audit delivered in [`phase_5_2b_gating_recovery_audit.md`](phase_5_2b_gating_recovery_audit.md).

---

## 9. Next Engineering Priorities

1. **Phase 5.3 — Cross-Track Crab Compensation on Final Approach:** Implement cross-track line-of-sight tracking during `FINAL_APPROACH` using EKF course-over-ground to directly eliminate lateral crosswind drift.
2. **Phase 6.1 — Large-Sample Monte Carlo Dispersion Analysis:** Run $N=500$ Monte Carlo evaluations over stochastic wind, initial deployment attitudes, aerodynamic variations, sensor faults, and actuator tolerances.
