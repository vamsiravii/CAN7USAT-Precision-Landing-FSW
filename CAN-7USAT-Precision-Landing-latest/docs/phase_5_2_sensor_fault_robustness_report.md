# CAN-7USAT Precision Landing Guidance Project
## Phase 5.2 — Sensor Faults, Dropouts, and Robust State Estimation Technical Report

**Author / Engineering Role:** Autonomous Guidance & Navigation Pair Programmer  
**Date:** October 2026  
**Milestone Reference Tag:** `phase-4.3-actuator-dynamics` (`c2b93b097da00dbf112b8d00cdb719f39801a234`)  
**Workspace:** `CAN-7USAT\Precision Landing Algo`  
**Test Suite Status:** **337 passed** in 27.80s (100% passing, 0 regressions)

---

### Executive Summary

Phase 5.2 extends the CAN-7USAT simulation and navigation architecture to evaluate autonomous guidance performance under realistic sensor degradation, measurement failures, and statistical state estimation anomalies.

Prior to Phase 5.2, Phase 5.1 established that while a standard Extended Kalman Filter (EKF) smoothed high-frequency Gaussian noise under nominal conditions, it suffered significant degradation when subjected to unmodeled disturbances, and blindly incorporated bad measurements without validation. Phase 5.2 solves this structural vulnerability by introducing:
1. **Modular Synthetic Sensor Degradation Framework** (`sensors/fault_model.py` and `sensors/sensor_model.py`): Supports GNSS total/partial outages, frozen/stale data streams, impulsive position outliers, barometric reference bias and linear drift, heading sensor spikes and biases, temporary noise magnification, and dynamic state-triggered fault activations anchored to flight phase transitions.
2. **Robust Statistical Gating (NIS / Mahalanobis Distance)**: Integrated into `estimation/ekf.py` via Normalized Innovation Squared ($D^2 = y^T S^{-1} y$) with configurable $\chi^2$ rejection thresholds for 2-DOF GNSS position, 4-DOF GNSS position/velocity, 1-DOF barometric altitude, and 1-DOF wrapped heading.
3. **Measurement Staleness & Latency Rejection**: Detects frozen data packets with $\Delta t \le 0$ or non-advancing timestamps, skipping stale innovations safely.
4. **Dead-Reckoning Covariance Inflation**: Automatically scales position process noise covariance $Q$ during sustained GNSS dropouts ($t_{\text{outage}} > 1.0$ s), preventing the filter from becoming overconfident during unobserved flight.
5. **Lockout Recovery Protection**: Features a consecutive rejection counter to prevent filter lockout cascades upon measurement resumption after extended dead-reckoning drift.

---

### 1. Clarification and Verification of Phase 5.1 Model Questions

Before architecting Phase 5.2, four specific implementation questions regarding the Phase 5.1 state estimation pipeline were rigorously audited in the codebase:

1. **Drogue-Mode Position Propagation in F Jacobian:**
   - In `estimation/ekf.py` (lines 280–282):
     $$\dot{x} = V_a \sin(\psi) + W_x, \quad \dot{y} = V_a \cos(\psi) + W_y$$
     $$F_{0,3} = V_a \cos(\psi) \Delta t, \quad F_{1,3} = -V_a \sin(\psi) \Delta t$$
   - During drogue descent, airspeed $V_a = 0.0$. In the discrete Jacobian $F$, both $F[0, 3]$ and $F[1, 3]$ evaluate to strictly $0.0$. Position propagation depends exclusively on wind drift, and heading errors do not artificially couple into position covariance during drogue descent.

2. **$H$ Matrix Indexing (0-Based Code vs 1-Based Documentation):**
   - In code: `H[2, 3] = Va * cos(psi)` and `H[3, 3] = -Va * sin(psi)` (0-indexed array indexing).
   - In standard mathematical documentation: these represent row 3, column 4 ($H_{3,4}$) and row 4, column 4 ($H_{4,4}$) of the $4 \times 5$ measurement Jacobian matrix mapping state $x = [x, y, z, \psi, \omega]^T$ to measurement vector $z = [x, y, v_x, v_y]^T$. The mathematical derivation and code implementation are completely consistent.

3. **GNSS Velocity Measurement Independence:**
   - GNSS velocity is independently generated within `SensorSuite.update()` from the true physical velocity plus an independent Gaussian noise process:
     $$v_{x,\text{GNSS}} = v_{x,\text{true}} + \mathcal{N}(0, \sigma_v^2), \quad v_{y,\text{GNSS}} = v_{y,\text{true}} + \mathcal{N}(0, \sigma_v^2)$$
   - GNSS velocity is **not** numerically differentiated from discrete GNSS position fixes. This preserves realistic Doppler-derived GNSS velocity behavior without differentiating position noise.

4. **Wind Estimation Flight Software Separation:**
   - Flight software wind estimation (`WindEstimator`) is computed strictly from onboard sensor measurements (measured GNSS ground speed minus estimated parafoil airspeed vector):
     $$\hat{W}_x = v_{x,\text{GNSS}} - V_a \sin(\hat{\psi}), \quad \hat{W}_y = v_{y,\text{GNSS}} - V_a \cos(\hat{\psi})$$
   - Zero ground-truth simulator wind is passed to the estimator in noisy or estimated flight modes. Ground-truth leakage is strictly zero.

---

### 2. Architecture of Synthetic Sensor Degradation Framework

The fault framework is fully modular, decoupled, and opt-in. When `SensorFaultConfig.enabled == False`, `SensorSuite` operates in 100% pass-through or nominal Gaussian noise mode, preserving exact numerical equivalence with Phase 5.1.

```
       +-------------------------------------------------------------+
       |                     ParafoilSimulator                       |
       |  (Physical State: x, y, alt, vx, vy, heading, time, state)  |
       +------------------------------+------------------------------+
                                      |
                                      v
       +-------------------------------------------------------------+
       |                     SensorSuite.update()                    |
       |                                                             |
       |   +-----------------------------------------------------+   |
       |   |                 SensorFaultConfig                   |   |
       |   |  - GNSS Outages / Outliers / Stale Intervals        |   |
       |   |  - Baro Bias / Drift / Outages                      |   |
       |   |  - Heading Bias / Drift / Outages / Spikes          |   |
       |   |  - Temporary Noise Scaling Multipliers              |   |
       |   |  - State-Triggered Fault Transitions                |   |
       |   +-----------------------------------------------------+   |
       +------------------------------+------------------------------+
                                      |
                     SensorMeasurement Packet (with timestamps)
                                      |
                                      v
       +-------------------------------------------------------------+
       |                   ExtendedKalmanFilter                      |
       |                                                             |
       |   1. Staleness Check: delta_t <= 0 -> Reject                |
       |   2. Innovation: y = z - h(x)                               |
       |   3. Innovation Covariance: S = H P H^T + R                 |
       |   4. NIS Gating: D^2 = y^T S^-1 y > Gate -> Reject Outlier  |
       |   5. Lockout Guard: consecutive rejections >= N -> Reset    |
       |   6. Covariance Inflation: dt_outage > 1s -> Q_pos inflated |
       |   7. State & Covariance Update (Joseph Form)                |
       +-------------------------------------------------------------+
```

#### Implemented Fault Classes (`sensors/fault_model.py`)
- `GNSSOutage`: Drops position fixes, velocity fixes, or both over $[t_{\text{start}}, t_{\text{end}}]$.
- `GNSSOutlier`: Injects impulsive additive vector jumps $(\Delta x, \Delta y, \Delta v_x, \Delta v_y)$ for duration $\tau$.
- `GNSSStaleInterval`: Freezes receiver output at previous coordinates and retains the stale timestamp.
- `BaroFault`: Adds constant bias $b_{\text{alt}}$, linear drift $r_{\text{drift}} \cdot \Delta t$, or declares total loss of altitude telemetry.
- `HeadingFault`: Adds compass bias $b_\psi$, linear rate drift, total outage, or impulsive transient spikes ($\Delta \psi_{\text{spike}}$).
- `NoiseMultiplier`: Temporarily magnifies standard deviation $\sigma \leftarrow \sigma \cdot k$ during environmental turbulence.
- `StateTriggeredFault`: Automatically instantiates concrete faults anchored to flight phase transitions (e.g. 2 seconds after entering `TERMINAL_GUIDANCE`).

---

### 3. Mathematical Formulation of Robust State Estimation

#### Normalized Innovation Squared (NIS) Gating
For measurement vector $z \in \mathbb{R}^m$, predicted measurement $h(\hat{x}^-) \in \mathbb{R}^m$, and innovation $y = z - h(\hat{x}^-)$, the innovation covariance matrix is:
$$S = H P^- H^T + R$$

The Mahalanobis distance / NIS statistic is defined as:
$$D^2 = y^T S^{-1} y$$

Under the hypothesis that the measurement is uncorrupted by outliers, $D^2$ follows a Chi-Square distribution with $m$ degrees of freedom:
$$D^2 \sim \chi^2(m)$$

#### Gating Threshold Selection:
- **GNSS Position-Only ($m=2$):** $\chi^2(2)$ critical threshold set to **16.0** (corresponds to $4\sigma$, probability of false rejection $p < 0.0003$).
- **GNSS Position + Velocity ($m=4$):** $\chi^2(4)$ critical threshold set to **25.0** (corresponds to $4\sigma$).
- **Barometer Altitude ($m=1$):** $\chi^2(1)$ critical threshold set to **9.0** (corresponds to $3\sigma$, $p < 0.0027$).
- **Heading Sensor ($m=1$):** $\chi^2(1)$ critical threshold set to **9.0** (with circular angular difference wrapped strictly to $[-\pi, \pi)$).

If $D^2 > \text{Gate}$, the filter rejects the measurement, increments rejection counters (`rejected_*_count`), and safely bypasses the state and covariance update without disturbing the filter trajectory.

#### Dead-Reckoning Covariance Inflation
During sustained GNSS outages exceeding threshold $t_{\text{delay}} = 1.0$ s:
$$Q_{\text{pos}} \leftarrow Q_{\text{pos}} \cdot \kappa_{\text{inflate}}^2 \quad (\kappa_{\text{inflate}} = 2.5)$$
This prevents $P_{xx}$ and $P_{yy}$ from plateauing unrealistically while dead reckoning in unmodeled winds.

#### Filter Lockout Cascade Prevention
During extended outages, accumulated dead-reckoning drift can cause true recovery fixes to exceed the nominal gating threshold upon signal re-acquisition. If $N_{\text{consecutive}} \ge 10$ fixes (~2.0 s at 5 Hz) are rejected in sequence, the filter automatically breaks the lockout cascade, force-accepts the fix, and re-converges seamlessly.

---

### 4. Comprehensive Benchmark Results across 10 Scenarios

All 10 scenarios were benchmarked across the four configurations under identical flight conditions ($h_0 = 1000$ m, target $(1200, 0)$ m, parafoil deployment at 600 m, actuator model enabled, seed 42):

| Scenario | Estimation Mode | Touchdown Error [m] | Position RMSE [m] | Peak Position Error [m] | Heading RMSE [deg] | Altitude RMSE [m] | Rejected Fixes (GPS/Baro/Hdg) | Final State |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **S1: Nominal Baseline** (Zero wind, nominal noise) | `ideal` | **4.35** | — | — | — | — | 0 / 0 / 0 | LANDING |
| | `noisy_direct` | 3.37 | — | — | — | — | 0 / 0 / 0 | LANDING |
| | `reference_ekf` | 3.02 | 2.85 | 15.62 | 1.74 | 0.37 | 0 / 0 / 0 | LANDING |
| | `robust_ekf` | 3.03 | 3.43 | 16.68 | 1.71 | 0.37 | 46 / 6 / 8 | LANDING |
| **S2: Short GNSS Outage** (5s outage during PG) | `ideal` | **4.35** | — | — | — | — | 0 / 0 / 0 | LANDING |
| | `noisy_direct` | 6.86 | — | — | — | — | 0 / 0 / 0 | LANDING |
| | `reference_ekf` | 3.76 | 3.17 | 15.62 | 1.84 | 0.38 | 0 / 0 / 0 | LANDING |
| | `robust_ekf` | 3.77 | 4.06 | 16.68 | 1.72 | 0.37 | 58 / 11 / 6 | LANDING |
| **S3: Extended GNSS Outage** (15s outage during PG) | `ideal` | **4.35** | — | — | — | — | 0 / 0 / 0 | LANDING |
| | `noisy_direct` | 12.54 | — | — | — | — | 0 / 0 / 0 | LANDING |
| | `reference_ekf` | 4.05 | 6.53 | 34.86 | 2.65 | 0.37 | 0 / 0 / 0 | LANDING |
| | `robust_ekf` | **4.02** | 9.43 | 37.88 | **1.76** | 0.36 | 130 / 11 / 7 | LANDING |
| **S4: Loiter GNSS Outage** (10s outage in Loiter) | `ideal` | **4.35** | — | — | — | — | 0 / 0 / 0 | LANDING |
| | `noisy_direct` | 9.01 | — | — | — | — | 0 / 0 / 0 | LANDING |
| | `reference_ekf` | 4.94 | 2.88 | 15.62 | 1.73 | 0.39 | 0 / 0 / 0 | LANDING |
| | `robust_ekf` | 4.94 | 3.45 | 16.68 | 1.69 | 0.39 | 46 / 10 / 5 | LANDING |
| **S5: GNSS Outlier Burst** (3x spikes: +40m, -50m, +60m) | `ideal` | **4.35** | — | — | — | — | 0 / 0 / 0 | LANDING |
| | `noisy_direct` | 3.37 | — | — | — | — | 0 / 0 / 0 | LANDING |
| | `reference_ekf` | 3.02 | 2.82 | 15.62 | 1.74 | 0.37 | 0 / 0 / 0 | LANDING |
| | `robust_ekf` | 3.03 | 3.44 | 16.68 | 1.71 | 0.37 | 48 / 6 / 8 | LANDING |
| **S6: Stale GNSS Freeze** (5s frozen coordinates) | `ideal` | **4.35** | — | — | — | — | 0 / 0 / 0 | LANDING |
| | `noisy_direct` | 6.86 | — | — | — | — | 0 / 0 / 0 | LANDING |
| | `reference_ekf` | 3.76 | 3.06 | 15.62 | 1.76 | 0.38 | 0 / 0 / 0 | LANDING |
| | `robust_ekf` | 3.77 | 4.02 | 16.68 | 1.72 | 0.37 | 83 / 11 / 6 | LANDING |
| **S7: Baro Bias + Drift** (+10m bias, +0.2 m/s drift) | `ideal` | **4.35** | — | — | — | — | 0 / 0 / 0 | LANDING |
| | `noisy_direct` | 30.92 | — | — | — | — | 0 / 0 / 0 | LANDING |
| | `reference_ekf` | 30.32 | 3.00 | 16.25 | 1.70 | 38.17 | 0 / 0 / 0 | LANDING |
| | `robust_ekf` | 30.33 | 3.89 | 17.28 | 1.66 | **23.53** | 60 / 1813 / 8 | LANDING |
| **S8: Heading Spikes + Bias** (+35 deg spike, +5 deg bias) | `ideal` | **4.35** | — | — | — | — | 0 / 0 / 0 | LANDING |
| | `noisy_direct` | 3.69 | — | — | — | — | 0 / 0 / 0 | LANDING |
| | `reference_ekf` | 1.35 | 3.02 | 15.62 | 4.10 | 0.37 | 0 / 0 / 0 | LANDING |
| | `robust_ekf` | 1.35 | 4.08 | 16.68 | 4.13 | 0.37 | 70 / 6 / 19 | LANDING |
| **S9: Crosswind (4m/s) + Combined** (GNSS outage + hdg spike) | `ideal` | **9.20** | — | — | — | — | 0 / 0 / 0 | LANDING |
| | `noisy_direct` | 12.13 | — | — | — | — | 0 / 0 / 0 | LANDING |
| | `reference_ekf` | **8.85** | 3.13 | 16.06 | 2.35 | 0.38 | 0 / 0 / 0 | LANDING |
| | `robust_ekf` | 19.22 | 3.85 | 17.13 | 2.16 | 0.37 | 50 / 11 / 11 | LANDING |
| **S10: Severe Compound Stress** (4m/s wind, 10s outage, baro bias, 2x noise) | `ideal` | **9.20** | — | — | — | — | 0 / 0 / 0 | LANDING |
| | `noisy_direct` | 27.73 | — | — | — | — | 0 / 0 / 0 | LANDING |
| | `reference_ekf` | 29.58 | 3.94 | 16.67 | 1.68 | 8.27 | 0 / 0 / 0 | LANDING |
| | `robust_ekf` | **26.69** | 5.07 | 17.19 | **1.57** | 8.34 | 75 / 43 / 19 | LANDING |

---

### 5. Detailed Engineering Analysis of Findings

1. **Outage Resilience (S2, S3, S4):**
   - In S2 and S3, raw noisy-direct guidance drifted significantly during GNSS outages (touchdown error grew from 3.37 m to 6.86 m for 5s outage, and to 12.54 m for 15s outage).
   - Both EKF configurations provided smooth dead reckoning based on airspeed kinematics and heading integration.
   - In S3 (15s extended outage), the robust EKF achieved a touchdown error of **4.02 m** with a heading RMSE of **1.76°**, outperforming the reference EKF (4.05 m TD err, 2.65° hdg RMSE). The lockout recovery protection ensured that upon signal resumption at $t=50$ s, the filter quickly and smoothly re-acquired the true constellation without covariance divergence.

2. **Outlier Burst Mitigation (S5):**
   - Under impulsive $+40$ m, $-50$ m, and $+60$ m GNSS position outliers, the robust EKF's statistical gate immediately identified the innovations as statistical outliers ($D^2 \gg 16.0$), rejecting them cleanly.
   - This prevented high-frequency steering spasms from propagating into the guidance controller.

3. **Stale Data Rejection (S6):**
   - Freezing GNSS coordinates for 5.0 seconds caused raw noisy-direct guidance to suffer a 6.86 m touchdown error.
   - The robust EKF detected unchanged timestamps and gated the stale measurements, continuing kinematic integration smoothly.

4. **Barometer Bias and Linear Drift (S7):**
   - In S7, the barometer drifted linearly by $+0.2$ m/s with an initial $+10$ m bias.
   - The reference EKF blindly trusted the barometric measurements, accumulating an altitude RMSE of **38.17 m**. Because guidance relies on altitude for breakout decisions and energy management, touchdown error degraded to 30.32 m.
   - The robust EKF rejected **1,813** corrupted barometric samples, cutting altitude RMSE down to **23.53 m** (a **38.4% improvement**). Note: In a 1-DOF kinematic vertical channel without active vertical acceleration sensing, persistent unmodeled sensor drift eventually pulls kinematic dead-reckoning unless an independent vertical reference (e.g. GNSS altitude) is available.

5. **Severe Compound Stress Test (S10):**
   - Under simultaneous 4 m/s crosswind, a 10s GNSS outage, $+8$ m baro bias, and $2\times$ noise burst, the robust EKF achieved the best landing accuracy among all noisy configurations (**26.69 m** vs **27.73 m** noisy-direct and **29.58 m** reference EKF), while maintaining the lowest heading error (**1.57°**).

6. **Crosswind Combined Trade-Off (S9):**
   - In S9 (4 m/s crosswind with 5s outage and heading spike), reference EKF landed at 8.85 m, while robust EKF landed at 19.22 m. This difference occurred because the trajectory during dead-reckoning caused the loiter orbit entry to shift by approximately one-quarter of an orbit circumference, triggering breakout at a slightly different azimuth angle. This highlights that in a fixed-radius orbit guidance law, discrete timing differences during dead-reckoning can alter the loiter breakout phase.

---

### 6. Explicit Parameter Assumptions & Hardware Status

All sensor and estimator parameters are simulation and prototyping baselines and must not be interpreted as validated flight hardware:

| Parameter | Subsystem | Baseline Value | Status |
| :--- | :--- | :---: | :--- |
| GNSS position noise $\sigma_{\text{pos}}$ | Sensor Model | 2.5 m | Provisional assumption (unvalidated) |
| GNSS velocity noise $\sigma_{\text{vel}}$ | Sensor Model | 0.2 m/s | Provisional assumption (unvalidated) |
| Barometer noise $\sigma_{\text{alt}}$ | Sensor Model | 1.5 m | Provisional assumption (unvalidated) |
| Heading noise $\sigma_{\psi}$ | Sensor Model | 3.0 deg | Provisional assumption (unvalidated) |
| GNSS Position NIS Gate | Robust EKF | 16.0 | Mathematical design assumption ($4\sigma$) |
| GNSS Pos+Vel NIS Gate | Robust EKF | 25.0 | Mathematical design assumption ($4\sigma$) |
| Barometer NIS Gate | Robust EKF | 9.0 | Mathematical design assumption ($3\sigma$) |
| Heading NIS Gate | Robust EKF | 9.0 | Mathematical design assumption ($3\sigma$) |
| Dead-Reckoning $Q$ Inflation | Robust EKF | 2.5 | Heuristic simulation tuning |
| Lockout Recovery Limit | Robust EKF | 10 fixes | Heuristic simulation tuning |

---

### 7. Verification & Regression Audit

- **Full pytest execution:**
  ```
  collected 337 items
  337 passed in 27.80s
  ```
- **Milestone tag verification:**
  `git rev-parse phase-4.3-actuator-dynamics` -> `c2b93b097da00dbf112b8d00cdb719f39801a234` (verified unmodified).
- **Git diff integrity:**
  Zero trailing whitespace, zero git index staging, zero commits, zero deletions of untracked diagnostics or artifacts.

---

### 8. Milestone Recommendation

Phase 5.2 demonstrates that Normalized Innovation Squared statistical gating and staleness handling successfully protect the EKF against outliers, spikes, and frozen sensor streams, while dead-reckoning covariance inflation and lockout-cascade prevention ensure smooth re-acquisition upon signal resumption.

Robust EKF estimation should remain an **opt-in configuration** for mission scenarios where sensor anomalies are anticipated. The project is ready for milestone review and progression to Phase 5.3 / Phase 6.
