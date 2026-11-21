# Phase 4.4b — Wind-Biased Loiter Planner Independent Validation Audit

**CAN-7USAT Precision Landing Guidance Project**  
**Audit Target:** Phase 4.4 Wind-Biased Loiter Center Planning  
**Auditor Role:** Independent Guidance & Control System Validation Audit  
**Status:** Completed & Rigorously Diagnosed  
**Milestone Reference Tag:** `phase-4.3-actuator-dynamics` points to commit `c2b93b097da00dbf112b8d00cdb719f39801a234` (verified intact)  
**Test Suite Verification:** 297 passed in 12.38s (100% pass rate)

---

## 1. Executive Summary & Audit Mandate

Phase 4.4 introduced `LoiterCenterPlanner` (`guidance/loiter_center_planner.py`), which offsets the planned holding orbit center relative to the fixed landing target using flight software estimated wind velocity:
$$P_{\text{center}} = P_{\text{target}} + \Delta \vec{r}_{\text{bias}}, \quad \Delta \vec{r}_{\text{bias}} = -k_{\text{bias}} \cdot \vec{W}_{\text{est}}$$

While the Phase 4.4 report demonstrated that an upwind offset ($k_{\text{bias}} = 2.5\text{ s}$) reduces touchdown miss in a $+Y$ (Northward) 4 m/s crosswind from **9.20 m to 6.63 m** (and to **2.01 m** under realistic actuator lag), the report also revealed substantial regressions in other wind configurations:
- **$-Y$ (Southward) crosswind:** Touchdown error increased from **7.66 m to 17.48 m** (+9.82 m regression).
- **Tailwind ($+X$, Eastward):** Touchdown error increased from **2.88 m to 16.75 m** (+13.87 m regression).
- **Diagonal ($(2, 2)\text{ m/s}$):** Touchdown error increased from **11.26 m to 21.66 m** (+10.40 m regression).

This independent audit investigates whether these directional discrepancies stem from an implementation defect, a coordinate frame error, or an intrinsic structural inadequacy in the isotropic upwind-offset heuristic.

### Key Audit Findings:
1. **Mathematical Implementation Correctness:** The code correctly implements the isotropic offset equation $\Delta \vec{r}_{\text{bias}} = -k_{\text{bias}} \cdot \vec{W}_{\text{est}}$. There are no sign errors, frame inversion bugs, unit mismatches, or target mutation bugs. The fixed landing target $P_{\text{target}}$ remains strictly invariant.
2. **Exact Numerical Reproduction:** All 11 core scenarios reported in Phase 4.4 were independently reproduced with 100% precision.
3. **Severe Directional Anisotropy:** An expanded 8-point compass sweep reveals that the current heuristic creates extreme directional divergence:
   - North wind ($+Y$): **-2.56 m improvement**
   - South wind ($-Y$): **+9.82 m regression**
   - East wind ($+X$, tailwind): **+13.87 m regression**
   - Northeast wind (NE): **+17.42 m regression**
   - Southeast wind (SE): **-20.68 m improvement**
4. **Statistical Inconsistency Under Low-Frequency Winds:** While high-frequency turbulence ($\tau = 1.0\text{ s}$) shows 75% pairwise improvement (15/20 seeds), sustained turbulence ($\tau = 20.0\text{ s}$) regresses in **65% of seeds** (13/20 seeds), worsening mean touchdown error from 11.02 m to 13.34 m.
5. **Root-Cause Identified:** The regression is caused by an **intrinsically inadequate isotropic heuristic**. Shifting the orbit center directly upwind ignores the vehicle's directional clockwise orbit dynamics, entry geometry, and dynamic breakout tangent arc. Furthermore, because the final approach straight glide uses direct-bearing guidance with zero cross-track crab compensation, shifting the orbit center upstream increases the glide distance $L_{\text{glide}}$, causing the parafoil to integrate significantly larger uncorrected lateral wind drift.

---

## 2. Invariants & Rules Compliance Audit

| Rule / Requirement | Verification Method | Status | Audit Findings |
| :--- | :--- | :---: | :--- |
| **Verify tag `phase-4.3-actuator-dynamics`** | `git rev-parse phase-4.3-actuator-dynamics` | **PASS** | Points strictly to commit `c2b93b097da00dbf112b8d00cdb719f39801a234`. Tag was not moved. |
| **No production code modifications** | `git diff --check`, `git status` | **PASS** | No production code or existing tests modified during audit. |
| **Preserve diagnostic scratch files** | `git status --short --untracked-files=all` | **PASS** | All diagnostic scripts and csv traces preserved intact. |
| **Planner disabled by default** | Code inspection `LoiterCenterPlannerConfig` | **PASS** | `enabled: bool = False` by default. `sim.loiter_center_planner=None` retains 100% baseline. |
| **Target position invariance** | Code inspection and simulation assertions | **PASS** | `sim.target_position` remains $(1200.0, 0.0)$ invariant across all phases. |
| **Full test suite passing** | `pytest` full execution | **PASS** | **297 passed in 12.38s**, 0 failures, 0 regressions. |

---

## 3. Mathematical & Signal Flow Trace

### 3.1 Coordinate Conventions & Units
The project establishes:
- $+X = \text{East}$ [m], $+Y = \text{North}$ [m], $+Z = \text{Up}$ (Altitude) [m].
- Heading: $0^\circ = \text{North}$ ($+Y$), $90^\circ = \text{East}$ ($+X$), $180^\circ = \text{South}$ ($-Y$), $270^\circ = \text{West}$ ($-X$).
- Parafoil airspeed: $V_a = 8.0\text{ m/s}$, descent rate: $V_z = 2.5\text{ m/s}$, max turn rate: $30.0^\circ/\text{s}$.

In `guidance/loiter_center_planner.py`:
- Inputs: `wind_x`, `wind_y` from `WindEstimator` [m/s].
- Estimated wind magnitude: $W_{\text{speed}} = \sqrt{w_x^2 + w_y^2}$ [m/s].
- Unit vector: $\hat{w} = [w_x / W_{\text{speed}}, w_y / W_{\text{speed}}]^T$ [dimensionless].
- Bounded offset: $D_{\text{offset}} = \min(k_{\text{bias}} \cdot W_{\text{speed}}, D_{\max})$ [m], with $k_{\text{bias}} = 2.5\text{ s}$, $D_{\max} = 30.0\text{ m}$.
- Direction factor: $S_{\text{dir}} = -1.0$ for `"upwind"`.
- Spatial displacement: $\Delta \vec{r}_{\text{bias}} = S_{\text{dir}} \cdot D_{\text{offset}} \cdot \hat{w} = -k_{\text{bias}} \cdot \vec{W}_{\text{est}}$ [m].
- Planned orbit center: $P_{\text{center}} = P_{\text{target}} + \Delta \vec{r}_{\text{bias}}$ [m].

### 3.2 Mathematical Assessment of the Equations
1. **Dimensional Consistency:**
   $$[k_{\text{bias}}] \cdot [W_{\text{speed}}] = [\text{s}] \cdot [\text{m/s}] = [\text{m}]$$
   The units are dimensionally consistent.
2. **Sign Verification:**
   - For a Northward wind ($\vec{W} = [0, +4]^T$), $\Delta \vec{r}_{\text{bias}} = [0, -10]^T$, placing $P_{\text{center}}$ at $(1200.0, -10.0)$ (South). This is unambiguously upwind.
   - For a Southward wind ($\vec{W} = [0, -4]^T$), $\Delta \vec{r}_{\text{bias}} = [0, +10]^T$, placing $P_{\text{center}}$ at $(1200.0, +10.0)$ (North). This is unambiguously upwind.
   - There are **no sign or frame transformation errors** in the code.
3. **Isotropic Heuristic vs Kinematic Realism:**
   - The offset equation $\Delta \vec{r}_{\text{bias}} = -k \vec{W}$ is an **ad-hoc isotropic heuristic**. It is not derived from closed-loop orbital dynamics or terminal glide kinematics.
   - Ground-track orbit propagation is intrinsically non-isotropic because the vehicle flies a closed, single-direction circular trajectory (clockwise) with discrete entry points and a dynamic tangent-breakout threshold.

### 3.3 Genuine Exercise of Deterministic Gust
The deterministic gust scenario defined in `environment/wind.py` applies a cosine gust from $t = 40\text{ s}$ to $t = 60\text{ s}$.
- The CanSat completes drogue descent, deployment, and precision waypoint progression, reaching terminal guidance entry at $t \approx 140\text{ s}$.
- By $t = 140\text{ s}$, the gust has been inactive for $80\text{ s}$. The onboard `WindEstimator` (filter time constant $\sim 2\text{ s}$) has completely relaxed to $(0.0, 0.0)\text{ m/s}$.
- Therefore, at terminal guidance entry, the planner calculates $\Delta \vec{r}_{\text{bias}} = (0.0, 0.0)\text{ m}$.
- **Finding:** The existing deterministic gust scenario **does not exercise** the loiter-center bias feature during terminal holding. To test gust response during terminal holding, a gust must be active during the loiter phase ($t \in [140, 200]\text{ s}$).

---

## 4. Independent Numerical Reproduction

The original baseline and enabled-planner scenarios were rerun under strictly identical initial conditions ($x=0, y=0, z=1000\text{ m}, \psi=0^\circ$, deployment at 600 m, target at $(1200, 0)$):

| Scenario | Re-Run Baseline [m] | Re-Run Upwind [m] | Re-Run Delta [m] | Reported Baseline [m] | Reported Upwind [m] | Reproduction Match? |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **Zero Wind** | 4.35 | 4.35 | $\pm 0.00$ | 4.35 | 4.35 | **EXACT** |
| **+Y Crosswind (4 m/s)** | 9.20 | 6.63 | **-2.56** | 9.20 | 6.63 | **EXACT** |
| **-Y Crosswind (4 m/s)** | 7.66 | 17.48 | **+9.82** | 7.66 | 17.48 | **EXACT** |
| **Headwind (4 m/s, -X)** | 362.05 | 362.05 | $\pm 0.00$ | 362.05 | 362.05 | **EXACT** |
| **Tailwind (4 m/s, +X)** | 2.88 | 16.75 | **+13.87** | 2.88 | 16.75 | **EXACT** |
| **Diagonal (2, 2 m/s)** | 11.26 | 21.66 | **+10.40** | 11.26 | 21.66 | **EXACT** |
| **Deterministic Gust** | 4.35 | 4.35 | $\pm 0.00$ | 4.35 | 4.35 | **EXACT** |
| **Stochastic ($\tau=1$ s, s=42)** | 19.54 | 11.86 | **-7.68** | 19.54 | 11.86 | **EXACT** |
| **Stochastic ($\tau=5$ s, s=42)** | 3.94 | 9.32 | **+5.38** | 3.94 | 9.32 | **EXACT** |
| **Stochastic ($\tau=20$ s, s=42)** | 8.24 | 10.34 | **+2.10** | 8.24 | 10.34 | **EXACT** |
| **Stochastic ($\tau=50$ s, s=42)** | 8.07 | 1.85 | **-6.22** | 8.07 | 1.85 | **EXACT** |

**Reproduction Conclusion:** All reported metrics reproduce identically with zero deviation.

---

## 5. Expanded Directional & Symmetry Matrix

To understand the directional behavior, a comprehensive 8-point compass sweep was executed at $W = 4.0\text{ m/s}$ across both Clockwise (CW) and Counter-Clockwise (CCW) orbit configurations:

| Wind Direction (4 m/s) | Baseline CW Miss [m] | Upwind CW Miss [m] | Delta CW [m] | Baseline CCW Miss [m] | Upwind CCW Miss [m] | Delta CCW [m] | Outcome Classification |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :--- |
| **North (+Y, 0°)** | 9.20 | 6.63 | **-2.56** | 9.20 | 6.63 | **-2.56** | **Improvement** |
| **South (-Y, 180°)** | 7.66 | 17.48 | **+9.82** | 7.66 | 17.48 | **+9.82** | **Severe Regression** |
| **East (+X, 90°, Tailwind)** | 2.88 | 16.75 | **+13.87** | 2.88 | 16.75 | **+13.87** | **Severe Regression** |
| **West (-X, 270°, Headwind)**| 362.05 | 362.05 | $\pm 0.00$ | 362.05 | 362.05 | $\pm 0.00$ | Reachability Shortfall |
| **NE (+X, +Y, 45°)** | 5.55 | 22.97 | **+17.42** | 5.55 | 22.97 | **+17.42** | **Severe Regression** |
| **SE (+X, -Y, 135°)** | 24.19 | 3.51 | **-20.68** | 24.19 | 3.51 | **-20.68** | **Major Improvement** |
| **NW (-X, +Y, 315°)** | 246.81 | 246.81 | $\pm 0.00$ | 246.81 | 246.81 | $\pm 0.00$ | Reachability Shortfall |
| **SW (-X, -Y, 225°)** | 209.49 | 209.49 | $\pm 0.00$ | 209.49 | 209.49 | $\pm 0.00$ | Reachability Shortfall |

### Critical Architectural Insight — CW vs CCW Equivalence:
In the table above, `Base CW == Base CCW` and `Up CW == Up CCW` across all 8 directions.
- **Root Cause in Code:** In `simulation/simulator.py`, when terminal guidance is entered:
  ```python
  plan = solve_shear_terminal_plan(...)
  self.terminal_orbit_plan = plan
  self.loiter_guidance.direction = self.terminal_orbit_plan.orbit_direction
  ```
- The shear orbit planner (`guidance/shear_orbit_planner.py`) **automatically overrides** the configured orbit direction with its internally calculated optimal direction based on wind shear! Thus, passing `loiter_direction` to `ParafoilSimulator` has no effect on the terminal orbit.

---

## 6. Multi-Seed Stochastic Statistical Validation

To evaluate the planner beyond single lucky/unlucky random seeds, a rigorous 20-seed Monte Carlo evaluation was executed across two distinct correlation regimes ($\tau = 1.0\text{ s}$ and $\tau = 20.0\text{ s}$) using explicitly recorded seeds $\{100, 101, \dots, 119\}$:

### 6.1 Statistical Summary (N=20 Seeds)

| Metric | High-Frequency Turbulence ($\tau = 1.0\text{ s}$) | | Low-Frequency Turbulence ($\tau = 20.0\text{ s}$) | |
| :--- | :---: | :---: | :---: | :---: |
| | **Baseline** | **Upwind Planner** | **Baseline** | **Upwind Planner** |
| **Mean Touchdown Error** | 18.30 m | 20.02 m (+1.72 m) | **11.02 m** | 13.34 m (+2.32 m) |
| **Median Touchdown Error** | 17.13 m | **11.07 m (-6.06 m)** | **9.46 m** | 10.75 m (+1.29 m) |
| **90th Percentile Error** | **31.21 m** | 49.27 m | 24.76 m | **22.24 m** |
| **Success Rate ($\le 10\text{ m}$)** | 15.0% | **40.0% (+25%)** | **55.0%** | 40.0% (-15%) |
| **Success Rate ($\le 25\text{ m}$)** | 75.0% | **85.0% (+10%)** | **95.0%** | **95.0%** |
| **Pairwise Superiority** | 5/20 (25%) | **15/20 (75%)** | **13/20 (65%)** | 7/20 (35%) |

### 6.2 Statistical Findings
1. **For High-Frequency Turbulence ($\tau = 1.0\text{ s}$):**
   - The planner improves median touchdown error by **6.06 m** and increases the $< 10\text{ m}$ precision success rate from 15% to 40%.
   - In 75% of paired seeds, the planner outperforms the baseline.
   - However, the mean error is skewed higher (+1.72 m) by a heavy tail (90th percentile = 49.27 m) caused by outlier seeds where wind reversed abruptly immediately after latching.
2. **For Low-Frequency Turbulence ($\tau = 20.0\text{ s}$):**
   - Low-frequency turbulence resembles steady directional wind.
   - In low-frequency wind, the planner **regresses in 65% of seeds** (13 out of 20 runs).
   - Mean touchdown error increases from 11.02 m to 13.34 m, and the $< 10\text{ m}$ precision success rate drops from 55% to 40%.

---

## 7. Trajectory Geometry & Breakout Root-Cause Trace

To definitively diagnose why the planner helps $+Y$ crosswind and SE diagonal but hurts $-Y$ crosswind and tailwind, the detailed spatial trajectories of three matched cases were traced from orbit entry through final touchdown:

### Case A: $+Y$ Crosswind (4 m/s, North) — Improvement (-2.56 m)
- **Baseline:**
  - Orbit Center: $(1200.0, 0.0)$
  - Breakout Position: $(1145.6, +14.2)\text{ m}$, Breakout Heading: $292.9^\circ$, Breakout Distance: $56.3\text{ m}$
  - Final Approach Duration: $22.3\text{ s}$
  - Wind Action: Wind pushes vehicle North ($+Y$) for $22.3\text{ s}$.
  - Touchdown: $(1198.0, +9.0)\text{ m}$ $\longrightarrow$ **Miss = 9.20 m**
- **Upwind Planner:**
  - Orbit Center: $(1200.0, -10.0)\text{ m}$ (shifted South by $-10\text{ m}$)
  - Breakout Position: $(1175.8, -26.2)\text{ m}$, Breakout Heading: $292.2^\circ$, Breakout Distance: $35.7\text{ m}$
  - Final Approach Duration: $15.0\text{ s}$
  - Wind Action: Vehicle breaks out $26.2\text{ m}$ South of the target. During the shorter $15.0\text{ s}$ glide, Northward wind ($+4\text{ m/s}$) drifts the vehicle North toward the centerline ($y \approx -5.4\text{ m}$).
  - Touchdown: $(1196.2, -5.4)\text{ m}$ $\longrightarrow$ **Miss = 6.63 m**

### Case B: $-Y$ Crosswind (4 m/s, South) — Severe Regression (+9.82 m)
- **Baseline:**
  - Orbit Center: $(1200.0, 0.0)$
  - Breakout Position: $(1177.1, -5.9)\text{ m}$, Breakout Heading: $317.1^\circ$, Breakout Distance: $23.7\text{ m}$
  - Final Approach Duration: $10.3\text{ s}$
  - Touchdown: $(1198.7, +7.5)\text{ m}$ $\longrightarrow$ **Miss = 7.66 m**
- **Upwind Planner:**
  - Orbit Center: $(1200.0, +10.0)\text{ m}$ (shifted North by $+10\text{ m}$)
  - Breakout Position: $(1220.7, +24.0)\text{ m}$, Breakout Heading: $340.6^\circ$, Breakout Distance: $31.7\text{ m}$
  - Wind Action: Shifting the center North pushed the orbit downstream of the arrival path. The vehicle was forced to complete an extra half-turn, overshooting the target to the East ($x = 1220.7\text{ m}$) before the breakout altitude condition was met.
  - Breakout occurred pointing Northwest ($340.6^\circ$). During final approach, Southward wind pushed the vehicle further South to $y = -16.4\text{ m}$.
  - Touchdown: $(1194.1, -16.4)\text{ m}$ $\longrightarrow$ **Miss = 17.48 m**

### Case C: Tailwind ($+X$, 4 m/s, East) — Severe Regression (+13.87 m)
- **Baseline:**
  - Orbit Center: $(1200.0, 0.0)$
  - Vehicle loitered smoothly for 1.94 orbits, broke out at $h \approx 62\text{ m}$, and glided in cleanly.
  - Touchdown: $(1197.2, +0.7)\text{ m}$ $\longrightarrow$ **Miss = 2.88 m**
- **Upwind Planner:**
  - Orbit Center: $(1190.0, 0.0)\text{ m}$ (shifted West by $-10\text{ m}$)
  - The $-10\text{ m}$ shift placed the orbit center closer to the incoming flight path. The vehicle's ground speed over the top of the orbit reached $12\text{ m/s}$ (airspeed $8 + 4$).
  - The vehicle was unable to satisfy the geometric energy-aligned breakout threshold along its orbit arc and became trapped in loiter for **4.91 orbits**!
  - Altitude depleted down to the low-altitude emergency failsafe floor ($16.0\text{ m}$ AGL), forcing a panic breakout with only $6.5\text{ s}$ of flight remaining at heading $295.4^\circ$.
  - Touchdown: $(1214.6, +8.1)\text{ m}$ $\longrightarrow$ **Miss = 16.75 m**

---

## 8. Failure Diagnosis: Why the Isotropic Heuristic Fails

The audit conclusively identifies three interacting physical mechanisms that invalidate an isotropic upwind offset:

### 1. Asymmetry Between Orbit Entry and Breakout Tangents
A circular orbit in a wind field does not break out symmetrically. Breakout occurs when:
$$h(t) \le h_{\text{breakout}}(\vec{r}, \psi) = h_{\text{margin}} + \frac{L_{\text{turn}}(\psi \to \theta_{\text{target}})}{\text{GR}} + \frac{D_{\text{target}}}{\text{GR}}$$
This condition is evaluated continuously along the circle.
- When an isotropic shift moves the orbit center against the wind, it changes the spatial distance $D_{\text{target}}$ asymmetrically depending on whether the vehicle is flying on the upwind or downwind half of the circle.
- In $+Y$ crosswind, the shift brings the vehicle closer to the breakout threshold at an advantageous tangent angle.
- In $-Y$ crosswind and tailwind, the shift delays the threshold, trapping the vehicle in the orbit until altitude starvation forces an emergency breakout at an adverse heading.

### 2. Lack of Cross-Track Compensation in Final Approach
The final approach guidance law is frozen as **direct-bearing pursuit**:
$$\psi_{\text{cmd}} = \text{bearing}(\vec{r}, P_{\text{target}})$$
Direct-bearing guidance aims the vehicle's nose directly at the target without crabbing into the crosswind. As a result, the lateral displacement during final approach is:
$$\Delta y_{\text{drift}} \approx W_\perp \cdot t_{\text{glide}} = W_\perp \cdot \frac{h_{\text{breakout}}}{V_z}$$
An upwind center shift only improves touchdown error if the breakout point happens to be positioned exactly $W_\perp \cdot t_{\text{glide}}$ upwind of the target. In $-Y$ crosswind and diagonal winds, the breakout point is shifted to the *downwind* side or overshoots the target horizontally, doubling the accumulated error.

### 3. Tailwind Orbit Compression & Trapping
In a tailwind ($+X$), shifting the orbit center upwind ($-X$) compresses the available glide budget between the breakout point and the target. Because ground speed is high ($12\text{ m/s}$), the required turn radius and turn arc length increase, raising the calculated $h_{\text{breakout}}$ threshold above the actual altitude, causing the breakout condition to fail repeatedly.

---

## 9. Proposed Defensible Algorithm Revision

Rather than an isotropic spatial displacement $\Delta \vec{r} = -k \vec{W}$, terminal guidance energy management requires a **breakout-aligned, tangent-aware guidance architecture**:

```
[Estimated Wind W_est] + [Target P_target]
           |
           v
+-------------------------------------------------------------+
| 1. Tangent-Aligned Breakout Point Planning                  |
|    - Compute the optimal final approach glide vector        |
|      pointing directly into the wind (upwind landing)       |
|    - Define breakout waypoint P_breakout such that glide    |
|      heading aligns with ground track crab angle            |
+-------------------------------------------------------------+
           |
           v
+-------------------------------------------------------------+
| 2. Tangent-Continuous Orbit Center Placement                |
|    - Place loiter orbit center P_center tangent to the      |
|      planned final approach corridor:                       |
|      P_center = P_breakout + R * normal_vector              |
+-------------------------------------------------------------+
           |
           v
+-------------------------------------------------------------+
| 3. Final Approach Crab-Angle Guidance                       |
|    - Replace direct-bearing pursuit with line-of-sight      |
|      cross-track crab guidance (Phase 4.2g candidate C)     |
+-------------------------------------------------------------+
```

### Required Experimental Validation Before Production:
1. Validate that the planned breakout point $P_{\text{breakout}}$ produces a straight glide vector that aligns with the wind vector.
2. Confirm that orbit direction (CW vs CCW) is explicitly chosen to enter the breakout corridor tangentially without crossing over the target.
3. Test symmetry across all 8 compass directions before accepting the milestone.

---

## 10. Audit Conclusion

The Phase 4.4 implementation was executed with high code quality, comprehensive safety guards (deadbands, latching, saturation, finite checks), zero target mutation, and 100% test pass rate.

However, the core physical algorithm—an **isotropic constant upwind-offset heuristic**—is structurally unsuitable for general wind fields. It improves $+Y$ crosswind and SE diagonal by chance of geometric tangent alignment, but produces severe regressions (+9 m to +17 m) in $-Y$ crosswind, tailwind, and low-frequency stochastic winds.

```
PHASE 4.4b REQUIRES ALGORITHM REVISION
```
