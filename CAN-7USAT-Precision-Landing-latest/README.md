# CAN-7USAT Precision Landing Algorithm

Python simulation and algorithm-development project for autonomous parafoil guidance, navigation, control, and state estimation.

> **Experimental:** Simulation results do not establish hardware or flight readiness. This Python project is not directly deployable as microcontroller firmware.

## Project structure

- `control/`: PID and actuator models
- `environment/`: Wind models
- `estimation/`: Extended Kalman Filter (EKF) and state estimation
- `flight/`: Parafoil dynamics and flight states
- `guidance/`: Guidance, loiter, and breakout algorithms
- `navigation/`: Coordinates, bearings, and waypoints
- `sensors/`: Synthetic sensors and fault injection
- `simulation/`: Simulation integration
- `tests/`: Regression and integration tests
- `benchmarks/`: Benchmark scripts for scenario-specific validation

## Getting started on Windows

### 1. Clone the repository

```bash
git clone https://github.com/vamsiravii/CAN7USAT-Precision-Landing-FSW.git
cd CAN7USAT-Precision-Landing-FSW/CAN-7USAT-Precision-Landing-latest
```

### 2. Set up Python (Windows PowerShell)

The development environment has been tested with Python 3.13 on Windows.

Create a virtual environment:

```powershell
py -3.13 -m venv .venv
```

Install the project and development dependencies:

```powershell
.\.venv\Scripts\python.exe -m pip install -r requirements-dev.txt
```

On macOS or Linux:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-dev.txt
.venv/bin/python -m pytest -q
.venv/bin/python run_simulation.py
```

### 3. Run the tests

```powershell
.\.venv\Scripts\python.exe -m pytest -q
```

### 4. Run the primary simulation

```powershell
.\.venv\Scripts\python.exe run_simulation.py
```

## Reference configuration

The following values describe the simulation configuration, not certified flight parameters.

| Parameter | Value |
|---|---|
| PID proportional gain (Kp) | 10.0 |
| PID integral gain (Ki) | 0.1 |
| PID derivative gain (Kd) | 0.5 |
| Heading PID input | Radians |
| Integral limits | [-10, +10] |
| Normalized steering output | [-1, +1] |
| Nominal airspeed | 8 m/s |
| Nominal descent rate | 2.5 m/s |
| Maximum modeled turn rate | 30 degrees/second |
| Terminal loiter radius | 30 m |
| Local target | (1200, 0) m |
| Default simulation mode | Ideal |

### Coordinate conventions

- +X points East.
- +Y points North.
- Heading 0° points North.
- Heading 90° points East.
- Heading 180° points South.
- Heading 270° points West.

## Engineering reports and benchmarks

Start with [`docs/FINAL_PROJECT_STATUS.md`](docs/FINAL_PROJECT_STATUS.md). All phase reports are in [`docs/`](docs/).

The Phase 4 and Phase 5 reports document guidance, actuator dynamics, loiter planning, state estimation, sensor faults, and validation results.

Benchmark results must be interpreted alongside their scenario settings, random seeds, estimator modes, and actuator configuration. Results from one scenario should not be treated as proof of general robustness.

## Relationship to the flight firmware

This folder is the Python simulation and algorithm reference. The C++ flight code lives in [`../CAN7USAT_UNO`](../CAN7USAT_UNO) and is a separate implementation, not a direct port: it uses a lighter navigation filter (`NavFilter`) in place of this project's EKF so that it fits on small microcontrollers. Running it on hardware still requires real sensor interfaces, calibrated servo outputs, and hardware-in-the-loop testing.

Basic EMAX ES3004 servo movement does not validate the complete flight-control system, servo linkage geometry, or parafoil flight performance.

Simulation validation is not a substitute for hardware validation or controlled flight testing.