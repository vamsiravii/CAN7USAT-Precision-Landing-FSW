# CAN7USAT Precision Landing: Flight Software and GNC Simulation

Flight software and guidance, navigation and control (GNC) simulation for a steerable-parafoil CanSat, built by **Team Parikshit (Team 017)** for the IN-SPACe CAN-7USAT Student Competition 2026-27. The CanSat is carried up, released, descends under a drogue, deploys a parafoil, and steers itself to a target landing point.

The companion ground station is in [can7usat-ground-control-station](https://github.com/vamsiravii/CAN7USAT-ground-control-station).

> **Status:** experimental. Results below are from software simulation. They do not show flight readiness, and field testing is still to come.

## Mission sequence

The firmware runs a 10-state flight state machine (`CAN7USAT_UNO/flight_sm.cpp`):

```
BOOT -> LAUNCH_PAD -> ASCENT -> DROGUE_DESCENT -> PARAFOIL_DEPLOYMENT
     -> PRECISION_GUIDANCE -> TERMINAL_GUIDANCE -> FINAL_APPROACH -> LANDING
```

A tenth state, `FAILSAFE`, is defined for emergency recovery.

Guidance has four modes: `HOMING` toward the target, `LOITER` to burn height, `DOWNWIND`, and `FINAL`. A burnwire cut releases the canopy at about 600 m, and two brake servos then steer it.

## Repository layout

| Folder | What it is |
|---|---|
| [`CAN7USAT_UNO/`](CAN7USAT_UNO) | C++ flight firmware for Arduino Uno (ATmega328P) and Mega 2560 |
| [`CAN7USAT_FLATSAT_UNO/`](CAN7USAT_FLATSAT_UNO) | FlatSat (bench) build of the same firmware. Currently identical source apart from the sketch filename |
| [`sil/`](sil) | Native software-in-the-loop harness: compiles the real flight code on a PC and flies it against a simulated world |
| [`CAN-7USAT-Precision-Landing-latest/`](CAN-7USAT-Precision-Landing-latest) | Python 6-DOF simulation, guidance algorithms, 33 test files, and engineering reports in `docs/` |

### Firmware modules (`CAN7USAT_UNO/`)

| File | Role |
|---|---|
| `CAN7USAT_UNO.ino` | Entry point: sensor reads, serial/LoRa command handling, telemetry, SD logging |
| `flight_sm.*` | Flight state machine, burnwire and buzzer timing |
| `navigation.*` | `NavFilter` (GNSS + gyro + barometer) and wind/air-data estimation |
| `guidance.*` | Homing, loiter, downwind and final guidance with a heading PID controller |
| `gnc.*` | Ties navigation, guidance and the state machine together |
| `actuators.*` | Brake servos, reaction-wheel ESC, burnwire and buzzer drivers |
| `config.h` | Pin map, radio settings, loop rates, safety limits |

**Hardware targeted:** MS5611 barometer, MPU6050 IMU, u-blox NEO-M9N GNSS, Ra-08H LoRa radio, two EMAX ES3004 brake servos, MOSFET-switched nichrome burnwire and buzzer, and a BLDC reaction wheel. Pin assignments are documented at the top of [`config.h`](CAN7USAT_UNO/config.h).

## Run the simulation

The SIL harness needs only a C++17 compiler:

```bash
cd sil
make
./sil --n 200                     # 200-flight Monte Carlo summary
./sil --trace run.csv --seed 7    # one flight, 10 Hz CSV trace
```

Options such as `--wind-min`, `--wind-max` and `--gnss-hz` are listed at the top of [`sil/sil_main.cpp`](sil/sil_main.cpp).

For the Python simulation and its tests, see [`CAN-7USAT-Precision-Landing-latest/README.md`](CAN-7USAT-Precision-Landing-latest/README.md).

## Simulation results

200-flight Monte Carlo with the real flight code (default seed; wind 0-6 m/s at 600 m, turn authority 25-50 deg/s). Distances are miss distance from the target:

| Wind at 600 m | Flights | Median | 90th percentile | Within 25 m |
|---|---|---|---|---|
| 0-2 m/s | 60 | 4.1 m | 12.1 m | 97% |
| 2-4 m/s | 78 | 10.4 m | 52.8 m | 79% |
| 4-5 m/s | 39 | 170.0 m | 396.9 m | 23% |
| 5-6 m/s | 23 | 253.9 m | 571.9 m | 9% |
| **All** | **200** | **11.1 m** | **331.9 m** | **66%** |

In light wind the system lands close to the target. Above about 4 m/s accuracy drops sharply, which is the main weakness to investigate before relying on it.

## Further reading

- Simulation design and phase-by-phase validation: [`CAN-7USAT-Precision-Landing-latest/docs/`](CAN-7USAT-Precision-Landing-latest/docs)
- Ground station for this firmware's telemetry: [can7usat-ground-control-station](https://github.com/vamsiravii/CAN7USAT-ground-control-station)
