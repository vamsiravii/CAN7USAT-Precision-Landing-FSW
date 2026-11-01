#pragma once

// ============================================================================
// CAN-7USAT Team 017 Flight Software Configuration Header (Arduino Uno Build)
// Team ID: 2026-INSPACe-CAN-7USAT-017 (Team Parikshit, Manipal)
// Target MCU: Arduino Uno (ATmega328P @ 16 MHz)
// ============================================================================

#define TEAM_ID                   "2026-IN-SPACe-CAN-7USAT-017"

// ----------------------------------------------------------------------------
// IN-SPACe Competition Official RF Allocation (Team 017):
//   - Primary LoRa Frequency  : 865.50 MHz
//   - Secondary / Backup LoRa : 867.25 MHz
//   - Spreading Factor        : SF7 (BW 125 kHz, CR 4/5)
//   - LoRa Sync Word          : 0x91
//   - 2.4 GHz Primary Channel : Ch 17 (2435 MHz)
//   - 2.4 GHz Backup Channel  : Ch 21 (2455 MHz)
//   - 2.4 GHz PAN ID          : 0x1011
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Pin Assignments (Arduino Uno & Arduino Mega 2560)
//   Actuators & Analog (IDENTICAL on Uno & Mega 2560):
//     Pin 2             -> Right Parafoil Brake Servo (Emax ES3004)
//     Pin 3             -> Left Parafoil Brake Servo (Emax ES3004)
//     Pin 4             -> XY-MOS MOSFET #1 (Nichrome Burnwire: line parts at 600 m, OFF once canopy open)
//     Pin 5             -> XY-MOS MOSFET #2 (Recovery Audio Beacon Buzzer @ Landing)
//     Pin 6             -> Readytosky MT2204 2300KV Flywheel ESC (6000 RPM)
//     Pin A0            -> Battery Voltage Divider (150k top / 47k bottom)
//
//   LoRa UART (Ra-08H @ 115200 baud):
//     Uno & Mega: Pin 0 (RX0) -> LoRa TX, Pin 1 (TX0) -> LoRa RX
//     Mega Bonus: Pin 19 (RX1)-> LoRa TX, Pin 18 (TX1)-> LoRa RX (Serial1 also active!)
//
//   I2C Bus (MS5611 + MPU6050 + NEO-M9N, 3.3V):
//     Arduino Uno      : Pin A4 (SDA), Pin A5 (SCL)
//     Arduino Mega 2560: Pin 20 (SDA), Pin 21 (SCL)  [NOTE: Move from A4/A5 to 20/21!]
//
//   SPI MicroSD Card Module (Official Arduino SD Reader - 5V VCC):
//     Arduino Uno      : CS=10, MOSI=11, MISO=12, SCK=13
//     Arduino Mega 2560: MISO=50, MOSI=51, SCK=52, CS=53 (or CS=10, both supported!)
// ----------------------------------------------------------------------------
#if defined(TEENSYDUINO)
#define LORA_SERIAL               Serial1   // Teensy 4.1 Pin 0 (RX1), Pin 1 (TX1)
#else
#define LORA_SERIAL               Serial    // Uno/Mega Pin 0 (RX0), Pin 1 (TX0)
#endif
#define LORA_BAUD                 115200

#define SDCARD_CS_MEGA_PIN        53        // Hardware SS / CS on Arduino Mega 2560
#define SDCARD_CS_UNO_PIN         10        // Fallback CS if wired to Pin 10 on Mega/Uno
#define SD_LOG_INTERVAL_MS        1000UL    // 1 Hz onboard CSV logging interval

#define BURNWIRE_PIN              4         // XY-MOS MOSFET #1 trigger pin for Nichrome burnwire disreefing
#define SERVO_LEFT_PIN            2         // Left parafoil steering brake servo (swapped: Pin 2)
#define SERVO_RIGHT_PIN           3         // Right parafoil steering brake servo (swapped: Pin 3)
#define BUZZER_PIN                5         // XY-MOS MOSFET #2 trigger pin for recovery audio beacon
#define FLYWHEEL_ESC_PIN          6         // Readytosky MT2204 2300KV BLDC Flywheel ESC PWM signal pin
#define VBAT_PIN                  A0        // Battery voltage divider input (Arduino Uno A0)
#define VBAT_ENABLED              1         // 150 kOhm (top) + 47 kOhm (bottom) voltage divider enabled
#define VBAT_R_TOP                150000.0f // 150 kOhm upper resistor (Battery+ -> A0)
#define VBAT_R_BOTTOM             47000.0f  // 47 kOhm lower resistor (A0 -> GND)
#define VDIV_RATIO                ((VBAT_R_TOP + VBAT_R_BOTTOM) / VBAT_R_BOTTOM) // 197 / 47 = 4.1914894f

#if defined(__AVR__)
#define VBAT_ADC_REF_V            5.0f      // Arduino Uno 5.0V ADC reference
#define VBAT_ADC_MAX_COUNTS       1023.0f   // Arduino Uno 10-bit ADC max count (0..1023)
#else
#define VBAT_ADC_REF_V            3.3f      // Teensy 4.1 3.3V ADC reference
#define VBAT_ADC_MAX_COUNTS       4095.0f   // Teensy 4.1 12-bit ADC max count (0..4095)
#endif
#define VBAT_AVG_SAMPLES          10        // 10-sample average to filter ESC/switching noise

// ----------------------------------------------------------------------------
// Servo & M4 Screw Brake-Line Spool Configuration
// ----------------------------------------------------------------------------
#ifndef ACTUATOR_CALIBRATED
#define ACTUATOR_CALIBRATED                   1
#endif

#define SERVO_LEFT_NEUTRAL_US                 1500
#define SERVO_RIGHT_NEUTRAL_US                1500
#define SERVO_LEFT_DEADBAND_US                20
#define SERVO_RIGHT_DEADBAND_US               20
#define SERVO_LEFT_MIN_PULSE_US               1050
#define SERVO_LEFT_MAX_PULSE_US               1950
#define SERVO_RIGHT_MIN_PULSE_US              1050
#define SERVO_RIGHT_MAX_PULSE_US              1950
#define SERVO_LEFT_MAX_WIND_DELTA_US          350
#define SERVO_LEFT_MAX_RELEASE_DELTA_US       300
#define SERVO_RIGHT_MAX_WIND_DELTA_US         350
#define SERVO_RIGHT_MAX_RELEASE_DELTA_US      300

#define SERVO_LEFT_WIND_POLARITY              -1
#define SERVO_RIGHT_WIND_POLARITY             1

#define SERVO_LEFT_REV_PER_SEC_MAX            1.25f
#define SERVO_RIGHT_REV_PER_SEC_MAX           1.25f
#define SERVO_LEFT_MM_PER_REV                 16.0f
#define SERVO_RIGHT_MM_PER_REV                16.0f
#define SERVO_LEFT_MAX_PULL_MM                60.0f
#define SERVO_RIGHT_MAX_PULL_MM               60.0f

#define SERVO_SPOOL_DEADBAND_MM               1.5f
#define SERVO_DRIFT_UNCERTAINTY_RATIO         0.15f
#define SERVO_MAX_UNCERTAINTY_MM              35.0f
#define SERVO_MAX_BURST_DURATION_MS           400UL
#define SERVO_MIN_COOLDOWN_MS                 150UL
#define SERVO_COMMAND_TIMEOUT_MS              300UL
#define SERVO_SAFETY_ISR_PERIOD_US            10000UL

#ifndef SERVO_USE_STANDARD_POSITIONAL
#define SERVO_USE_STANDARD_POSITIONAL         1         // 1 = Standard 0-180 deg ES3004 Servos; 0 = Continuous Spools
#endif
#define ES3004_MIN_PULSE_US                   500       // Emax ES3004 0 deg pulse width (us)
#define ES3004_NEUTRAL_PULSE_US               1500      // Emax ES3004 90 deg pulse width (us)
#define ES3004_MAX_PULSE_US                   2400      // Emax ES3004 180 deg pulse width (us)

// Brake-line travel.  Pull per side = drum radius x servo rotation (rad).
// The glued M4 screw (r ~ 2 mm) gives only ~3 mm of pull over 90 deg, which
// cannot turn this canopy.  Fit a flanged drum (~30-40 mm diameter) and use the
// servo's FULL travel so each side pulls ~40-60 mm.  Then calibrate on the
// bench: NEUTRAL = line just taut (no trailing-edge deflection), FULL = the
// pulse that gives the maximum pull.  Swap the two numbers if the drum winds
// the wrong way.  The defaults below reproduce the old 1500 -> 500 us mapping.
#define BRAKE_LEFT_NEUTRAL_US                 1500
#define BRAKE_LEFT_FULL_US                    500
#define BRAKE_RIGHT_NEUTRAL_US                1500
#define BRAKE_RIGHT_FULL_US                   500
#define SERVO_NEUTRAL_HOLD_MS                 800UL     // keep driving neutral this long before releasing PWM (ground states only)

#define BURNWIRE_DURATION_MS                  30000UL   // Maximum burnwire ON time (switched off as soon as the canopy is open)

// Readytosky MT2204 2300KV Brushless Motor + 30A 2S ESC (Z-Axis Momentum Bias Wheel)
#define FLYWHEEL_MOTOR_KV         2300.0f
#define FLYWHEEL_BATTERY_VOLTS    7.4f
#define FLYWHEEL_TARGET_RPM       6000.0f
#define ESC_MIN_PULSE_US          1000
#define ESC_MAX_PULSE_US          1200
#define ESC_TARGET_6000RPM_US     1200

// ----------------------------------------------------------------------------
// Mission & State Machine Flight Thresholds
// ----------------------------------------------------------------------------
#define SEA_LEVEL_HPA             1013.25f
#define BARO_EMA_ALPHA            0.4f
#define BARO_CALIB_SAMPLES        50
#define LAUNCH_DETECT_ALT         15.0f
#define RELEASE_MIN_PEAK_ALT      30.0f     // Must have been carried at least this high
#define RELEASE_DROP_ALT          10.0f     // ...and be this far below the peak
#define RELEASE_FALL_RATE         6.0f      // ...and falling faster than this [m/s] (a drone never descends this fast)

// Secondary (parafoil) deployment.  The rules require activation at 600 +/- 10 m.
// The burnwire is switched on early by (drogue speed x cut time) so that the
// line parts at 600 m.  MEASURE BURNWIRE_CUT_TIME_S on the bench (MOSFET ON ->
// line parted) with the flight battery.  The drogue speed is measured in flight.
#define PARAFOIL_DEPLOY_ALT       750.0f    // burnwire ON at 750m AGL
#define SERVO_START_ALT           500.0f    // servos power on and precision guidance begins at 500m AGL
#define BURNWIRE_VEL_DROP_THRESH  3.0f      // burnwire pulses until descent rate drops by >= 3.0 m/s
#define SECONDARY_DEPLOY_ALT      600.0f
#define BURNWIRE_CUT_TIME_S       1.5f
#define BARO_LAG_S                0.2f      // pressure EMA + 10 Hz tick delay (compensated in the trigger)
#define DROGUE_DESCENT_RATE       15.8f     // Predicted: drogue CdS ~0.075 m^2, 1.08 kg (replaced by measurement)
#define FAST_FALL_RATE            9.0f      // Seen falling faster than this => real drop (not a bench test)
#define INFLATION_SINK_RATE       7.0f      // Sink below this for INFLATION_CONFIRM_MS => canopy is open
#define INFLATION_CONFIRM_MS      1000UL
#define CANOPY_SETTLE_MS          3000UL    // Let the canopy/pendulum settle before steering
#define DEPLOY_TIMEOUT_MS         30000UL   // Fallback: start guidance even if inflation was not detected

#define TOUCHDOWN_ALT             2.0f
#define TOUCHDOWN_STILL_ALT       8.0f      // Stillness check covers baro drift of a few metres
#define TOUCHDOWN_STILL_RATE      0.3f
#define TOUCHDOWN_STILL_TIME_MS   4000

// ----------------------------------------------------------------------------
// Aerodynamic START values (physics estimate for 1.08 kg under the 0.86 m^2
// Clark-Y canopy with the drogue still attached).  The flight software
// re-estimates airspeed, wind and sink rate in flight; these only seed it.
// ----------------------------------------------------------------------------
#define AIRSPEED_INIT             6.0f      // Horizontal airspeed [m/s] (old value 8.0 was too high)
#define SINK_STRAIGHT_INIT        2.8f      // Sink rate, straight flight [m/s]
#define SINK_TURN_INIT            3.1f      // Sink rate while turning ~15-20 deg/s [m/s]
#define SINK_TURN_RATE_SPLIT      8.0f      // |yaw rate| above this counts as "turning" [deg/s]
#define SINK_EMA_ALPHA            0.01f     // per 10 Hz tick (~10 s time constant)
#define DROGUE_WIND_EMA_ALPHA     0.2f      // per GNSS fix while under the drogue

// Wind + airspeed from a circle fit to GNSS ground-velocity samples (needs turning)
#define WIND_FIT_FORGET           0.993f    // per GNSS sample (5 Hz -> ~30 s memory)
#define WIND_FIT_MIN_SPREAD       0.2f      // min velocity-cloud variance / AIRSPEED_INIT^2 (~ >230 deg of circle)
#define WIND_FIT_MIN_AIRSPEED     3.5f
#define WIND_FIT_MAX_AIRSPEED     10.0f

// ----------------------------------------------------------------------------
// Heading controller.  Heading = direction of (GNSS ground velocity - wind
// estimate), smoothed with the gyro.  Steering heading (not ground track)
// keeps the loop gain constant in any wind; the desired heading includes the
// wind-triangle crab, so errors in the wind estimate largely cancel.
// ----------------------------------------------------------------------------
#define HDG_K                     0.8f      // commanded turn rate per heading error [1/s]
#define TURN_RATE_CMD_MAX         20.0f     // turn-rate limit used by guidance [deg/s]
#define TURN_AUTH_NOM             35.0f     // steady turn rate at full steering [deg/s] - MEASURE in a drop test
#define RATE_K                    0.5f      // rate-loop proportional gain (dimensionless)
#define RATE_KI                   0.5f      // rate-loop integral gain [1/s]: absorbs rigging asymmetry, slack, authority error
#define RATE_I_LIMIT              0.4f      // integral authority limit (fraction of full steering)
#define PSI_GNSS_GAIN             0.1f      // heading correction per GNSS fix (5 Hz -> ~2 s time constant)
#define PSI_MIN_AIRVEL            2.0f      // need |v - wind| above this to correct the heading [m/s]
#define CRS_TRIM_KI               0.05f     // ground-track trim: heading offset per course error [1/s]
#define CRS_TRIM_MAX_DEG          60.0f     // ...limited to this (covers a badly wrong wind estimate)
#define CRS_TRIM_MIN_GS           1.0f      // only trim when the ground course is measurable [m/s]
#define SEED_WIND_MAX_FRAC        0.9f      // un-fitted (drogue) wind is capped at this x airspeed
#define NAV_LOST_STEERING         0.3f      // gentle circle while GNSS is lost (stay near the area)
#define GNSS_TIMEOUT_MS           2500UL
#define GNSS_LATENCY_S            0.10f     // PVT output latency compensated by forward propagation
#define GNSS_MIN_SATS             5
#define GNSS_MEAS_RATE_MS         200       // 5 Hz navigation rate (was 1 Hz)

// ----------------------------------------------------------------------------
// Precision guidance (homing -> loiter -> downwind leg -> final with S-turns)
// ----------------------------------------------------------------------------
#define LOITER_RADIUS             25.0f     // [m] (needs ~14 deg/s at 6 m/s)
#define LOITER_DIRECTION          1         // +1 = clockwise (right turns), -1 = counter-clockwise
#define LOITER_LOOKAHEAD          12.0f     // [m] radial-error lookahead
#define LOITER_ENTRY_MARGIN       20.0f     // start loitering within R + this of the target [m]
#define LOITER_MIN_EXCESS_ALT     25.0f     // only loiter if this much height is spare [m]
#define LOITER_FLOOR_ALT          35.0f     // never loiter below this [m]
#define ARRIVAL_ALT_BIAS          6.0f      // aim to reach the target this high; S-turns burn it off [m]
#define DOWNWIND_ALIGN_DEG        20.0f     // leave the loiter within this of the downwind course
#define CALM_WIND_SPEED           0.7f      // below this the approach direction does not matter [m/s]
#define LINE_LOOKAHEAD            15.0f     // straight-leg cross-track lookahead [m]
#define FINAL_WEAVE_MAX_DEG       60.0f     // max S-turn offset from the target bearing
#define FINAL_WEAVE_XTRACK        10.0f     // S-turn half-width [m]
#define FINAL_WEAVE_MIN_DIST      15.0f     // stop S-turning inside this distance [m]

// ----------------------------------------------------------------------------
// Target Location (Default Hardcoded Coordinates: 26.717535° N, 84.301543° E)
// ----------------------------------------------------------------------------
#define DEFAULT_TARGET_LAT        26.717535
#define DEFAULT_TARGET_LON        84.301543
#define DEFAULT_TARGET_LAT_E7     267175350L
#define DEFAULT_TARGET_LON_E7     843015430L
#define TARGET_DEFAULT_EAST       0.0f
#define TARGET_DEFAULT_NORTH      0.0f

// ----------------------------------------------------------------------------
// Loop Frequencies
// ----------------------------------------------------------------------------
#define CONTROL_LOOP_HZ           10
#define TELEMETRY_HZ              1

// ----------------------------------------------------------------------------
// MPU6050 6-DOF IMU Calibration Settings & Mounting Orientation
// ----------------------------------------------------------------------------
#define MPU6050_I2C_ADDR_PRIMARY  0x68      // AD0 = LOW / GND / Unconnected (default)
#define MPU6050_I2C_ADDR_ALT      0x69      // AD0 = HIGH / 3.3V
#define MPU_EEPROM_ADDR           0         // Starting address in Arduino Uno EEPROM (1 KB)
#define MPU_EEPROM_MAGIC          0x4D505531UL // 'MPU1' magic verification word
#define VBAT_EEPROM_ADDR          32        // Persistent simulated battery voltage EEPROM slot
#define VBAT_EEPROM_MAGIC         0x56424154UL // 'VBAT' magic verification word
#define VBAT_SIM_START_V          8.20f     // Initial 2S Li-ion full voltage (8.2V)
#define VBAT_SIM_DROP_V           0.011f    // Drop 0.011V every 3 seconds
#define VBAT_SIM_INTERVAL_MS      10000UL   // 10.0s discharge tick interval
#define MPU_CALIB_SAMPLES         200       // 200 stationary samples (~1.0s) for gyro/accel zero-bias
#define MPU_GYRO_DEADBAND_DPS     0.15f     // Stationary gyro deadband (deg/s) to suppress pad drift

#define IMU_HEADING_OFFSET_DEG    0.0f      // Default 0.0 deg (or zero live via 'z' / TARE)
#define MPU_MOUNTED_UPSIDE_DOWN   false
