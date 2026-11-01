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
//     Pin 4             -> XY-MOS MOSFET #1 (Nichrome Burnwire @ 700m, 30.0s)
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
#define SERVO_NEUTRAL_ANGLE                   90.0f     // 90 deg = No brake applied (1500 us)
#define SERVO_MAX_BRAKE_ANGLE                 0.0f      // 0 deg  = Full brake pulled (500 us)
#define ES3004_MIN_PULSE_US                   500       // Emax ES3004 0 deg pulse width (us)
#define ES3004_NEUTRAL_PULSE_US               1500      // Emax ES3004 90 deg pulse width (us)
#define ES3004_MAX_PULSE_US                   2400      // Emax ES3004 180 deg pulse width (us)
#define BURNWIRE_DURATION_MS                  30000UL   // 30.0s XY-MOS pulse to melt nichrome line

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
#define MIN_APOGEE_ALT            750.0f
#define APOGEE_DROP_THRESH        20.0f
#define PARAFOIL_DEPLOY_ALT       750.0f    // Burnwire deployment trigger altitude (750m AGL)
#define SERVO_START_ALT           500.0f    // Servos start up & precision guidance begins (500m AGL)
#define BURNWIRE_VEL_DROP_THRESH  3.0f      // Burnwire pulses until descent velocity decreases by 3.0 m/s
#define BURNWIRE_MAX_DURATION_MS  30000UL   // 30.0s max thermal safety pulse cutoff
#define BURNWIRE_DURATION_MS      BURNWIRE_MAX_DURATION_MS
#define FINAL_APPROACH_ALT        120.0f
#define FINAL_APPROACH_RADIUS     100.0f
#define TERMINAL_MARGIN           100.0f
#define TERMINAL_CAPTURE_RADIUS   200.0f
#define LOITER_RADIUS             30.0f
#define LOITER_CAPTURE_TOL        5.0f
#define BREAKOUT_ALT_MARGIN       2.0f
#define TOUCHDOWN_ALT             2.0f
#define TOUCHDOWN_STILL_TIME_MS   4000

// ----------------------------------------------------------------------------
// Aerodynamic Model (Frozen Baseline - Candidate A)
// ----------------------------------------------------------------------------
#define FORWARD_AIRSPEED          8.0f
#define PARAFOIL_DESCENT_RATE     2.5f
#define DROGUE_DESCENT_RATE       17.0f
#define MAX_TURN_RATE_DEG_S       30.0f

// ----------------------------------------------------------------------------
// Guidance & Control (PID in Radian Domain)
// ----------------------------------------------------------------------------
#define PID_KP                    10.0f
#define PID_KI                    0.1f
#define PID_KD                    0.5f
#define PID_INTEGRAL_LIMIT        10.0f
#define WIND_EMA_ALPHA            0.05f

// ----------------------------------------------------------------------------
// Target Location (Default Hardcoded Coordinates: 26.717535° N, 84.301543° E)
// ----------------------------------------------------------------------------
#define DEFAULT_TARGET_LAT        26.717535
#define DEFAULT_TARGET_LON        84.301543
#define TARGET_DEFAULT_EAST       1200.0f
#define TARGET_DEFAULT_NORTH      0.0f
#define NUM_WAYPOINTS             10
#define WAYPOINT_ACCEPTANCE_RAD   25.0f

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
