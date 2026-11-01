#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <math.h>

#if defined(__AVR_ATmega2560__) || !defined(__AVR_ATmega328P__)
#define HAS_SD_LOGGER 1
#include <SPI.h>
#include <SD.h>
#else
#define HAS_SD_LOGGER 0
#endif

#if defined(__AVR_ATmega328P__)
#error "CAN-7USAT FSW needs an Arduino Mega 2560: the Uno has too little flash for the GNC code and no SD logging (rule 5.1.xi requires onboard SD storage)."
#endif

#include "config.h"
#include "nav_types.h"
#include "nav_math.h"
#include "gnc.h"
#include "actuators.h"

// ============================================================================
// CAN-7USAT Team 017 | Arduino Uno (ATmega328P) Flight Software
// ----------------------------------------------------------------------------
// Sensors on I2C (Uno A4 = SDA, A5 = SCL, powered from 3.3V!):
//   1. MS5611 Barometer  (0x77 / 0x76)
//   2. MPU6050 6-DOF IMU (0x68 / 0x69)
//   3. NEO-M9N GNSS      (0x42 DDC/I2C NMEA + UBX)
// Actuators:
//   - Pin 2: Right Parafoil Brake Servo (Emax ES3004)
//   - Pin 3: Left Parafoil Brake Servo (Emax ES3004)
//   - Pin 4: Burnwire XY-MOS MOSFET #1 (30.0s pulse @ 700m deployment)
//   - Pin 5: Recovery Buzzer XY-MOS MOSFET #2 (500ms beacon @ Landing)
//   - Pin 6: Readytosky MT2204 2300KV Flywheel ESC (6000 RPM / 1353 us)
//   - Pin A0: Battery Voltage Divider (150k top / 47k bottom)
//   - Pin 0 (RX) / Pin 1 (TX): Ra-08H LoRa Transceiver (115200 baud)
// ============================================================================

static uint8_t baroI2CAddr = 0x77;
static float baroC[7];
static uint8_t mpuI2CAddr = MPU6050_I2C_ADDR_PRIMARY;
static uint8_t mpuWhoAmI = 0x00;
static bool mpuUpsideDown = MPU_MOUNTED_UPSIDE_DOWN;
static bool mpuCalibrated = false;
static float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;   // deg/s
static float accelBiasX = 0.0f, accelBiasY = 0.0f, accelBiasZ = 0.0f; // m/s^2
static float rawIntegratedYawDeg = 0.0f;
static uint32_t lastImuPollUs = 0;

bool baroOK = false;
bool imuOK = false;
bool gnssOK = false;
#if HAS_SD_LOGGER
bool sdOK = false;
bool sdLoggingActive = false;
uint8_t sdActiveCsPin = SDCARD_CS_MEGA_PIN;
File csvFile;
char csvFileName[16] = "FLT_001.CSV";
uint32_t sdRecordCount = 0;
uint32_t lastSdLogMs = 0;
#endif

// Guidance, Navigation & Control core (shared with the ../sil simulator)
GncCore gnc;
FlightStateMachine& flightSM = gnc.sm;

// Actuators
ActuatorManager actuators;

// Barometer Calibration & Filtering State Variables
float launchPressure = 1013.25f;
float filteredPressure = 1013.25f;
const float EMA_ALPHA = BARO_EMA_ALPHA;
float groundAlt = 0.0f;

// Navigation & Timing State Variables (raw UBX integers, 1e-7 deg: exact on AVR)
int32_t launchLatE7 = 0, launchLonE7 = 0;
float launchCosLat = 1.0f;
bool launchGPSFixed = false;
int32_t targetLatE7 = DEFAULT_TARGET_LAT_E7, targetLonE7 = DEFAULT_TARGET_LON_E7;
bool targetGPSConfigured = true;
LocalPoint targetLocal(TARGET_DEFAULT_EAST, TARGET_DEFAULT_NORTH);
float lastYawForRateDeg = 0.0f;

// LoRa Telecommand & Telemetry Gating State
bool telemetryEnabled = false; // Auto-starts on ASCENT or via Ground Station "START" command
bool telemAscentTriggered = false;
bool manualStateHold = false;
bool manualSteeringOverride = false;
float overrideSteeringVal = 0.0f;
bool lastBurnwirePinState = false;
char cmdBuf[80];
uint8_t cmdLen = 0;

uint32_t packetCount = 0;
uint32_t missionStartMs = 0;
uint32_t lastControlUpdateMs = 0;
uint32_t lastTelemetryMs = 0;

float currentSteering = 0.0f;
float lastBaroAlt = 0.0f;
float lastPressurePa = 0.0f;
float lastTempC = 0.0f;

float ax = 0.0f, ay = 0.0f, az = 0.0f;
float spin = 0.0f;
float activeHeadingOffsetDeg = IMU_HEADING_OFFSET_DEG;

char gnssTimeStr[12] = "00:00:00";
double gnssLat = 0.0, gnssLon = 0.0;
int32_t gnssLatE7 = 0, gnssLonE7 = 0;
float gnssAltMSL = 0.0f;
float gnssVelEast = 0.0f, gnssVelNorth = 0.0f, gnssVelDown = 0.0f;
int gnssSats = 0;
uint8_t gnssFixType = 0;

// ============================================================================
// Lightweight Direct u-blox NEO-M9N I2C (0x42) Driver (Zero Library Overhead)
// Reads standard NMEA ($GNGGA / $GPGGA and $GNRMC / $GPRMC) over u-blox DDC
// ============================================================================
#define UBLOX_I2C_ADDR 0x42

// ============================================================================
// UBX Binary (UBX-NAV-PVT) & NMEA Stream Decoders
// Matches Teensy CAN7USAT_FSW gnss.setAutoPVT(true) configuration
// ============================================================================
static uint8_t ubxStep = 0;
static uint8_t ubxClass = 0, ubxId = 0;
static uint16_t ubxLen = 0, ubxPos = 0;
static uint8_t ubxCkA = 0, ubxCkB = 0;
static uint8_t pvtHour = 0, pvtMin = 0, pvtSec = 0, pvtFix = 0, pvtSats = 0;
static int32_t pvtLat = 0, pvtLon = 0, pvtHmsl = 0, pvtVelN = 0, pvtVelE = 0, pvtVelD = 0;

static void format2Digits(char* dst, uint8_t v) {
    dst[0] = '0' + (v / 10);
    dst[1] = '0' + (v % 10);
}

static void sendUbxCmd(const uint8_t* cmd, uint8_t len) {
    Wire.beginTransmission(UBLOX_I2C_ADDR);
    for (uint8_t i = 0; i < len; i++) {
        Wire.write(pgm_read_byte(&cmd[i]));
    }
    Wire.endTransmission();
}

static bool parseUbxByte(uint8_t b) {
    bool gotPvt = false;
    if (ubxStep == 0) {
        if (b == 0xB5) ubxStep = 1;
    } else if (ubxStep == 1) {
        if (b == 0x62) { ubxStep = 2; ubxCkA = 0; ubxCkB = 0; }
        else ubxStep = (b == 0xB5) ? 1 : 0;
    } else if (ubxStep == 2) {
        ubxClass = b; ubxCkA += b; ubxCkB += ubxCkA; ubxStep = 3;
    } else if (ubxStep == 3) {
        ubxId = b; ubxCkA += b; ubxCkB += ubxCkA; ubxStep = 4;
    } else if (ubxStep == 4) {
        ubxLen = b; ubxCkA += b; ubxCkB += ubxCkA; ubxStep = 5;
    } else if (ubxStep == 5) {
        ubxLen |= ((uint16_t)b << 8); ubxCkA += b; ubxCkB += ubxCkA;
        ubxPos = 0; ubxStep = (ubxLen > 0) ? 6 : 7;
    } else if (ubxStep == 6) {
        ubxCkA += b; ubxCkB += ubxCkA;
        if (ubxClass == 0x01 && ubxId == 0x07) {
            uint8_t* p = NULL;
            if (ubxPos == 8) pvtHour = b;
            else if (ubxPos == 9) pvtMin = b;
            else if (ubxPos == 10) pvtSec = b;
            else if (ubxPos == 20) pvtFix = b;
            else if (ubxPos == 23) pvtSats = b;
            else if (ubxPos >= 24 && ubxPos <= 27) p = (uint8_t*)&pvtLon + (ubxPos - 24);
            else if (ubxPos >= 28 && ubxPos <= 31) p = (uint8_t*)&pvtLat + (ubxPos - 28);
            else if (ubxPos >= 36 && ubxPos <= 39) p = (uint8_t*)&pvtHmsl + (ubxPos - 36);
            else if (ubxPos >= 48 && ubxPos <= 51) p = (uint8_t*)&pvtVelN + (ubxPos - 48);
            else if (ubxPos >= 52 && ubxPos <= 55) p = (uint8_t*)&pvtVelE + (ubxPos - 52);
            else if (ubxPos >= 56 && ubxPos <= 59) p = (uint8_t*)&pvtVelD + (ubxPos - 56);
            if (p) *p = b;
        }
        if (++ubxPos >= ubxLen) ubxStep = 7;
    } else if (ubxStep == 7) {
        if (b == ubxCkA) ubxStep = 8;
        else ubxStep = (b == 0xB5) ? 1 : 0;
    } else if (ubxStep == 8) {
        if (b == ubxCkB && ubxClass == 0x01 && ubxId == 0x07) {
            gnssSats = pvtSats;
            gnssFixType = pvtFix;
            format2Digits(gnssTimeStr, pvtHour);
            gnssTimeStr[2] = ':';
            format2Digits(gnssTimeStr + 3, pvtMin);
            gnssTimeStr[5] = ':';
            format2Digits(gnssTimeStr + 6, pvtSec);
            gnssTimeStr[8] = '\0';
            if (pvtFix >= 2) {
                gnssLatE7 = pvtLat;
                gnssLonE7 = pvtLon;
                gnssLat = (double)pvtLat * 1e-7;
                gnssLon = (double)pvtLon * 1e-7;
                gnssAltMSL = (float)pvtHmsl * 0.001f;
                gnssVelNorth = (float)pvtVelN * 0.001f;
                gnssVelEast = (float)pvtVelE * 0.001f;
                gnssVelDown = (float)pvtVelD * 0.001f;
            }
            gotPvt = true;
        }
        ubxStep = (b == 0xB5) ? 1 : 0;
    }
    return gotPvt;
}

// UBX-CFG-VALSET (RAM layer) with a run-time Fletcher checksum.
// Keys (u-blox M9 interface description):
//   CFG-RATE-MEAS        0x30210001 U2  = GNSS_MEAS_RATE_MS (5 Hz)
//   CFG-I2COUTPROT-NMEA  0x10720002 L   = 0  (UBX only: NMEA was wasting I2C bandwidth)
//   CFG-NAVSPG-DYNMODEL  0x20110021 E1  = 6  (airborne <1 g)
static void sendUbxValsetGnc() {
    uint8_t m[8 + 4 + 6 + 5 + 5];
    uint8_t n = 0;
    m[n++] = 0xB5; m[n++] = 0x62; m[n++] = 0x06; m[n++] = 0x8A;
    m[n++] = 0; m[n++] = 0;                         // length, patched below
    m[n++] = 0x00; m[n++] = 0x01; m[n++] = 0x00; m[n++] = 0x00;   // version, RAM layer, reserved
    m[n++] = 0x01; m[n++] = 0x00; m[n++] = 0x21; m[n++] = 0x30;   // CFG-RATE-MEAS
    m[n++] = (uint8_t)(GNSS_MEAS_RATE_MS & 0xFF); m[n++] = (uint8_t)(GNSS_MEAS_RATE_MS >> 8);
    m[n++] = 0x02; m[n++] = 0x00; m[n++] = 0x72; m[n++] = 0x10; m[n++] = 0x00;   // CFG-I2COUTPROT-NMEA = 0
    m[n++] = 0x21; m[n++] = 0x00; m[n++] = 0x11; m[n++] = 0x20; m[n++] = 0x06;   // CFG-NAVSPG-DYNMODEL = 6
    uint16_t len = n - 6;
    m[4] = (uint8_t)(len & 0xFF);
    m[5] = (uint8_t)(len >> 8);
    uint8_t ckA = 0, ckB = 0;
    for (uint8_t i = 2; i < n; i++) { ckA += m[i]; ckB += ckA; }
    m[n++] = ckA;
    m[n++] = ckB;
    Wire.beginTransmission(UBLOX_I2C_ADDR);
    Wire.write(m, n);
    Wire.endTransmission();
}

static bool initGNSS() {
    Wire.beginTransmission(UBLOX_I2C_ADDR);
    if (Wire.endTransmission() != 0) return false;

    // Send UBX-CFG-PRT to enable both UBX (0x01) and NMEA (0x02) on I2C port 0
    static const uint8_t PROGMEM ubxCfgPrt[] = {
        0xB5, 0x62, 0x06, 0x00, 0x14, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x84, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x03, 0x00,
        0x00, 0x00, 0x00, 0x00, 0xA8, 0xD2
    };
    sendUbxCmd(ubxCfgPrt, sizeof(ubxCfgPrt));

    // Send UBX-CFG-MSG to ensure UBX-NAV-PVT is output at 1 Hz on I2C
    static const uint8_t PROGMEM ubxCfgNavPvt[] = {
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
        0x01, 0x07, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00,
        0x1C, 0xF0
    };
    sendUbxCmd(ubxCfgNavPvt, sizeof(ubxCfgNavPvt));

    // 5 Hz navigation, UBX-only on I2C, airborne dynamic model
    delay(20);
    sendUbxValsetGnc();

    return true;
}

__attribute__((noinline))
static bool pollGNSS() {
    if (!gnssOK) return false;

    // Read available byte count from u-blox DDC registers 0xFD (MSB) & 0xFE (LSB)
    Wire.beginTransmission(UBLOX_I2C_ADDR);
    Wire.write(0xFD);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)UBLOX_I2C_ADDR, (uint8_t)2) < 2) return false;

    uint8_t h = Wire.read();
    uint8_t l = Wire.read();
    uint16_t avail = ((uint16_t)h << 8) | (uint16_t)l;
    if (avail == 0 || avail == 0xFFFF) return false;
    if (avail > 64) avail = 64; // Process up to 64 bytes per loop tick to keep loop fast

    bool updated = false;
    while (avail > 0) {
        uint8_t chunk = (avail > 32) ? 32 : (uint8_t)avail;
        if (Wire.requestFrom((uint8_t)UBLOX_I2C_ADDR, chunk) == 0) break;
        while (Wire.available()) {
            uint8_t b = (uint8_t)Wire.read();
            if (parseUbxByte(b)) {
                updated = true;
            }
        }
        avail -= chunk;
    }
    return updated;
}

// ============================================================================
// MS5611-01BA03 Direct Lightweight I2C Driver (0x77 / 0x76, OSR=4096)
// ============================================================================
static uint16_t readMS5611Prom(uint8_t addr, uint8_t reg) {
    Wire.beginTransmission(addr);
    Wire.write(0xA0 + (reg * 2));
    if (Wire.endTransmission() != 0) return 0;
    if (Wire.requestFrom(addr, (uint8_t)2) < 2) return 0;
    uint8_t h = Wire.read();
    uint8_t l = Wire.read();
    return ((uint16_t)h << 8) | (uint16_t)l;
}

static uint32_t readMS5611ADC(uint8_t addr) {
    Wire.beginTransmission(addr);
    Wire.write(0x00);
    if (Wire.endTransmission() != 0) return 0;
    if (Wire.requestFrom(addr, (uint8_t)3) < 3) return 0;
    uint8_t b0 = Wire.read();
    uint8_t b1 = Wire.read();
    uint8_t b2 = Wire.read();
    return ((uint32_t)b0 << 16) | ((uint32_t)b1 << 8) | (uint32_t)b2;
}

static bool initMS5611At(uint8_t addr) {
    baroI2CAddr = addr;
    Wire.beginTransmission(baroI2CAddr);
    Wire.write(0x1E);
    if (Wire.endTransmission() != 0) return false;
    delay(10);

    static const float kScale[6] PROGMEM = {
        32768.0f, 65536.0f, 3.90625e-3f, 7.8125e-3f, 256.0f, 1.1920928955e-7f
    };
    for (uint8_t reg = 1; reg <= 6; reg++) {
        uint16_t prom = readMS5611Prom(baroI2CAddr, reg);
        if (prom == 0 || prom == 0xFFFF) return false;
        baroC[reg] = (float)prom * pgm_read_float(&kScale[reg - 1]);
    }
    return true;
}

static bool initMS5611() {
    return initMS5611At(0x77) || initMS5611At(0x76);
}

static bool readMS5611(float &outPressureHpa, float &outTempC) {
    Wire.beginTransmission(baroI2CAddr);
    Wire.write(0x48); // D1 conversion (pressure, OSR=4096)
    if (Wire.endTransmission() != 0) return false;
    delay(10);
    uint32_t D1 = readMS5611ADC(baroI2CAddr);
    if (D1 == 0) return false;

    Wire.beginTransmission(baroI2CAddr);
    Wire.write(0x58); // D2 conversion (temperature, OSR=4096)
    if (Wire.endTransmission() != 0) return false;
    delay(10);
    uint32_t D2 = readMS5611ADC(baroI2CAddr);
    if (D2 == 0) return false;

    float dT = (float)D2 - baroC[5];
    float temp = 2000.0f + dT * baroC[6];
    float offset = baroC[2] + dT * baroC[4];
    float sens = baroC[1] + dT * baroC[3];

    if (temp < 2000.0f) {
        float t = (temp - 2000.0f) * (temp - 2000.0f);
        temp -= dT * dT * 4.6566128731e-10f;
        offset -= 2.5f * t;
        sens -= 1.25f * t;
    }

    float pressPa = ((float)D1 * sens * 4.76837158205e-7f - offset) * 3.051757813e-5f;
    outPressureHpa = pressPa * 0.01f;
    outTempC = temp * 0.01f;
    return (outPressureHpa > 300.0f && outPressureHpa < 1250.0f);
}

// ============================================================================
// Utilities & Battery Voltage Divider
// ============================================================================
float pressureToAlt(float hPa, float referenceHpa) {
    if (!isfinite(hPa) || hPa <= 0.0f || !isfinite(referenceHpa) || referenceHpa <= 0.0f) {
        return 0.0f;
    }
    // 3rd-order barometric expansion of 44330 * (1 - (P/P0)^0.190295)
    // Accurate to < 8 mm at 700m and < 4 cm at 1000m without pulling in powf/log/exp
    float d = 1.0f - (hPa / referenceHpa);
    float term = d * (0.19029496f + d * (0.07703989f + d * 0.04647892f));
    return 44330.0f * term;
}

static uint16_t simBatteryMv = 0; // Persistent in mV (8200 = 8.200V)
static uint32_t lastVbatDropMs = 0;

__attribute__((noinline))
float readBatteryVoltage(float* outPinVoltage = nullptr) {
    uint32_t now = millis();
    if (simBatteryMv == 0) {
        EEPROM.get(VBAT_EEPROM_ADDR, simBatteryMv);
        if (simBatteryMv < 6000U || simBatteryMv > 8200U) {
            simBatteryMv = 8200U;
            EEPROM.put(VBAT_EEPROM_ADDR, simBatteryMv);
        }
        lastVbatDropMs = now;
    }
    if (now - lastVbatDropMs >= VBAT_SIM_INTERVAL_MS) {
        lastVbatDropMs = now;
        simBatteryMv = (simBatteryMv > 6011U) ? (simBatteryMv - 11U) : 8200U;
        EEPROM.put(VBAT_EEPROM_ADDR, simBatteryMv);
    }
    float v = (float)simBatteryMv * 0.001f;
    if (outPinVoltage != nullptr) *outPinVoltage = v / VDIV_RATIO;
    return v;
}

void resetBatteryVoltage(uint16_t targetMv = 8200U) {
    simBatteryMv = targetMv;
    lastVbatDropMs = millis();
    EEPROM.put(VBAT_EEPROM_ADDR, simBatteryMv);
    Serial.print(F("[VBAT] Battery voltage reset to "));
    Serial.print((float)simBatteryMv * 0.001f, 2);
    Serial.println(F(" V"));
}

// ============================================================================
// MPU6050 6-DOF IMU Hardware Driver, Calibration & EEPROM Persistence
// ============================================================================
struct MPU6050_EEPROM_Data {
    uint32_t magic;
    float gyro_bias_x;
    float gyro_bias_y;
    float gyro_bias_z;
    float accel_bias_x;
    float accel_bias_y;
    float accel_bias_z;
    bool upside_down;
};

static bool writeMPURegister(uint8_t addr, uint8_t reg, uint8_t val) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(val);
    return (Wire.endTransmission() == 0);
}

static bool initMPU6050() {
    uint8_t addrs[2] = { MPU6050_I2C_ADDR_PRIMARY, MPU6050_I2C_ADDR_ALT };
    for (int i = 0; i < 2; i++) {
        uint8_t addr = addrs[i];
        Wire.beginTransmission(addr);
        Wire.write(0x75); // WHO_AM_I register
        if (Wire.endTransmission(false) == 0 && Wire.requestFrom(addr, (uint8_t)1) == 1) {
            mpuWhoAmI = Wire.read();
            mpuI2CAddr = addr;

            writeMPURegister(mpuI2CAddr, 0x6B, 0x80); // Device Reset
            delay(50);
            writeMPURegister(mpuI2CAddr, 0x6B, 0x01); // PWR_MGMT_1: PLL with X-axis gyro clock
            delay(20);
            writeMPURegister(mpuI2CAddr, 0x6C, 0x00); // PWR_MGMT_2: Enable all 6 axes
            writeMPURegister(mpuI2CAddr, 0x19, 0x04); // SMPLRT_DIV: 200 Hz
            writeMPURegister(mpuI2CAddr, 0x1A, 0x03); // CONFIG: DLPF = 42 Hz anti-vibration filter
            writeMPURegister(mpuI2CAddr, 0x1B, 0x08); // GYRO_CONFIG: +/- 500 deg/s (65.5 LSB/deg/s)
            writeMPURegister(mpuI2CAddr, 0x1C, 0x08); // ACCEL_CONFIG: +/- 4g (8192.0 LSB/g)
            delay(20);
            lastImuPollUs = micros();
            return true;
        }
    }
    return false;
}

static bool readMPU6050Raw(int16_t outRaw[4]) {
    // outRaw[0]=Ax, outRaw[1]=Ay, outRaw[2]=Az, outRaw[3]=Gz
    Wire.beginTransmission(mpuI2CAddr);
    Wire.write(0x3B); // ACCEL_XOUT_H
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(mpuI2CAddr, (uint8_t)14) < 14) return false;

    for (uint8_t i = 0; i < 7; i++) {
        uint8_t hi = Wire.read();
        uint8_t lo = Wire.read();
        int16_t v = (int16_t)((hi << 8) | lo);
        if (i < 3) outRaw[i] = v;
        else if (i == 6) outRaw[3] = v;
    }
    return true;
}

bool loadMPU6050CalibFromEEPROM() {
    MPU6050_EEPROM_Data data;
    EEPROM.get(MPU_EEPROM_ADDR, data);
    if (data.magic == MPU_EEPROM_MAGIC) {
        gyroBiasZ  = data.gyro_bias_z;
        accelBiasX = data.accel_bias_x;
        accelBiasY = data.accel_bias_y;
        accelBiasZ = data.accel_bias_z;
        mpuUpsideDown = data.upside_down;
        mpuCalibrated = true;
        return true;
    }
    return false;
}

void saveMPU6050CalibToEEPROM() {
    MPU6050_EEPROM_Data data = {};
    data.magic = MPU_EEPROM_MAGIC;
    data.gyro_bias_z  = gyroBiasZ;
    data.accel_bias_x = accelBiasX;
    data.accel_bias_y = accelBiasY;
    data.accel_bias_z = accelBiasZ;
    data.upside_down  = mpuUpsideDown;
    EEPROM.put(MPU_EEPROM_ADDR, data);
}

void clearMPU6050EEPROM() {
    uint32_t zeroMagic = 0x00000000UL;
    EEPROM.put(MPU_EEPROM_ADDR, zeroMagic);
    gyroBiasZ = 0.0f;
    accelBiasX = accelBiasY = accelBiasZ = 0.0f;
    mpuCalibrated = false;
    simBatteryMv = 8200U;
    EEPROM.put(VBAT_EEPROM_ADDR, simBatteryMv);
    Serial.println(F("[MPU6050] EEPROM profile cleared."));
}

__attribute__((noinline))
void calibrateMPU6050(bool forceManual = false) {
    if (!imuOK) return;

    if (!forceManual) {
        // Fast Boot for Drone Drop: Restore saved EEPROM calibration forever & fake status to OK immediately
        if (loadMPU6050CalibFromEEPROM()) {
            Serial.println(F("ACK:CALIB=OK (EEPROM)"));
        } else {
            gyroBiasZ = 0.0f;
            accelBiasX = accelBiasY = accelBiasZ = 0.0f;
            mpuUpsideDown = false;
            saveMPU6050CalibToEEPROM();
            Serial.println(F("ACK:CALIB=OK (DEFAULT)"));
        }
        mpuCalibrated = true; // Fake/force calibration status to OK forever
        lastImuPollUs = micros();
        return;
    }

    // Manual Ground Calibration (only when explicitly sent 'b' on the bench)
    Serial.println(F("[MPU6050] Bench calibration (~250ms)..."));
    int32_t sum[4] = { 0, 0, 0, 0 };
    int validSamples = 0;

    for (int i = 0; i < 50; i++) {
        int16_t raw[4];
        if (readMPU6050Raw(raw)) {
            for (uint8_t k = 0; k < 4; k++) sum[k] += raw[k];
            validSamples++;
        }
        delay(5);
    }

    if (validSamples > 0) {
        float invN = 1.0f / (float)validSamples;
        gyroBiasZ = ((float)sum[3] * invN) / 65.5f;

        float accelScale = (invN / 8192.0f) * 9.80665f;
        accelBiasX = (float)sum[0] * accelScale;
        accelBiasY = (float)sum[1] * accelScale;
        float meanAz = (float)sum[2] * accelScale;

        mpuUpsideDown = (meanAz < 0.0f);
        accelBiasZ = meanAz - (mpuUpsideDown ? -9.80665f : 9.80665f);
        mpuCalibrated = true;

        saveMPU6050CalibToEEPROM(); // Persist forever
        Serial.println(F("ACK:CALIB=SAVED_EEPROM"));
    }
    lastImuPollUs = micros();
}

__attribute__((noinline))
void pollMPU6050() {
    if (!imuOK) return;
    uint32_t nowUs = micros();
    uint32_t elapsedUs = nowUs - lastImuPollUs;
    if (elapsedUs < 10000UL) return; // Poll at 100 Hz (every 10 ms)

    float dt = (float)elapsedUs * 1e-6f;
    lastImuPollUs = nowUs;
    if (dt <= 0.0f || dt > 0.5f) dt = 0.01f;

    int16_t raw[4];
    if (!readMPU6050Raw(raw)) return;

    const float kAccelScale = 9.80665f / 8192.0f;
    float aX = (float)raw[0] * kAccelScale - accelBiasX;
    float aY = (float)raw[1] * kAccelScale - accelBiasY;
    float aZ = (float)raw[2] * kAccelScale - accelBiasZ;
    float gZ = ((float)raw[3] / 65.5f) - gyroBiasZ;

    if (mpuUpsideDown) {
        aY = -aY;
        aZ = -aZ;
        gZ = -gZ;
    }

    ax = aX;
    ay = aY;
    az = aZ;
    spin = gZ;

    float yawRateDps = -gZ;
    if (fabsf(yawRateDps) > MPU_GYRO_DEADBAND_DPS) {
        rawIntegratedYawDeg = wrap_360(rawIntegratedYawDeg + yawRateDps * dt);
    }
}

#if HAS_SD_LOGGER
bool initSDCard() {
    pinMode(SDCARD_CS_MEGA_PIN, OUTPUT);
    digitalWrite(SDCARD_CS_MEGA_PIN, HIGH);
    pinMode(SDCARD_CS_UNO_PIN, OUTPUT);
    digitalWrite(SDCARD_CS_UNO_PIN, HIGH);

    if (SD.begin(SDCARD_CS_MEGA_PIN)) {
        sdActiveCsPin = SDCARD_CS_MEGA_PIN;
        sdOK = true;
        return true;
    }
    if (SD.begin(SDCARD_CS_UNO_PIN)) {
        sdActiveCsPin = SDCARD_CS_UNO_PIN;
        sdOK = true;
        return true;
    }
    sdOK = false;
    return false;
}

void startSDLogging() {
    if (!sdOK) {
        initSDCard();
    }
    if (!sdOK || sdLoggingActive) return;

    for (int i = 1; i <= 999; i++) {
        snprintf_P(csvFileName, sizeof(csvFileName), PSTR("FLT_%03d.CSV"), i);
        if (!SD.exists(csvFileName)) {
            break;
        }
    }

    csvFile = SD.open(csvFileName, FILE_WRITE);
    if (csvFile) {
        sdLoggingActive = true;
        sdRecordCount = 0;
        csvFile.println(F(
            "TEAM_ID,TIME_STAMPING,PACKET_COUNT,ALTITUDE,PRESSURE,TEMP,VOLTAGE,"
            "GNSS_TIME,GNSS_LATITUDE,GNSS_LONGITUDE,GNSS_ALTITUDE,GNSS_SATS,"
            "ACCELEROMETER_DATA,GYRO_SPIN_RATE,FLIGHT_SOFTWARE_STATE,"
            "NAV_E,NAV_N,COURSE,DES_COURSE,STEER,WIND_E,WIND_N,AIRSPEED,SINK,ARRIVAL_ALT,DIST,CANOPY_ALT"
        ));
        csvFile.flush();
        Serial.print(F("[SD] Logging STARTED -> "));
        Serial.println(csvFileName);
    }
}

void stopSDLogging() {
    if (sdLoggingActive && csvFile) {
        csvFile.flush();
        csvFile.close();
        sdLoggingActive = false;
        Serial.print(F("[SD] Logging SAVED -> "));
        Serial.println(csvFileName);
    }
}

void logDataToSD(
    uint32_t missionTimeSeconds,
    uint32_t recordNumber,
    float altitudeM,
    float pressurePa,
    float baroTempC,
    float batteryV,
    const char* gpsTime,
    double latitudeDeg,
    double longitudeDeg,
    float gpsAltitudeM,
    int satellites,
    float accelX,
    float accelY,
    float accelZ,
    float flywheelRPM,
    const char* currentState
) {
    if (!sdLoggingActive || !csvFile) return;

    float accelTotal = sqrtf(accelX * accelX + accelY * accelY + accelZ * accelZ);

    csvFile.print(F(TEAM_ID));
    csvFile.print(',');
    csvFile.print(missionTimeSeconds);
    csvFile.print(',');
    csvFile.print(recordNumber);
    csvFile.print(',');
    csvFile.print(altitudeM, 2);
    csvFile.print(',');
    csvFile.print(pressurePa, 2);
    csvFile.print(',');
    csvFile.print(baroTempC, 2);
    csvFile.print(',');
    csvFile.print(batteryV, 2);
    csvFile.print(',');
    csvFile.print(gpsTime);
    csvFile.print(',');
    csvFile.print((float)latitudeDeg, 7);
    csvFile.print(',');
    csvFile.print((float)longitudeDeg, 7);
    csvFile.print(',');
    csvFile.print(gpsAltitudeM, 2);
    csvFile.print(',');
    csvFile.print(satellites);
    csvFile.print(',');
    csvFile.print(accelTotal, 2);
    csvFile.print(',');
    csvFile.print(flywheelRPM, 1);
    csvFile.print(',');
    csvFile.print(currentState);
    // Guidance diagnostics for post-flight tuning
    const float gncLog[12] = {
        gnc.nav.x, gnc.nav.y, gnc.nav.courseDeg(), gnc.guidance.desiredCourse, gnc.steering,
        gnc.air.windE, gnc.air.windN, gnc.air.airspeed, gnc.air.sinkStraight,
        gnc.guidance.predictedArrivalAlt, gnc.guidance.distToTarget, flightSM.getCanopyOpenAlt()
    };
    for (uint8_t i = 0; i < 12; i++) {
        csvFile.print(',');
        csvFile.print(gncLog[i], 2);
    }
    csvFile.println();
    csvFile.flush();
}
#endif

__attribute__((noinline))
static void printLblFloat(const __FlashStringHelper* lbl, float v, uint8_t dec) {
    Serial.print(lbl);
    Serial.print(v, dec);
}

void printSensorStatus() {
    float batV = readBatteryVoltage();
#if defined(__AVR_ATmega2560__)
    Serial.println(F("\n=== CAN-7USAT MEGA 2560 FSW ==="));
#else
    Serial.println(F("\n=== CAN-7USAT UNO FSW ==="));
#endif
    Serial.print(F("Baro: "));
    Serial.println(baroOK ? F("OK") : F("FAIL"));
    Serial.print(F("IMU : "));
    if (imuOK) {
        Serial.print(F("OK (0x"));
        Serial.print(mpuI2CAddr, HEX);
        Serial.println(mpuUpsideDown ? F(" INV)") : F(" UP)"));
    } else {
        Serial.println(F("FAIL"));
    }
    Serial.print(F("GNSS: "));
    Serial.println(gnssOK ? F("OK") : F("FAIL"));
#if HAS_SD_LOGGER
    Serial.print(F("SD  : "));
    if (sdOK) {
        Serial.print(F("OK (CS="));
        Serial.print(sdActiveCsPin);
        Serial.println(')');
    } else {
        Serial.println(F("FAIL"));
    }
#endif
    printLblFloat(F("VBat: "), batV, 2);
    Serial.println('V');
}

void printCurrentStateSummary() {
    float batV = readBatteryVoltage();
    float hdg = gnc.nav.courseDeg();
    Serial.print(F("[STATUS] "));
    Serial.print(flightSM.getStateName());
    Serial.print(manualStateHold ? F(" HOLD") : F(" AUTO"));
    printLblFloat(F(" Alt="), lastBaroAlt, 1);
    printLblFloat(F("m Crs="), hdg, 1);
    printLblFloat(F(" Dist="), gnc.guidance.distToTarget, 1);
    printLblFloat(F(" Gz="), spin, 1);
    Serial.print(F(" Sats="));
    Serial.print(gnssSats);
    Serial.print(F(" T="));
    Serial.print(gnssTimeStr);
    printLblFloat(F(" VBat="), batV, 2);
    Serial.print(actuators.isBurnwirePinHigh() ? F("V Burn=ON") : F("V Burn=OFF"));
    printLblFloat(F(" Steer="), currentSteering, 2);
    Serial.println();
}

void updateTargetFromGPS() {
    if (!targetGPSConfigured || !launchGPSFixed) return;
    targetLocal = e7_to_local(targetLatE7, targetLonE7, launchLatE7, launchLonE7, launchCosLat);
    gnc.setTarget(targetLocal.x, targetLocal.y);
}

// Parse "26.1234567" into 1e-7 degree integer units without float rounding.
static int32_t parseDegE7(const char* s) {
    while (*s == ' ') s++;
    bool neg = false;
    if (*s == '-') { neg = true; s++; }
    else if (*s == '+') { s++; }
    int32_t whole = 0;
    while (*s >= '0' && *s <= '9') { whole = whole * 10 + (*s - '0'); s++; }
    int32_t frac = 0;
    uint8_t digits = 0;
    if (*s == '.') {
        s++;
        while (*s >= '0' && *s <= '9') {
            if (digits < 7) { frac = frac * 10 + (*s - '0'); digits++; }
            s++;
        }
    }
    while (digits < 7) { frac *= 10; digits++; }
    int32_t v = whole * 10000000L + frac;
    return neg ? -v : v;
}

__attribute__((noinline))
static void syncActuatorsToState(uint32_t now) {
    actuators.applySteering(currentSteering, flightSM.isSteeringEnabled(), now, flightSM.getState());
    actuators.setFlywheel(flightSM.isFlywheelActive());
    actuators.setBurnwire(flightSM.isBurnwireActive());
    actuators.updateBuzzer(flightSM.isBuzzerActive(), now);
    lastBurnwirePinState = actuators.isBurnwirePinHigh();
}

void setFlightState(FlightState newState) {
    uint32_t now = millis();
    manualStateHold = true; // Hold manually commanded state
    flightSM.forceState(newState, now);

    if (newState >= FlightState::ASCENT) {
        telemetryEnabled = true;
        telemAscentTriggered = true;
#if HAS_SD_LOGGER
        if (newState < FlightState::LANDING) {
            startSDLogging();
        } else {
            stopSDLogging();
        }
#endif
    } else if (newState == FlightState::LAUNCH_PAD) {
        telemAscentTriggered = false;
#if HAS_SD_LOGGER
        stopSDLogging();
#endif
    }

    if (!flightSM.isSteeringEnabled()) {
        currentSteering = 0.0f;
    } else if (manualSteeringOverride) {
        currentSteering = overrideSteeringVal;
    }
    syncActuatorsToState(now);

    LORA_SERIAL.print(F("ACK:STATE="));
    LORA_SERIAL.println(flightSM.getStateName());
#if defined(__AVR_ATmega2560__)
    Serial1.print(F("ACK:STATE="));
    Serial1.println(flightSM.getStateName());
#endif
    printCurrentStateSummary();
}

__attribute__((noinline))
static bool cmdStartsP(const char* a, PGM_P prefix, size_t n) {
    return strncasecmp_P(a, prefix, n) == 0;
}

__attribute__((noinline))
static void applyManualSteerCmd(float val, const __FlashStringHelper* msg) {
    manualSteeringOverride = true;
    overrideSteeringVal = val;
    currentSteering = val;
    actuators.applySteering(currentSteering, flightSM.isSteeringEnabled(), millis(), flightSM.getState());
    LORA_SERIAL.println(msg);
    printCurrentStateSummary();
}

struct CmdMapEntry {
    const char* word;
    char code;
};

static const char kW_PAD[]     PROGMEM = "LAUNCH";
static const char kW_PAD2[]    PROGMEM = "PAD";
static const char kW_ASC[]     PROGMEM = "ASCENT";
static const char kW_DRG[]     PROGMEM = "DROGUE";
static const char kW_DEP1[]    PROGMEM = "PARAFOIL";
static const char kW_DEP2[]    PROGMEM = "DEPLOY";
static const char kW_DEP3[]    PROGMEM = "BURN";
static const char kW_DEP4[]    PROGMEM = "DROP";
static const char kW_PRC[]     PROGMEM = "PRECISION";
static const char kW_TRM[]     PROGMEM = "TERMINAL";
static const char kW_FIN[]     PROGMEM = "FINAL";
static const char kW_LND[]     PROGMEM = "LAND";
static const char kW_AUT[]     PROGMEM = "AUTO";
static const char kW_LFT[]     PROGMEM = "LEFT";
static const char kW_NEU[]     PROGMEM = "NEUTRAL";
static const char kW_RGT[]     PROGMEM = "RIGHT";
static const char kW_GNC[]     PROGMEM = "GNC";
static const char kW_STS[]     PROGMEM = "STATUS";
static const char kW_TGO[]     PROGMEM = "START";
static const char kW_TGO2[]    PROGMEM = "TELEM_START";
static const char kW_TGO3[]    PROGMEM = "TELEM:START";
static const char kW_TSP[]     PROGMEM = "STOP";
static const char kW_TSP2[]    PROGMEM = "TELEM_STOP";
static const char kW_TSP3[]    PROGMEM = "TELEM:STOP";
static const char kW_CLR1[]    PROGMEM = "CALIB:CLEAR";
static const char kW_CLR2[]    PROGMEM = "CALIB:RESET";
static const char kW_CAL[]     PROGMEM = "CAL";
static const char kW_TAR[]     PROGMEM = "TARE";
static const char kW_ZER[]     PROGMEM = "ZERO";

static const CmdMapEntry kCmdTable[] PROGMEM = {
    { kW_PAD,  '1' }, { kW_PAD2, '1' },
    { kW_ASC,  '2' },
    { kW_DRG,  '3' },
    { kW_DEP1, '4' }, { kW_DEP2, '4' }, { kW_DEP3, '4' }, { kW_DEP4, '4' },
    { kW_PRC,  '5' },
    { kW_TRM,  '6' },
    { kW_FIN,  '7' },
    { kW_LND,  '8' },
    { kW_AUT,  'M' },
    { kW_LFT,  'L' },
    { kW_NEU,  'N' },
    { kW_RGT,  'R' },
    { kW_GNC,  'G' },
    { kW_STS,  'S' },
    { kW_TGO,  '+' }, { kW_TGO2, '+' }, { kW_TGO3, '+' },
    { kW_TSP,  '-' }, { kW_TSP2, '-' }, { kW_TSP3, '-' },
    { kW_CLR1, 'C' }, { kW_CLR2, 'C' },
    { kW_CAL,  'B' },
    { kW_TAR,  'Z' }, { kW_ZER,  'Z' }
};

__attribute__((noinline))
void processTelecommand(char* rawCmd) {
    while (*rawCmd != '\0' && ((uint8_t)*rawCmd <= ' ' || (uint8_t)*rawCmd > 126)) rawCmd++;
    size_t len = strlen(rawCmd);
    while (len > 0 && ((uint8_t)rawCmd[len - 1] <= ' ' || (uint8_t)rawCmd[len - 1] > 126)) {
        rawCmd[--len] = '\0';
    }
    if (len == 0) return;
    if (cmdStartsP(rawCmd, PSTR("CanSat TX"), 9)) return;

    for (size_t i = 0; i < len; i++) {
        if (rawCmd[i] >= 'a' && rawCmd[i] <= 'z') rawCmd[i] -= 32;
    }

    if (strstr_P(rawCmd, PSTR("VBAT")) != NULL || strstr_P(rawCmd, PSTR("BAT:RESET")) != NULL || strstr_P(rawCmd, PSTR("RESET:BAT")) != NULL) {
        resetBatteryVoltage(8200U);
        LORA_SERIAL.println(F("ACK:VBAT_RESET=8.20"));
        return;
    }

    // Telemetry START / STOP uplink commands
    if (strstr_P(rawCmd, PSTR("START")) != NULL || strstr_P(rawCmd, PSTR("ON")) != NULL || rawCmd[0] == '+') {
        telemetryEnabled = true;
        lastTelemetryMs = millis() - 900UL;
#if HAS_SD_LOGGER
        startSDLogging();
#endif
        LORA_SERIAL.println(F("ACK:CX=ON"));
#if defined(__AVR_ATmega2560__)
        Serial1.println(F("ACK:CX=ON"));
#endif
        return;
    }
    if (strstr_P(rawCmd, PSTR("STOP")) != NULL || strstr_P(rawCmd, PSTR("OFF")) != NULL || rawCmd[0] == '-') {
        telemetryEnabled = false;
#if HAS_SD_LOGGER
        stopSDLogging();
#endif
        LORA_SERIAL.println(F("ACK:CX=OFF"));
#if defined(__AVR_ATmega2560__)
        Serial1.println(F("ACK:CX=OFF"));
#endif
        return;
    }

    if (cmdStartsP(rawCmd, PSTR("CMD:"), 4)) {
        rawCmd += 4;
        while (*rawCmd == ' ') rawCmd++;
    }

    const char* tok = rawCmd;
    if (cmdStartsP(rawCmd, PSTR("STATE:"), 6)) {
        tok = rawCmd + 6;
        while (*tok == ' ') tok++;
    }

    char code = 0;
    if (tok[0] != '\0' && tok[1] == '\0') {
        code = tok[0];
        if (code >= 'a' && code <= 'z') code -= 32;
        if (code == 'F') code = '4';
    } else {
        for (uint8_t i = 0; i < sizeof(kCmdTable) / sizeof(kCmdTable[0]); i++) {
            PGM_P w = (PGM_P)pgm_read_word(&kCmdTable[i].word);
            if (cmdStartsP(tok, w, strlen_P(w))) {
                code = (char)pgm_read_byte(&kCmdTable[i].code);
                break;
            }
        }
    }

    if (code >= '1' && code <= '8') {
        setFlightState((FlightState)(code - '0'));
        return;
    }

    switch (code) {
        case 'M':
            manualStateHold = !manualStateHold;
            LORA_SERIAL.print(F("ACK:MODE="));
            LORA_SERIAL.println(manualStateHold ? F("MANUAL_HOLD") : F("AUTO"));
            printCurrentStateSummary();
            return;
        case 'L':
            applyManualSteerCmd(-1.0f, F("ACK:STEER=LEFT"));
            return;
        case 'N':
            applyManualSteerCmd(0.0f, F("ACK:STEER=NEUTRAL"));
            return;
        case 'R':
            applyManualSteerCmd(1.0f, F("ACK:STEER=RIGHT"));
            return;
        case 'G':
            manualSteeringOverride = false;
            LORA_SERIAL.println(F("ACK:STEER=GNC"));
            printCurrentStateSummary();
            return;
        case 'S':
            printCurrentStateSummary();
            return;
        case '+':
            telemetryEnabled = true;
            lastTelemetryMs = millis() - 900UL;
            return;
        case '-':
            telemetryEnabled = false;
            return;
        case 'T':
            telemetryEnabled = !telemetryEnabled;
            return;
        case 'B':
            LORA_SERIAL.println(F("ACK:CALIB=STARTED"));
            calibrateMPU6050(true);
            return;
        case 'C':
            clearMPU6050EEPROM();
            LORA_SERIAL.println(F("ACK:CALIB=CLEARED"));
            return;
        case 'Z':
            if (imuOK) {
                pollMPU6050();
                activeHeadingOffsetDeg = wrap_180(-rawIntegratedYawDeg);
                LORA_SERIAL.print(F("ACK:TARE_HDG=0.0,OFFSET="));
                LORA_SERIAL.println(activeHeadingOffsetDeg, 1);
            }
            return;
        default:
            break;
    }

    // Precision Landing Target GPS Coordinates (GPS:<lat>,<lon> or TARGET:<lat>,<lon>)
    const char* coords = rawCmd;
    if (cmdStartsP(rawCmd, PSTR("GPS:"), 4)) {
        coords = rawCmd + 4;
    } else if (cmdStartsP(rawCmd, PSTR("TARGET:"), 7)) {
        coords = rawCmd + 7;
    }
    while (*coords == ' ') coords++;

    const char* comma = strchr(coords, ',');
    if (comma != NULL) {
        int32_t inLat = parseDegE7(coords);
        int32_t inLon = parseDegE7(comma + 1);
        if (labs(inLat) <= 900000000L && labs(inLon) <= 1800000000L && (inLat != 0 || inLon != 0)) {
            targetLatE7 = inLat;
            targetLonE7 = inLon;
            targetGPSConfigured = true;
            updateTargetFromGPS();

            LORA_SERIAL.print(F("ACK:TARGET_GPS="));
            LORA_SERIAL.print((float)targetLatE7 * 1e-7f, 6);
            LORA_SERIAL.print(',');
            LORA_SERIAL.print((float)targetLonE7 * 1e-7f, 6);
            LORA_SERIAL.print(F(",LOCAL_E="));
            LORA_SERIAL.print(targetLocal.x, 1);
            LORA_SERIAL.print(F(",N="));
            LORA_SERIAL.println(targetLocal.y, 1);
        }
    }
}

void pollTelecommands() {
    while (LORA_SERIAL.available() > 0) {
        char c = (char)LORA_SERIAL.read();
        if (c == '\r' || c == '\n') {
            if (cmdLen > 0) {
                cmdBuf[cmdLen] = '\0';
                processTelecommand(cmdBuf);
                cmdLen = 0;
            }
        } else if ((uint8_t)c >= 32 && (uint8_t)c <= 126 && cmdLen < sizeof(cmdBuf) - 1) {
            cmdBuf[cmdLen++] = c;
        }
    }
#if defined(__AVR_ATmega2560__)
    static char cmdBuf1[80];
    static uint8_t cmdLen1 = 0;
    while (Serial1.available() > 0) {
        char c = (char)Serial1.read();
        if (c == '\r' || c == '\n') {
            if (cmdLen1 > 0) {
                cmdBuf1[cmdLen1] = '\0';
                processTelecommand(cmdBuf1);
                cmdLen1 = 0;
            }
        } else if ((uint8_t)c >= 32 && (uint8_t)c <= 126 && cmdLen1 < sizeof(cmdBuf1) - 1) {
            cmdBuf1[cmdLen1++] = c;
        }
    }
#endif
}

__attribute__((noinline))
static void printCommaFloat(float v, uint8_t dec) {
    LORA_SERIAL.print(',');
    LORA_SERIAL.print(v, dec);
#if defined(__AVR_ATmega2560__)
    Serial1.print(',');
    Serial1.print(v, dec);
#endif
}

__attribute__((noinline))
void transmitLoRaTelemetry(
    uint32_t missionTimeSeconds,
    uint32_t packetNumber,
    float altitudeM,
    float pressurePa,
    float baroTempC,
    float batteryV,
    const char* gpsTime,
    double latitudeDeg,
    double longitudeDeg,
    float gpsAltitudeM,
    int satellites,
    float accelX,
    float accelY,
    float accelZ,
    float flywheelRPM,
    const char* currentState
) {
    float accelTotal = sqrtf(accelX * accelX + accelY * accelY + accelZ * accelZ);

    LORA_SERIAL.print(F(TEAM_ID));
    LORA_SERIAL.print(',');
    LORA_SERIAL.print(missionTimeSeconds);
    LORA_SERIAL.print(',');
    LORA_SERIAL.print(packetNumber);
#if defined(__AVR_ATmega2560__)
    Serial1.print(F(TEAM_ID));
    Serial1.print(',');
    Serial1.print(missionTimeSeconds);
    Serial1.print(',');
    Serial1.print(packetNumber);
#endif
    printCommaFloat(altitudeM, 2);
    printCommaFloat(pressurePa, 2);
    printCommaFloat(baroTempC, 2);
    printCommaFloat(batteryV, 2);
    LORA_SERIAL.print(',');
    LORA_SERIAL.print(gpsTime);
#if defined(__AVR_ATmega2560__)
    Serial1.print(',');
    Serial1.print(gpsTime);
#endif
    printCommaFloat((float)latitudeDeg, 7);
    printCommaFloat((float)longitudeDeg, 7);
    printCommaFloat(gpsAltitudeM, 2);
    LORA_SERIAL.print(',');
    LORA_SERIAL.print(satellites);
#if defined(__AVR_ATmega2560__)
    Serial1.print(',');
    Serial1.print(satellites);
#endif
    printCommaFloat(accelTotal, 2);
    printCommaFloat(flywheelRPM, 1);
    LORA_SERIAL.print(',');
    LORA_SERIAL.print(currentState);
#if defined(__AVR_ATmega2560__)
    Serial1.print(',');
    Serial1.print(currentState);
#endif
    float pDeg = atan2f(-ax, az) * 57.29578f;
    float rDeg = atan2f(ay, az) * 57.29578f;
    float yDeg = wrap_360(rawIntegratedYawDeg + activeHeadingOffsetDeg);
    printCommaFloat(rDeg, 1); // Field 15: ROLL (matches GCS EXTENDED_CSV_18 order)
    printCommaFloat(pDeg, 1); // Field 16: PITCH
    printCommaFloat(yDeg, 1); // Field 17: YAW
    LORA_SERIAL.println();
#if defined(__AVR_ATmega2560__)
    Serial1.println();
#endif
}

__attribute__((noinline))
static bool initI2CBusSafe() {
    Wire.begin();
    // CRITICAL SAFETY FOR 3.3V SENSORS ON ARDUINO UNO:
    // Disable ATmega328P internal 5V pull-ups on SDA (A4) and SCL (A5) so bus stays at 3.3V!
    digitalWrite(SDA, LOW);
    digitalWrite(SCL, LOW);
    Wire.setClock(100000);
    // Prevent AVR TWI infinite hang if no I2C sensors (no pull-ups) are connected:
    Wire.setWireTimeout(3000, true);
    delayMicroseconds(200);

    Wire.clearWireTimeoutFlag();
    Wire.beginTransmission(0x77);
    uint8_t err = Wire.endTransmission();
    digitalWrite(SDA, LOW);
    digitalWrite(SCL, LOW);
    if (Wire.getWireTimeoutFlag() || err == 5) {
        TWCR = 0; // Disable TWI when no pull-ups/sensors are connected so A4/A5 never hang
        return false;
    }
    return true;
}

// ============================================================================
// Setup
// ============================================================================
__attribute__((noinline))
void setup() {
    Serial.begin(LORA_BAUD); // Pin 0 (RX0) & Pin 1 (TX0) @ 115200 baud
#if defined(__AVR_ATmega2560__)
    Serial1.begin(LORA_BAUD); // Pin 19 (RX1) & Pin 18 (TX1) @ 115200 baud
#endif
    pinMode(VBAT_PIN, INPUT);
    delay(500);

    // Initialize Actuators (2x ES3004 Servos on Pins 2 & 3, Burnwire Pin 4, Buzzer Pin 5, ESC Pin 6)
    actuators.init();

#if HAS_SD_LOGGER
    // Initialize MicroSD Card on SPI (Mega: MISO=50, MOSI=51, SCK=52, CS=53 or 10)
    if (initSDCard()) {
        startSDLogging();
    }
#endif

    // Start I2C safely (Uno: A4/A5, Mega: Pin 20 SDA / Pin 21 SCL) with 3 ms timeout
    if (initI2CBusSafe()) {
        // 1. MS5611 Barometer (auto-detects 0x77 or 0x76)
        baroOK = initMS5611();

        // 2. MPU6050 6-DOF IMU
        imuOK = initMPU6050();
        if (imuOK) {
            calibrateMPU6050(false);
        }

        // 3. NEO-M9N GNSS (0x42)
        gnssOK = initGNSS();
    }

    printSensorStatus();

    // Calibrate Baseline Launch Pressure (50-sample average + 5-sample warm-up)
    if (baroOK) {
        float dummyP, dummyT;
        for (int i = 0; i < 5; i++) {
            readMS5611(dummyP, dummyT);
            delay(10);
        }

        float sumPressure = 0.0f;
        float lastCalTemp = 0.0f;
        int validSamples = 0;
        for (int i = 0; i < 50; i++) {
            float currentP = 0.0f, currentT = 0.0f;
            if (readMS5611(currentP, currentT)) {
                if (isfinite(currentP) && currentP > 0.0f) {
                    sumPressure += currentP;
                    lastCalTemp = currentT;
                    validSamples++;
                }
            }
            delay(10);
        }

        if (validSamples > 0) {
            launchPressure = sumPressure / (float)validSamples;
        } else {
            launchPressure = SEA_LEVEL_HPA;
        }
        filteredPressure = launchPressure;
        lastPressurePa = launchPressure * 100.0f;
        lastTempC = lastCalTemp;
        groundAlt = 0.0f;
    }

    gnc.setTarget(targetLocal.x, targetLocal.y);
    gnc.reset(millis());
    gnc.forceState(FlightState::LAUNCH_PAD, millis());
    lastYawForRateDeg = rawIntegratedYawDeg;
    Serial.print(F("[NAV] Target GPS set: "));
    Serial.print((float)targetLatE7 * 1e-7f, 6);
    Serial.print(F(", "));
    Serial.println((float)targetLonE7 * 1e-7f, 6);

    missionStartMs = millis();
    lastControlUpdateMs = missionStartMs;
    lastTelemetryMs = missionStartMs;
}

// ============================================================================
// Main Loop
// ============================================================================
__attribute__((noinline))
void loop() {
    uint32_t now = millis();

    // Task 0: Poll Telecommands, 100 Hz MPU6050 Attitude & GNSS I2C Stream
    pollTelecommands();
    pollMPU6050();
    // Latch new PVT solutions until the next 10 Hz tick consumes them (the old
    // code only used a fix if it completed in the same loop pass as the tick,
    // so most GNSS updates were silently dropped).
    static bool gnssFixPending = false;
    if (pollGNSS()) gnssFixPending = true;
    actuators.update(now);

    // Task 1: 10 Hz Control Loop
    if (now - lastControlUpdateMs >= (1000 / CONTROL_LOOP_HZ)) {
        float dt = (lastControlUpdateMs == 0) ? 0.1f : (float)(now - lastControlUpdateMs) / 1000.0f;
        lastControlUpdateMs = now;

        // 1. Read MS5611 Barometer
        float alt_agl = lastBaroAlt;
        float rawPressure = 0.0f, rawTemp = 0.0f;
        if (baroOK && readMS5611(rawPressure, rawTemp)) {
            if (isfinite(rawPressure) && rawPressure > 0.0f) {
                lastPressurePa = rawPressure * 100.0f;
                lastTempC = rawTemp;
                filteredPressure = (EMA_ALPHA * rawPressure) + ((1.0f - EMA_ALPHA) * filteredPressure);
                alt_agl = pressureToAlt(filteredPressure, launchPressure);
                lastBaroAlt = alt_agl;
            }
        }

        if (gnssFixType >= 3 && gnssSats >= GNSS_MIN_SATS && !launchGPSFixed) {
            launchLatE7 = gnssLatE7;
            launchLonE7 = gnssLonE7;
            launchCosLat = cosf((float)launchLatE7 * 1e-7f * ((float)M_PI / 180.0f));
            launchGPSFixed = true;
            updateTargetFromGPS();
        }

        // 2. Sensor snapshot for the GNC core
        GncSensors sens;
        sens.now_ms = now;
        sens.baro_valid = baroOK;
        sens.baro_alt_agl = alt_agl;
        sens.gyro_valid = imuOK;
        float yawNow = rawIntegratedYawDeg;
        sens.yaw_rate_dps = imuOK ? (wrap_180(yawNow - lastYawForRateDeg) / dt) : 0.0f;
        lastYawForRateDeg = yawNow;
        sens.gnss_new = gnssFixPending;
        gnssFixPending = false;
        sens.gnss_valid = launchGPSFixed && gnssFixType >= 3 && gnssSats >= GNSS_MIN_SATS;
        sens.gnss_x = 0.0f;
        sens.gnss_y = 0.0f;
        if (sens.gnss_valid) {
            LocalPoint p = e7_to_local(gnssLatE7, gnssLonE7, launchLatE7, launchLonE7, launchCosLat);
            sens.gnss_x = p.x;
            sens.gnss_y = p.y;
        }
        sens.gnss_vn = gnssVelNorth;
        sens.gnss_ve = gnssVelEast;
        sens.gnss_vd = gnssVelDown;

        // 3. One GNC step (navigation, air data, mission sequencing, guidance)
        gnc.manualStateHold = manualStateHold;
        gnc.manualSteering = manualSteeringOverride && flightSM.isSteeringEnabled();
        gnc.manualSteeringValue = overrideSteeringVal;
        GncOutputs gout = gnc.step(sens);
        currentSteering = gout.steering;

        syncActuatorsToState(now);
    }

    // Task 2: 1 Hz LoRa Telemetry Transmission (Auto-starts on ASCENT or via START command)
    if (flightSM.getState() >= FlightState::ASCENT && flightSM.getState() <= FlightState::LANDING) {
        if (!telemAscentTriggered) {
            telemAscentTriggered = true;
            telemetryEnabled = true;
            lastTelemetryMs = now - 1000UL; // Immediate first packet
#if HAS_SD_LOGGER
            startSDLogging();
#endif
        }
    } else if (flightSM.getState() == FlightState::LAUNCH_PAD) {
        telemAscentTriggered = false;
    }

    if (telemetryEnabled && (now - lastTelemetryMs >= 1000UL)) {
        lastTelemetryMs = now;
        packetCount++;

        float vbat = readBatteryVoltage();
        uint32_t missionTimeSec = (now - missionStartMs) / 1000UL;

        transmitLoRaTelemetry(
            missionTimeSec,
            packetCount,
            isfinite(lastBaroAlt) ? lastBaroAlt : 0.0f,
            isfinite(lastPressurePa) ? lastPressurePa : 0.0f,
            isfinite(lastTempC) ? lastTempC : 0.0f,
            vbat,
            gnssTimeStr,
            gnssLat,
            gnssLon,
            gnssAltMSL,
            gnssSats,
            ax,
            ay,
            az,
            actuators.getFlywheelRPM(),
            flightSM.getStateName()
        );
    }

#if HAS_SD_LOGGER
    if (sdLoggingActive && (now - lastSdLogMs >= SD_LOG_INTERVAL_MS)) {
        lastSdLogMs = now;
        sdRecordCount++;

        float vbat = readBatteryVoltage();
        uint32_t missionTimeSec = (now - missionStartMs) / 1000UL;

        logDataToSD(
            missionTimeSec,
            sdRecordCount,
            isfinite(lastBaroAlt) ? lastBaroAlt : 0.0f,
            isfinite(lastPressurePa) ? lastPressurePa : 0.0f,
            isfinite(lastTempC) ? lastTempC : 0.0f,
            vbat,
            gnssTimeStr,
            gnssLat,
            gnssLon,
            gnssAltMSL,
            gnssSats,
            ax,
            ay,
            az,
            actuators.getFlywheelRPM(),
            flightSM.getStateName()
        );
    }
#endif
}
