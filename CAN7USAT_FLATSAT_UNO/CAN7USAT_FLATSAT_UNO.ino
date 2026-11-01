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

#include "config.h"
#include "nav_types.h"
#include "nav_math.h"
#include "ekf.h"
#include "guidance.h"
#include "flight_sm.h"
#include "actuators.h"

// ============================================================================
// CAN-7USAT Team 017 | Arduino Uno (ATmega328P) FlatSat Interactive FSW
// ----------------------------------------------------------------------------
// Sensors on I2C (Uno A4 = SDA, A5 = SCL, powered from 3.3V!):
//   1. MS5611 Barometer  (0x77 / 0x76)
//   2. MPU6050 6-DOF IMU (0x68 / 0x69)
//   3. NEO-M9N GNSS      (0x42 DDC/I2C NMEA)
// Actuators:
//   - Pin 2: Right Parafoil Brake Servo (Emax ES3004)
//   - Pin 3: Left Parafoil Brake Servo (Emax ES3004)
//   - Pin 4: Burnwire XY-MOS MOSFET #1 (30.0s pulse @ 700m deployment)
//   - Pin 5: Recovery Buzzer XY-MOS MOSFET #2 (500ms beacon @ Landing)
//   - Pin 6: Readytosky MT2204 2300KV Flywheel ESC (6000 RPM / 1353 us)
//   - Pin A0: Battery Voltage Divider (150k top / 47k bottom)
//   - Pin 0 (RX) / Pin 1 (TX): USB Serial Console & Ra-08H LoRa (115200 baud)
// ============================================================================

static uint8_t baroI2CAddr = 0x77;
static float baroC[7];
static uint8_t mpuI2CAddr = MPU6050_I2C_ADDR_PRIMARY;
static uint8_t mpuWhoAmI = 0x00;
static bool mpuUpsideDown = MPU_MOUNTED_UPSIDE_DOWN;
static bool mpuCalibrated = false;
static float gyroBiasZ = 0.0f;
static float accelBiasX = 0.0f, accelBiasY = 0.0f, accelBiasZ = 0.0f;
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

// Navigation & Estimation
ExtendedKalmanFilter ekf;
WindEstimator windEstimator;
PIDController pid(PID_KP, PID_KI, PID_KD, -1.0f, 1.0f, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);
WaypointManager waypointManager;
GuidanceController guidanceController(&waypointManager, &pid);
LoiterGuidance loiterGuidance;

// Flight Management & Actuators
FlightStateMachine flightSM;
ActuatorManager actuators;

// Barometer Calibration & Filtering State Variables
float launchPressure = 1013.25f;
float filteredPressure = 1013.25f;
const float EMA_ALPHA = BARO_EMA_ALPHA;
float groundAlt = 0.0f;

// Navigation & Timing State Variables
GPSPoint launchGPS(0.0, 0.0);
bool launchGPSFixed = false;
GPSPoint targetGPS(0.0, 0.0);
bool targetGPSConfigured = false;
LocalPoint targetLocal(TARGET_DEFAULT_EAST, TARGET_DEFAULT_NORTH);

// FlatSat Interactive Control Flags
bool telemetryEnabled = false;     // Off by default in FlatSat so Serial Monitor stays clean; toggle with 't' or 'START'
bool telemAscentTriggered = false;
bool manualStateHold = true;       // True by default in FlatSat so user can step states 1-8
bool manualSteeringOverride = false;
bool directServoPulseMode = false; // True when user triggers 'u' (Unwind), 'x' (Brake), or 'd' (Detach)
uint16_t directServoPulseUs = 0;
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
float gnssAltMSL = 0.0f;
float gnssVelEast = 0.0f, gnssVelNorth = 0.0f;
int gnssSats = 0;
static uint32_t gnssRxBytes = 0;

// ============================================================================
// Lightweight Direct u-blox NEO-M9N I2C (0x42) Driver
// ============================================================================
#define UBLOX_I2C_ADDR 0x42

static float parseDecimalFloat(const char* s) {
    if (!s) return 0.0f;
    while (*s == ' ') s++;
    float sign = 1.0f;
    if (*s == '-') { sign = -1.0f; s++; }
    else if (*s == '+') { s++; }

    float val = 0.0f;
    while (*s >= '0' && *s <= '9') {
        val = val * 10.0f + (float)(*s - '0');
        s++;
    }
    if (*s == '.') {
        s++;
        float scale = 0.1f;
        while (*s >= '0' && *s <= '9') {
            val += (float)(*s - '0') * scale;
            scale *= 0.1f;
            s++;
        }
    }
    return val * sign;
}




// ============================================================================
// UBX Binary (UBX-NAV-PVT) & NMEA Stream Decoders
// Matches Teensy CAN7USAT_FSW gnss.setAutoPVT(true) configuration
// ============================================================================
static uint8_t ubxStep = 0;
static uint8_t ubxClass = 0, ubxId = 0;
static uint16_t ubxLen = 0, ubxPos = 0;
static uint8_t ubxCkA = 0, ubxCkB = 0;
static uint8_t pvtHour = 0, pvtMin = 0, pvtSec = 0, pvtFix = 0, pvtSats = 0;
static int32_t pvtLat = 0, pvtLon = 0, pvtHmsl = 0, pvtVelN = 0, pvtVelE = 0;

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
            if (p) *p = b;
        }
        if (++ubxPos >= ubxLen) ubxStep = 7;
    } else if (ubxStep == 7) {
        if (b == ubxCkA) ubxStep = 8;
        else ubxStep = (b == 0xB5) ? 1 : 0;
    } else if (ubxStep == 8) {
        if (b == ubxCkB && ubxClass == 0x01 && ubxId == 0x07) {
            gnssSats = pvtSats;
            format2Digits(gnssTimeStr, pvtHour);
            gnssTimeStr[2] = ':';
            format2Digits(gnssTimeStr + 3, pvtMin);
            gnssTimeStr[5] = ':';
            format2Digits(gnssTimeStr + 6, pvtSec);
            gnssTimeStr[8] = '\0';
            if (pvtFix >= 2) {
                gnssLat = (double)pvtLat * 1e-7;
                gnssLon = (double)pvtLon * 1e-7;
                gnssAltMSL = (float)pvtHmsl * 0.001f;
                gnssVelNorth = (float)pvtVelN * 0.001f;
                gnssVelEast = (float)pvtVelE * 0.001f;
            }
            gotPvt = true;
        }
        ubxStep = (b == 0xB5) ? 1 : 0;
    }
    return gotPvt;
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

    return true;
}

__attribute__((noinline))
static bool pollGNSS() {
    if (!gnssOK) return false;

    Wire.beginTransmission(UBLOX_I2C_ADDR);
    Wire.write(0xFD);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)UBLOX_I2C_ADDR, (uint8_t)2) < 2) return false;

    uint8_t h = Wire.read();
    uint8_t l = Wire.read();
    uint16_t avail = ((uint16_t)h << 8) | (uint16_t)l;
    if (avail == 0 || avail == 0xFFFF) return false;
    if (avail > 64) avail = 64;

    bool updated = false;
    while (avail > 0) {
        uint8_t chunk = (avail > 32) ? 32 : (uint8_t)avail;
        if (Wire.requestFrom((uint8_t)UBLOX_I2C_ADDR, chunk) == 0) break;
        while (Wire.available()) {
            uint8_t b = (uint8_t)Wire.read();
            gnssRxBytes++;
            if (parseUbxByte(b)) {
                updated = true;
            }
        }
        avail -= chunk;
    }
    return updated;
}

// ============================================================================
// MS5611-01BA03 High-Precision Barometer Driver (Direct I2C)
// ============================================================================
static uint16_t readMS5611Prom(uint8_t addr, uint8_t reg) {
    Wire.beginTransmission(addr);
    Wire.write(0xA0 | (reg << 1));
    if (Wire.endTransmission() != 0) return 0;
    if (Wire.requestFrom(addr, (uint8_t)2) < 2) return 0;
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    return ((uint16_t)msb << 8) | (uint16_t)lsb;
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
        Wire.write(0x75);
        if (Wire.endTransmission(false) == 0 && Wire.requestFrom(addr, (uint8_t)1) == 1) {
            mpuWhoAmI = Wire.read();
            mpuI2CAddr = addr;

            writeMPURegister(mpuI2CAddr, 0x6B, 0x80);
            delay(50);
            writeMPURegister(mpuI2CAddr, 0x6B, 0x01);
            delay(20);
            writeMPURegister(mpuI2CAddr, 0x6C, 0x00);
            writeMPURegister(mpuI2CAddr, 0x19, 0x04);
            writeMPURegister(mpuI2CAddr, 0x1A, 0x03);
            writeMPURegister(mpuI2CAddr, 0x1B, 0x08);
            writeMPURegister(mpuI2CAddr, 0x1C, 0x08);
            delay(20);
            lastImuPollUs = micros();
            return true;
        }
    }
    return false;
}

static bool readMPU6050Raw(int16_t outRaw[4]) {
    Wire.beginTransmission(mpuI2CAddr);
    Wire.write(0x3B);
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
        saveMPU6050CalibToEEPROM();
        mpuCalibrated = true;
        Serial.println(F("ACK:CALIB=OK (SAVED)"));
    }
    lastImuPollUs = micros();
}

__attribute__((noinline))
void pollMPU6050() {
    if (!imuOK) return;
    uint32_t nowUs = micros();
    uint32_t elapsedUs = nowUs - lastImuPollUs;
    if (elapsedUs < 10000UL) return;

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
            "ACCELEROMETER_DATA,GYRO_SPIN_RATE,FLIGHT_SOFTWARE_STATE,OPTIONAL_DATA"
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
    csvFile.println(',');
    csvFile.flush();
}
#endif

// ============================================================================
// FlatSat Console Menu & ACTIVE/INACTIVE Status Matrix
// ============================================================================
__attribute__((noinline))
static void printLblFloat(const __FlashStringHelper* lbl, float v, uint8_t dec) {
    Serial.print(lbl);
    Serial.print(v, dec);
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
void printFlatSatMenu() {
#if defined(__AVR_ATmega2560__)
    Serial.println(F(
        "\n=== FLATSAT MEGA 2560 ===\n"
        "[1..8] States | [s] Status | [m] Hold/Auto | [t] LoRa TX\n"
        "[l/n/r/g] Steer | [u/x/d] Servos | [b] Calib | [z] Tare"
    ));
#else
    Serial.println(F(
        "\n=== FLATSAT UNO ===\n"
        "[1..8] States | [s] Status | [m] Hold/Auto | [t] LoRa TX\n"
        "[l/n/r/g] Steer | [u/x/d] Servos | [b] Calib | [z] Tare"
    ));
#endif
}

__attribute__((noinline))
void printCurrentStateSummary() {
    FlightState st = flightSM.getState();
    bool activeFlight = (st >= FlightState::ASCENT && st < FlightState::LANDING);

    Serial.print(F("\n--- FLATSAT STATUS ["));
    Serial.print(flightSM.getStateName());
    Serial.print(manualStateHold ? F(" | HOLD] ---\nBaro: ") : F(" | AUTO] ---\nBaro: "));
    if (!baroOK) Serial.println(F("OFFLINE"));
    else {
        printLblFloat(F("Alt="), lastBaroAlt, 1);
        printLblFloat(F("m P="), lastPressurePa, 0);
        printLblFloat(F("Pa T="), lastTempC, 1);
        Serial.println('C');
    }

    Serial.print(F("GPS : "));
    if (!gnssOK) Serial.println(F("OFFLINE"));
    else if (gnssSats > 0 || (gnssTimeStr[0] != '0' || gnssTimeStr[1] != '0')) {
        if (gnssSats > 0) {
            Serial.print(F("FIX Sats=")); Serial.print(gnssSats);
            printLblFloat(F(" Lat="), (float)gnssLat, 5);
            printLblFloat(F(" Lon="), (float)gnssLon, 5);
        } else {
            Serial.print(F("ACQ Sats=0"));
        }
        Serial.print(F(" T=")); Serial.println(gnssTimeStr);
    } else {
        Serial.print(F("SEARCHING Sats=0 (Rx="));
        Serial.print(gnssRxBytes);
        Serial.println(F(" B)"));
    }

    Serial.print(F("IMU : "));
    if (!imuOK) Serial.println(F("OFFLINE"));
    else {
        Serial.print(activeFlight ? F("ACT") : F("STBY"));
        printLblFloat(mpuUpsideDown ? F("(INV) Hdg=") : F("(UP) Hdg="), wrap_360(rawIntegratedYawDeg + activeHeadingOffsetDeg), 1);
        printLblFloat(F(" Gz="), spin, 1);
        Serial.println();
    }

    Serial.print(F("Burn: "));
    Serial.println(actuators.isBurnwirePinHigh() ? F("ON") : F("OFF"));

    Serial.print(F("Buzz: "));
    Serial.println(flightSM.isBuzzerActive() ? F("BEACON") : F("OFF"));

    Serial.print(F("Serv: "));
    if (directServoPulseMode || flightSM.isSteeringEnabled()) {
        printLblFloat(F("Steer="), currentSteering, 2);
        Serial.print(F(" L=")); Serial.print(actuators.getLeftServoPulseUs());
        Serial.print(F(" R=")); Serial.println(actuators.getRightServoPulseUs());
    } else {
        Serial.println(F("OFF [0mA]"));
    }

    Serial.print(F("Flyw: "));
    Serial.print(actuators.isFlywheelActive() ? F("6000RPM ") : F("0RPM "));
    Serial.print(actuators.getFlywheelPulseUs());
    Serial.println(F("us"));

#if HAS_SD_LOGGER
    Serial.print(F("SD  : "));
    if (!sdOK) {
        Serial.println(F("OFFLINE"));
    } else if (sdLoggingActive) {
        Serial.print(F("LOGGING -> "));
        Serial.print(csvFileName);
        Serial.print(F(" ("));
        Serial.print(sdRecordCount);
        Serial.println(F(" rows)"));
    } else {
        Serial.print(F("READY (CS="));
        Serial.print(sdActiveCsPin);
        Serial.println(F(")"));
    }
#endif

    printLblFloat(F("Batt: "), readBatteryVoltage(), 2);
    Serial.println(telemetryEnabled ? F("V | LoRa: ON") : F("V | LoRa: PAUSED"));
}

__attribute__((noinline))
void updateTargetFromGPS() {
    if (!targetGPSConfigured) return;
    GPSPoint ref = launchGPSFixed ? launchGPS : GPSPoint(gnssLat, gnssLon);
    if (!launchGPSFixed && gnssLat == 0.0 && gnssLon == 0.0) return;
    targetLocal = gps_to_local(targetGPS, ref);
    waypointManager.init(LocalPoint(0.0f, 0.0f), targetLocal, NUM_WAYPOINTS, WAYPOINT_ACCEPTANCE_RAD);
    loiterGuidance.setCenter(targetLocal);
}

__attribute__((noinline))
static void syncActuatorsToState(uint32_t now) {
    if (directServoPulseMode) {
        actuators.setDirectPulse(directServoPulseUs, directServoPulseUs);
    } else {
        actuators.applySteering(currentSteering, flightSM.isSteeringEnabled(), now, flightSM.getState());
    }
    actuators.setFlywheel(flightSM.isFlywheelActive());
    actuators.setBurnwire(flightSM.isBurnwireActive());
    actuators.updateBuzzer(flightSM.isBuzzerActive(), now);
    lastBurnwirePinState = actuators.isBurnwirePinHigh();
}

void setFlightState(FlightState newState) {
    uint32_t now = millis();
    directServoPulseMode = false;
    manualStateHold = true; // Always hold the manually chosen state on FlatSat
    flightSM.forceState(newState, now);

    if (newState >= FlightState::ASCENT) {
        telemetryEnabled = true;
        telemAscentTriggered = true;
        lastTelemetryMs = now - 1000UL;
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
static void sendControlAck(const __FlashStringHelper* msg) {
    LORA_SERIAL.println(msg);
#if defined(__AVR_ATmega2560__)
    Serial1.println(msg);
#endif
}

__attribute__((noinline))
static void applyManualSteerCmd(float val, const __FlashStringHelper* msg) {
    directServoPulseMode = false;
    manualSteeringOverride = true;
    overrideSteeringVal = val;
    currentSteering = val;
    actuators.applySteering(currentSteering, flightSM.isSteeringEnabled(), millis(), flightSM.getState());
    sendControlAck(msg);
    printCurrentStateSummary();
}

__attribute__((noinline))
static void applyDirectServoCmd(int pulseUs, const __FlashStringHelper* msg) {
    directServoPulseMode = true;
    directServoPulseUs = pulseUs;
    actuators.setDirectPulse(pulseUs, pulseUs);
    Serial.println(msg);
}

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
        sendControlAck(F("ACK:VBAT_RESET=8.20"));
        return;
    }

    // Telemetry START / STOP uplink commands
    if (strstr_P(rawCmd, PSTR("START")) != NULL || strstr_P(rawCmd, PSTR("ON")) != NULL || rawCmd[0] == '+') {
        telemetryEnabled = true;
        lastTelemetryMs = millis() - 900UL;
#if HAS_SD_LOGGER
        startSDLogging();
#endif
        sendControlAck(F("ACK:CX=ON"));
        return;
    }
    if (strstr_P(rawCmd, PSTR("STOP")) != NULL || strstr_P(rawCmd, PSTR("OFF")) != NULL || rawCmd[0] == '-') {
        telemetryEnabled = false;
#if HAS_SD_LOGGER
        stopSDLogging();
#endif
        sendControlAck(F("ACK:CX=OFF"));
        return;
    }

    if (cmdStartsP(rawCmd, PSTR("CMD:"), 4)) {
        rawCmd += 4;
        while (*rawCmd == ' ') rawCmd++;
    }
    if (cmdStartsP(rawCmd, PSTR("STATE:"), 6)) {
        rawCmd += 6;
        while (*rawCmd == ' ') rawCmd++;
    }

    char* comma = strchr(rawCmd, ',');
    if (comma != NULL) {
        char* coords = rawCmd;
        if (cmdStartsP(rawCmd, PSTR("GPS:"), 4)) coords = rawCmd + 4;
        else if (cmdStartsP(rawCmd, PSTR("TARGET:"), 7)) coords = rawCmd + 7;
        while (*coords == ' ') coords++;
        double inLat = (double)parseDecimalFloat(coords);
        double inLon = (double)parseDecimalFloat(comma + 1);
        if (isfinite(inLat) && isfinite(inLon) && (inLat != 0.0 || inLon != 0.0)) {
            targetGPS = GPSPoint(inLat, inLon);
            targetGPSConfigured = true;
            updateTargetFromGPS();
            LORA_SERIAL.print(F("ACK:TARGET_GPS="));
            LORA_SERIAL.print(targetGPS.lat, 6);
            LORA_SERIAL.print(',');
            LORA_SERIAL.println(targetGPS.lon, 6);
#if defined(__AVR_ATmega2560__)
            Serial1.print(F("ACK:TARGET_GPS="));
            Serial1.print(targetGPS.lat, 6);
            Serial1.print(',');
            Serial1.println(targetGPS.lon, 6);
#endif
        }
        return;
    }

    char code = rawCmd[0];
    if (code >= 'a' && code <= 'z') code -= 32;

    if (code >= '1' && code <= '8') {
        setFlightState((FlightState)(code - '0'));
        return;
    }

    switch (code) {
        case '?':
            printFlatSatMenu();
            return;
        case 'M':
            manualStateHold = !manualStateHold;
            sendControlAck(manualStateHold ? F("ACK:MODE=MANUAL_HOLD") : F("ACK:MODE=AUTO"));
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
            directServoPulseMode = false;
            manualSteeringOverride = false;
            sendControlAck(F("ACK:STEER=GNC"));
            printCurrentStateSummary();
            return;
        case 'U':
            applyDirectServoCmd(ES3004_MAX_PULSE_US, F("[SERVO] UNWIND (2400us)"));
            return;
        case 'X':
            applyDirectServoCmd(ES3004_MIN_PULSE_US, F("[SERVO] BRAKE (500us)"));
            return;
        case 'D':
            applyDirectServoCmd(0, F("[SERVO] DETACH (0mA)"));
            return;
        case 'S':
            printCurrentStateSummary();
            return;
        case 'T':
            telemetryEnabled = !telemetryEnabled;
            sendControlAck(telemetryEnabled ? F("ACK:CX=ON") : F("ACK:CX=OFF"));
            return;
        case 'B':
            sendControlAck(F("ACK:CALIB=STARTED"));
            calibrateMPU6050(true);
            return;
        case 'C':
            clearMPU6050EEPROM();
            sendControlAck(F("ACK:CALIB=CLEARED"));
            return;
        case 'Z':
            if (imuOK) {
                pollMPU6050();
                activeHeadingOffsetDeg = wrap_180(-rawIntegratedYawDeg);
                LORA_SERIAL.print(F("ACK:TARE="));
                LORA_SERIAL.println(activeHeadingOffsetDeg, 1);
#if defined(__AVR_ATmega2560__)
                Serial1.print(F("ACK:TARE="));
                Serial1.println(activeHeadingOffsetDeg, 1);
#endif
            }
            return;
        default:
            break;
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
    digitalWrite(SDA, LOW);
    digitalWrite(SCL, LOW);
    Wire.setClock(100000);
    Wire.setWireTimeout(3000, true);
    delayMicroseconds(200);

    Wire.clearWireTimeoutFlag();
    Wire.beginTransmission(0x77);
    uint8_t err = Wire.endTransmission();
    digitalWrite(SDA, LOW);
    digitalWrite(SCL, LOW);
    if (Wire.getWireTimeoutFlag() || err == 5) {
        TWCR = 0;
        return false;
    }
    return true;
}

// ============================================================================
// Setup
// ============================================================================
__attribute__((noinline))
void setup() {
    Serial.begin(LORA_BAUD);
#if defined(__AVR_ATmega2560__)
    Serial1.begin(LORA_BAUD);
#endif
    pinMode(VBAT_PIN, INPUT);
    delay(500);

    actuators.init();

#if HAS_SD_LOGGER
    if (initSDCard()) {
        startSDLogging();
    }
#endif

    if (initI2CBusSafe()) {
        baroOK = initMS5611();
        imuOK = initMPU6050();
        if (imuOK) {
            calibrateMPU6050(false);
        }
        gnssOK = initGNSS();
    }

    if (baroOK) {
        float sumP = 0.0f;
        float lastT = 0.0f;
        int valid = 0;
        for (int i = 0; i < 20; i++) {
            float p, t;
            if (readMS5611(p, t)) {
                sumP += p;
                lastT = t;
                valid++;
            }
            delay(15);
        }
        launchPressure = (valid > 0) ? (sumP / (float)valid) : SEA_LEVEL_HPA;
        filteredPressure = launchPressure;
        lastPressurePa = launchPressure * 100.0f;
        lastTempC = lastT;
        groundAlt = 0.0f;
    }

    LocalPoint startPt(0.0f, 0.0f);
    waypointManager.init(startPt, targetLocal, NUM_WAYPOINTS, WAYPOINT_ACCEPTANCE_RAD);
    loiterGuidance.setCenter(targetLocal);

    ekf.reset(0.0f, 0.0f, 0.0f, 0.0f);
    flightSM.reset();
    flightSM.forceState(FlightState::LAUNCH_PAD, millis());

    printFlatSatMenu();
    printCurrentStateSummary();

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

    pollTelecommands();
    pollMPU6050();
    bool newGnssFix = pollGNSS();
    actuators.update(now);

    if (now - lastControlUpdateMs >= (1000 / CONTROL_LOOP_HZ)) {
        float dt = (lastControlUpdateMs == 0) ? 0.1f : (float)(now - lastControlUpdateMs) / 1000.0f;
        lastControlUpdateMs = now;

        FlightState prevState = flightSM.getState();
        FlightState currentState = prevState;

        float alt_agl = lastBaroAlt;
        float rawPressure = 0.0f, rawTemp = 0.0f;
        if (baroOK && readMS5611(rawPressure, rawTemp)) {
            if (isfinite(rawPressure) && rawPressure > 0.0f) {
                lastPressurePa = rawPressure * 100.0f;
                lastTempC = rawTemp;
                filteredPressure = (EMA_ALPHA * rawPressure) + ((1.0f - EMA_ALPHA) * filteredPressure);
                alt_agl = pressureToAlt(filteredPressure, launchPressure);
                lastBaroAlt = alt_agl;

                if (currentState > FlightState::LAUNCH_PAD && currentState < FlightState::LANDING) {
                    ekf.correctBaro(alt_agl);
                }
            }
        }

        if (gnssSats >= 4 && !launchGPSFixed && (gnssLat != 0.0 || gnssLon != 0.0)) {
            launchGPS = GPSPoint(gnssLat, gnssLon);
            launchGPSFixed = true;
            updateTargetFromGPS();
        }

        if (currentState > FlightState::LAUNCH_PAD && currentState < FlightState::LANDING) {
            float heading_deg = 0.0f;
            if (imuOK) {
                heading_deg = wrap_360(rawIntegratedYawDeg + activeHeadingOffsetDeg);
                ekf.correctHeading(heading_deg);
            }

            if (newGnssFix && gnssSats >= 4) {
                GPSPoint currentGPS(gnssLat, gnssLon);
                LocalPoint posLocal = gps_to_local(currentGPS, launchGPS);
                ekf.correctGPS(posLocal.x, posLocal.y, gnssVelEast, gnssVelNorth, true);

                if (currentState <= FlightState::DROGUE_DESCENT) {
                    windEstimator.update(gnssVelEast, gnssVelNorth);
                } else {
                    float psi_rad = deg2rad(heading_deg);
                    float air_vx = FORWARD_AIRSPEED * sinf(psi_rad);
                    float air_vy = FORWARD_AIRSPEED * cosf(psi_rad);
                    windEstimator.update(gnssVelEast - air_vx, gnssVelNorth - air_vy);
                }
            }

            ekf.setWindEstimate(windEstimator.getWindX(), windEstimator.getWindY());
            ekf.predict(dt, currentSteering);
        }

        LocalPoint navPos(ekf.getX(), ekf.getY());
        float navHeading = ekf.getHeadingDeg();
        float navAlt = ekf.getAltitude();

        if (!manualStateHold) {
            flightSM.update(alt_agl, navHeading, navPos, targetLocal, dt, now);
        } else {
            flightSM.tickTimers(now);
        }
        currentState = flightSM.getState();

        ekf.setKinematics(
            (currentState == FlightState::DROGUE_DESCENT) ? 0.0f : FORWARD_AIRSPEED,
            (currentState == FlightState::DROGUE_DESCENT) ? DROGUE_DESCENT_RATE : PARAFOIL_DESCENT_RATE
        );

        if (manualSteeringOverride && flightSM.isSteeringEnabled()) {
            currentSteering = overrideSteeringVal;
        } else if (currentState == FlightState::PRECISION_GUIDANCE) {
            guidanceController.setWind(windEstimator.getWindX(), windEstimator.getWindY());
            GuidanceOutput gOut = guidanceController.update(navPos, navHeading, navAlt, dt);
            currentSteering = gOut.steering;
        } else if (currentState == FlightState::TERMINAL_GUIDANCE) {
            loiterGuidance.setWind(windEstimator.getWindX(), windEstimator.getWindY());
            currentSteering = loiterGuidance.update(navPos, navHeading);
        } else if (currentState == FlightState::FINAL_APPROACH) {
            float desired_course = bearing(navPos, targetLocal);
            WindCompResult comp = calculate_wind_corrected_heading(
                desired_course, FORWARD_AIRSPEED, windEstimator.getWindX(), windEstimator.getWindY()
            );
            float err_deg = heading_error(navHeading, comp.heading_deg);
            currentSteering = pid.update(deg2rad(err_deg), dt);
        } else {
            currentSteering = 0.0f;
            pid.reset();
        }

        syncActuatorsToState(now);

        if (currentState != prevState) {
            printCurrentStateSummary();
        }
    }

    // Auto-enable LoRa telemetry once ASCENT begins
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
