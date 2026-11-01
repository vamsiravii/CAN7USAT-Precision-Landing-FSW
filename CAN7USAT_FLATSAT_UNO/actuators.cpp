#include "actuators.h"
#include <Arduino.h>

#if defined(__AVR__)
#include <avr/interrupt.h>

static volatile uint16_t g_avrPulseTicks[3] = { 0, 0, 2000 }; // [0]=Left(Pin 3), [1]=Right(Pin 2), [2]=ESC(Pin 6) in 0.5us ticks
static volatile uint8_t  g_avrServoSlot = 0;
static volatile uint16_t g_avrFrameTicksUsed = 0;
static bool              g_avrTimerInitialized = false;

ISR(TIMER1_COMPA_vect) {
    uint16_t nowTicks = TCNT1;
    if (g_avrServoSlot == 0) {
        g_avrFrameTicksUsed = 0;
        uint16_t ticks = g_avrPulseTicks[0];
        if (ticks > 0) {
            digitalWrite(SERVO_LEFT_PIN, HIGH);
        } else {
            digitalWrite(SERVO_LEFT_PIN, LOW);
            ticks = 2000;
        }
        g_avrFrameTicksUsed += ticks;
        OCR1A = nowTicks + ticks;
        g_avrServoSlot = 1;
    } else if (g_avrServoSlot == 1) {
        digitalWrite(SERVO_LEFT_PIN, LOW);
        uint16_t ticks = g_avrPulseTicks[1];
        if (ticks > 0) {
            digitalWrite(SERVO_RIGHT_PIN, HIGH);
        } else {
            digitalWrite(SERVO_RIGHT_PIN, LOW);
            ticks = 2000;
        }
        g_avrFrameTicksUsed += ticks;
        OCR1A = nowTicks + ticks;
        g_avrServoSlot = 2;
    } else if (g_avrServoSlot == 2) {
        digitalWrite(SERVO_RIGHT_PIN, LOW);
        uint16_t ticks = g_avrPulseTicks[2];
        if (ticks > 0) {
            digitalWrite(FLYWHEEL_ESC_PIN, HIGH);
        } else {
            digitalWrite(FLYWHEEL_ESC_PIN, LOW);
            ticks = 2000;
        }
        g_avrFrameTicksUsed += ticks;
        OCR1A = nowTicks + ticks;
        g_avrServoSlot = 3;
    } else {
        digitalWrite(FLYWHEEL_ESC_PIN, LOW);
        uint16_t remTicks = (40000U > g_avrFrameTicksUsed) ? (40000U - g_avrFrameTicksUsed) : 4000U;
        OCR1A = nowTicks + remTicks;
        g_avrServoSlot = 0;
    }
}

static void initAvrServoTimer1() {
    if (g_avrTimerInitialized) return;
    pinMode(SERVO_LEFT_PIN, OUTPUT);
    digitalWrite(SERVO_LEFT_PIN, LOW);
    pinMode(SERVO_RIGHT_PIN, OUTPUT);
    digitalWrite(SERVO_RIGHT_PIN, LOW);
    pinMode(FLYWHEEL_ESC_PIN, OUTPUT);
    digitalWrite(FLYWHEEL_ESC_PIN, LOW);

    uint8_t oldSREG = SREG;
    cli();
    TCCR1A = 0;
    TCCR1B = (1 << CS11);     // Prescaler = 8 (16 MHz / 8 = 2 MHz -> 0.5 us per tick)
    TCNT1  = 0;
    OCR1A  = 1000;
    TIFR1  = (1 << OCF1A);
    TIMSK1 |= (1 << OCIE1A);
    SREG = oldSREG;
    g_avrTimerInitialized = true;
}

static void writeHardwarePwmUs(uint8_t pin, uint16_t pulse_us) {
    if (!g_avrTimerInitialized) {
        initAvrServoTimer1();
    }
    uint16_t ticks = (pulse_us == 0) ? 0 : (pulse_us * 2U);
    uint8_t oldSREG = SREG;
    cli();
    if (pin == SERVO_LEFT_PIN) {
        g_avrPulseTicks[0] = ticks;
        if (ticks == 0) digitalWrite(SERVO_LEFT_PIN, LOW);
    } else if (pin == SERVO_RIGHT_PIN) {
        g_avrPulseTicks[1] = ticks;
        if (ticks == 0) digitalWrite(SERVO_RIGHT_PIN, LOW);
    } else if (pin == FLYWHEEL_ESC_PIN) {
        g_avrPulseTicks[2] = ticks;
        if (ticks == 0) digitalWrite(FLYWHEEL_ESC_PIN, LOW);
    }
    SREG = oldSREG;
}
#else
static void writeHardwarePwmUs(uint8_t pin, uint16_t pulse_us) {
    if (pulse_us == 0) {
        analogWrite(pin, 0);
        return;
    }
    uint32_t duty = ((uint32_t)pulse_us * 65536UL + 10000UL) / 20000UL;
    if (duty > 65535UL) duty = 65535UL;
    analogWrite(pin, (int)duty);
}
#endif

static inline float clampf(float val, float lo, float hi) {
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

static uint32_t g_leftBrakeCutMs  = 0;
static uint32_t g_rightBrakeCutMs = 0;

static void writeServoWithInstantStop(uint8_t pin, uint16_t target_us, uint16_t prev_us, uint32_t now_ms) {
    uint32_t &cutMs = (pin == SERVO_LEFT_PIN) ? g_leftBrakeCutMs : g_rightBrakeCutMs;
    if (target_us == 0 || target_us == ES3004_NEUTRAL_PULSE_US) {
        if (prev_us != 0 && prev_us != ES3004_NEUTRAL_PULSE_US) {
            writeHardwarePwmUs(pin, ES3004_NEUTRAL_PULSE_US);
            cutMs = (now_ms ? now_ms : 1UL) + 65UL;
        } else if (cutMs == 0 || (int32_t)(now_ms - cutMs) >= 0) {
            cutMs = 0;
            writeHardwarePwmUs(pin, 0);
        }
    } else {
        cutMs = 0;
        writeHardwarePwmUs(pin, target_us);
    }
}

ActuatorManager::ActuatorManager()
    : flywheel_pulse_us(ESC_MIN_PULSE_US) {}

void ActuatorManager::init() {
    pinMode(BURNWIRE_PIN, OUTPUT);
    digitalWrite(BURNWIRE_PIN, LOW);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    writeHardwarePwmUs(SERVO_LEFT_PIN, 0);
    writeHardwarePwmUs(SERVO_RIGHT_PIN, 0);
    flywheel_pulse_us = ESC_MIN_PULSE_US;
    writeHardwarePwmUs(FLYWHEEL_ESC_PIN, ESC_MIN_PULSE_US);
}

bool ActuatorManager::isFlightStateSteeringPermitted(FlightState state) {
    return (state == FlightState::PRECISION_GUIDANCE ||
            state == FlightState::TERMINAL_GUIDANCE ||
            state == FlightState::FINAL_APPROACH);
}

void ActuatorManager::applySteering(
    float steering,
    bool enabled,
    uint32_t now_ms,
    FlightState flight_state
) {
    if (!enabled || !isFlightStateSteeringPermitted(flight_state)) {
        requested_steering_effort = 0.0f;
        left_brake_norm  = 0.0f;
        right_brake_norm = 0.0f;
        writeServoWithInstantStop(SERVO_LEFT_PIN, 0, left_pulse_us, now_ms);
        writeServoWithInstantStop(SERVO_RIGHT_PIN, 0, right_pulse_us, now_ms);
        left_pulse_us    = 0;
        right_pulse_us   = 0;
        return;
    }

    if (!isfinite(steering)) steering = 0.0f;
    float s = clampf(steering, -1.0f, 1.0f);
    requested_steering_effort = s;

    left_brake_norm  = (s < -0.02f) ? -s : 0.0f;
    right_brake_norm = (s >  0.02f) ?  s : 0.0f;

    float lAng = SERVO_NEUTRAL_ANGLE + left_brake_norm  * (SERVO_MAX_BRAKE_ANGLE - SERVO_NEUTRAL_ANGLE);
    float rAng = SERVO_NEUTRAL_ANGLE + right_brake_norm * (SERVO_MAX_BRAKE_ANGLE - SERVO_NEUTRAL_ANGLE);

    float lRatio = clampf(lAng / 90.0f, 0.0f, 1.0f);
    float rRatio = clampf(rAng / 90.0f, 0.0f, 1.0f);

    uint16_t new_left_us  = (left_brake_norm  > 0.0f) ? (uint16_t)(ES3004_MIN_PULSE_US + lRatio * (ES3004_NEUTRAL_PULSE_US - ES3004_MIN_PULSE_US)) : 0;
    uint16_t new_right_us = (right_brake_norm > 0.0f) ? (uint16_t)(ES3004_MIN_PULSE_US + rRatio * (ES3004_NEUTRAL_PULSE_US - ES3004_MIN_PULSE_US)) : 0;

    writeServoWithInstantStop(SERVO_LEFT_PIN, new_left_us, left_pulse_us, now_ms);
    writeServoWithInstantStop(SERVO_RIGHT_PIN, new_right_us, right_pulse_us, now_ms);
    left_pulse_us  = new_left_us;
    right_pulse_us = new_right_us;
}

void ActuatorManager::update(uint32_t now_ms) {
    if (g_leftBrakeCutMs != 0 && (int32_t)(now_ms - g_leftBrakeCutMs) >= 0) {
        g_leftBrakeCutMs = 0;
        writeHardwarePwmUs(SERVO_LEFT_PIN, 0);
    }
    if (g_rightBrakeCutMs != 0 && (int32_t)(now_ms - g_rightBrakeCutMs) >= 0) {
        g_rightBrakeCutMs = 0;
        writeHardwarePwmUs(SERVO_RIGHT_PIN, 0);
    }
}

void ActuatorManager::setDirectPulse(uint16_t left_us, uint16_t right_us) {
    uint32_t now_ms = millis();
    writeServoWithInstantStop(SERVO_LEFT_PIN, left_us, left_pulse_us, now_ms);
    writeServoWithInstantStop(SERVO_RIGHT_PIN, right_us, right_pulse_us, now_ms);
    left_pulse_us  = left_us;
    right_pulse_us = right_us;
}

void ActuatorManager::setBurnwire(bool active) {
    burnwire_state = active;
    digitalWrite(BURNWIRE_PIN, active ? HIGH : LOW);
}

void ActuatorManager::setFlywheel(bool active) {
    flywheel_state = active;
    if (!active) {
        flywheel_pulse_us = ESC_MIN_PULSE_US;
        writeHardwarePwmUs(FLYWHEEL_ESC_PIN, ESC_MIN_PULSE_US);
    } else {
        uint32_t now = millis();
        if (flywheel_pulse_us < 1060) {
            flywheel_pulse_us = 1060;
            last_flywheel_ramp_ms = now;
        } else if (now - last_flywheel_ramp_ms >= 80UL) {
            last_flywheel_ramp_ms = now;
            flywheel_pulse_us += 8;
            if (flywheel_pulse_us > ESC_TARGET_6000RPM_US) {
                flywheel_pulse_us = ESC_TARGET_6000RPM_US;
            }
        }
        writeHardwarePwmUs(FLYWHEEL_ESC_PIN, flywheel_pulse_us);
    }
}

void ActuatorManager::updateBuzzer(bool active, uint32_t now_ms) {
    if (!active) {
        digitalWrite(BUZZER_PIN, LOW);
        buzzer_state = false;
        return;
    }
    if (now_ms - last_buzzer_toggle_ms >= 500UL) {
        last_buzzer_toggle_ms = now_ms;
        buzzer_state = !buzzer_state;
        digitalWrite(BUZZER_PIN, buzzer_state ? HIGH : LOW);
    }
}
