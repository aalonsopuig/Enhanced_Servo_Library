/*
===============================================================================
Name:         Simple_servo_ctrl
Author:       Alejandro Alonso Puig + GPT
Date:         2026-03-10
License:      Apache 2.0
-------------------------------------------------------------------------------
Description:

Minimal example showing how to use the ServoController library with:

- One standard hobby servo (no feedback)
- One target potentiometer
- One speed potentiometer
- One acceleration potentiometer

The sketch reads the three user potentiometers, converts them into:

- target angle
- speed percentage
- acceleration percentage

and sends those values to a single ServoController instance.

Hardware:

- Servo signal pin  -> D9
- Target pot        -> A0
- Speed pot         -> A1
- Acceleration pot  -> A2

Important note:

This example does NOT use servo feedback. Therefore:

- the library runs in open-loop mode
- current position is estimated internally from the commanded reference
- fault detection based on feedback is disabled

===============================================================================
*/

#include <Arduino.h>
#include "ServoController.h"

// ============================================================================
// User wiring
// ============================================================================

// Servo PWM output pin.
// Connect this to the signal wire of the servo.
#define SERVO_PIN        9

// Potentiometer used to command target angle.
// Expected wiring: ends to 5V and GND, wiper to A0.
#define POT_TARGET_PIN   A0

// Potentiometer used to command speed percentage (1..100 %).
// Expected wiring: ends to 5V and GND, wiper to A1.
#define POT_VMAX_PIN     A1

// Potentiometer used to command acceleration percentage (1..100 %).
// Expected wiring: ends to 5V and GND, wiper to A2.
#define POT_ACCEL_PIN    A2

// ============================================================================
// Serial monitor
// ============================================================================

#define BAUDRATE         115200

// ============================================================================
// ADC constants
// ============================================================================

// Arduino Uno ADC full-scale value.
#define ADC_SCALE        1023.0f

// Number of samples used when averaging user potentiometers.
// Small averaging helps reduce noise and makes knob behaviour smoother.
#define POT_SAMPLES      4

// ============================================================================
// Servo example configuration
// ============================================================================

// This example assumes a standard servo such as a Futaba S3003 at 4.8 V.
//
// Typical datasheet speed:
// 0.23 s / 60°  ->  60 / 0.23 = 261 deg/s
//
// This is the PHYSICAL MAXIMUM SPEED of the actuator used by the library
// as reference for speed percentages.
// If you use another servo or change supply voltage, update this value.
#define SERVO_MAX_SPEED_DEGPS  261.0f

// ---- Servo PWM calibration ----
// These define how the 0..180° command maps to microseconds.
// Adjust to match YOUR servo endpoints safely.
#define SERVO_CENTER_US  1150
#define SERVO_HALFSPAN_US  650

#define PWM_MIN_US  (SERVO_CENTER_US - SERVO_HALFSPAN_US)   // 1000 us
#define PWM_MAX_US  (SERVO_CENTER_US + SERVO_HALFSPAN_US)   // 2000 us

// ============================================================================
// Motion-profile defaults
// ============================================================================

// Default speed and acceleration percentages used only at startup.
// They will immediately be overwritten by the potentiometers in loop().
#define DEFAULT_SPEED_PCT      50
#define DEFAULT_ACCEL_PCT      50

// ============================================================================
// Servo logical angle ranges
// ============================================================================

// In this simple example we assume the servo is calibrated for 0..180°,
// and we also allow the full 0..180° as the application motion range.
//
// In more advanced applications these two ranges do not have to be equal:
// - servo_min_deg / servo_max_deg   -> calibrated physical range
// - allowed_min_deg / allowed_max_deg -> safe application range
#define SERVO_MIN_DEG          0.0f
#define SERVO_MAX_DEG          180.0f

#define ALLOWED_MIN_DEG        0.0f
#define ALLOWED_MAX_DEG        180.0f

#define REST_DEG               90.0f

// ============================================================================
// Global objects
// ============================================================================

// Single servo configuration structure used to initialize the library object.
//
// Since this example has NO FEEDBACK:
// - feedback_adc_pin = -1
// - feedback calibration values are set to 0 and ignored
// - fault detection is disabled
ServoConfig servoCfg = {
    "SERVO_1",               // name
    SERVO_PIN,               // pwm_pin

    SERVO_MIN_DEG,           // servo_min_deg
    SERVO_MAX_DEG,           // servo_max_deg

    ALLOWED_MIN_DEG,         // allowed_min_deg
    ALLOWED_MAX_DEG,         // allowed_max_deg

    REST_DEG,                // rest_deg

    PWM_MIN_US,              // pwm_min_us
    PWM_MAX_US,              // pwm_max_us

    SERVO_MAX_SPEED_DEGPS,   // max_speed_degps

    DEFAULT_SPEED_PCT,       // default_speed_pct
    DEFAULT_ACCEL_PCT,       // default_accel_pct

    -1,                      // feedback_adc_pin -> no feedback
    0,                       // fb_adc_at_servo_min_deg (unused here)
    0,                       // fb_adc_at_servo_max_deg (unused here)

    false,                   // inverted
    false                    // fault_detection_enabled
};

// Main servo controller instance.
ServoController servo1;

// ============================================================================
// Helper functions
// ============================================================================

// Clamp helper for floats.
static inline float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

// Read one analog pin several times and return the average.
// This reduces visible jitter on target/speed/acceleration control.
float readAveragedADC(uint8_t pin, int samples) {
    long sum = 0;
    for (int i = 0; i < samples; i++) {
        sum += analogRead(pin);
    }
    return (float)sum / samples;
}

// Convert an ADC value (0..1023) into target angle (0..180°).
float targetDegFromAdc(float adc) {
    float deg = (adc / ADC_SCALE) * 180.0f;
    return clampf(deg, 0.0f, 180.0f);
}

// Convert an ADC value (0..1023) into integer percent (1..100).
//
// We intentionally use the full potentiometer travel for the entire percentage
// range so that the knob always feels useful.
uint8_t percentFromAdc(float adc) {
    float t = clampf(adc / ADC_SCALE, 0.0f, 1.0f);
    int pct = (int)(1.0f + t * 99.0f + 0.5f);  // rounded to nearest integer
    if (pct < 1) pct = 1;
    if (pct > 100) pct = 100;
    return (uint8_t)pct;
}

// Print a fixed-width telemetry row to keep Serial Monitor columns aligned.
void printTelemetry(float targetDeg, uint8_t speedPct, uint8_t accelPct) {
    char a[12], b[12], c[12], d[12], e[12];

    dtostrf(targetDeg,               6, 1, a);
    dtostrf(servo1.getCommandDeg(),  6, 1, b);
    dtostrf(servo1.getCurrentDeg(),  6, 1, c);
    dtostrf(servo1.getVelocityDegps(), 6, 1, d);
    dtostrf(servo1.getErrorDeg(),    6, 1, e);

    // Columns:
    // Target | Cmd | Curr | Err | V | V% | A% | PWM | State
    Serial.print(a); Serial.print(" | ");
    Serial.print(b); Serial.print(" | ");
    Serial.print(c); Serial.print(" | ");
    Serial.print(e); Serial.print(" | ");
    Serial.print(d); Serial.print(" | ");

    if (speedPct < 100) Serial.print(' ');
    if (speedPct < 10)  Serial.print(' ');
    Serial.print(speedPct); Serial.print("% | ");

    if (accelPct < 100) Serial.print(' ');
    if (accelPct < 10)  Serial.print(' ');
    Serial.print(accelPct); Serial.print("% | ");

    if (servo1.getPwmUs() < 1000) Serial.print(' ');
    Serial.print(servo1.getPwmUs()); Serial.print(" | ");

    switch (servo1.getState()) {
        case ServoState::UNINITIALIZED: Serial.println("UNINITIALIZED"); break;
        case ServoState::IDLE:          Serial.println("IDLE");          break;
        case ServoState::MOVING:        Serial.println("MOVING");        break;
        case ServoState::HOLDING:       Serial.println("HOLDING");       break;
        case ServoState::FAULT:         Serial.println("FAULT");         break;
        default:                        Serial.println("UNKNOWN");       break;
    }
}

// ============================================================================
// Setup
// ============================================================================

void setup() {
    // Start serial communication for telemetry.
    Serial.begin(BAUDRATE);

    // Short delay so Serial Monitor has time to connect comfortably.
    delay(500);

    // Initialize the servo controller with the static configuration.
    servo1.begin(servoCfg);

    // Since this example has no feedback, we synchronize the internal state
    // to the configured rest angle.
    //
    // This means:
    // - target = rest
    // - command = rest
    // - current = rest
    // - velocity = 0
    //
    // and the servo starts from a well-defined internal state.
    servo1.syncToAngle(REST_DEG);

    // Print header.
    Serial.println("Simple_servo_ctrl");
    Serial.println("Target |   Cmd |  Curr |   Err |     V |  V% |  A% |  PWM | State");
    Serial.println("------------------------------------------------------------------------");
}

// ============================================================================
// Loop
// ============================================================================

void loop() {
    // ------------------------------------------------------------------------
    // 1) Read user inputs
    // ------------------------------------------------------------------------
    //
    // We read:
    // - target potentiometer
    // - speed potentiometer
    // - acceleration potentiometer
    //
    // using a small average to reduce visible jitter.
    float adcTarget = readAveragedADC(POT_TARGET_PIN, POT_SAMPLES);
    float adcSpeed  = readAveragedADC(POT_VMAX_PIN,   POT_SAMPLES);
    float adcAccel  = readAveragedADC(POT_ACCEL_PIN,  POT_SAMPLES);

    // Convert raw ADC readings into user-friendly variables.
    float   targetDeg = targetDegFromAdc(adcTarget);
    uint8_t speedPct  = percentFromAdc(adcSpeed);
    uint8_t accelPct  = percentFromAdc(adcAccel);

    // ------------------------------------------------------------------------
    // 2) Send motion command to the library
    // ------------------------------------------------------------------------
    //
    // This example is intentionally very simple:
    // every loop we refresh the target and active motion parameters from the
    // three potentiometers.
    //
    // Note:
    // the first minimal library version we wrote only guaranteed setTarget(angle),
    // but the class design already contemplates the overload with percentages.
    //
    // For this example of public use, we call the explicit variant so that the
    // knobs immediately control both speed and acceleration.
    servo1.setTarget(targetDeg, speedPct, accelPct);

    // ------------------------------------------------------------------------
    // 3) Advance the controller one step
    // ------------------------------------------------------------------------
    //
    // update() is the heart of the library:
    // - computes dt
    // - updates motion profile
    // - computes PWM
    // - writes PWM to the servo
    // - updates current position estimate
    servo1.update();

    // ------------------------------------------------------------------------
    // 4) Print telemetry
    // ------------------------------------------------------------------------
    //
    // Since this example has no feedback:
    // - Curr is estimated position
    // - Err is mostly the internal tracking relation of the reference model
    // - FAULT is not expected because fault detection is disabled
    printTelemetry(targetDeg, speedPct, accelPct);

    // Small delay to keep the serial output readable and avoid flooding the PC.
    delay(20);
}
