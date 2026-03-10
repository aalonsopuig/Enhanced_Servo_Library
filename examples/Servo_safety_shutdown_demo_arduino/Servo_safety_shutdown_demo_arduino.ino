/*
===============================================================================
Name:         Servo_safety_shutdown_demo_arduino
Version:      1.0.0
Author:       Alejandro Alonso Puig + GPT
GitHub:       https://github.com/aalonsopuig
Date:         2026-03-10
License:      Apache 2.0
-------------------------------------------------------------------------------
Description:

Example demonstrating how to use the ServoController library with:

- one servo
- analog feedback from the internal potentiometer
- fault detection enabled
- automatic power shutdown through an external MOSFET/FET module if the servo
  is blocked or stops moving while motion is being commanded

Controls:

A0 -> target angle
A1 -> speed percentage
A2 -> acceleration percentage

Feedback:

A4 -> analog feedback from servo internal potentiometer

Servo power switching:

D7 -> digital output controlling an external MOSFET/FET module

Behaviour:

- the sketch powers the servo through the external switch
- it synchronizes the controller to the real servo angle using feedback
- it commands motion from the three potentiometers
- if the servo becomes blocked and the library latches a fault,
  the sketch cuts servo power immediately

Important notes:

1) This example requires a servo with accessible analog feedback.
2) The servo must be powered from an external supply.
3) The MOSFET/FET module is handled by the sketch, not by the library.

===============================================================================
*/

#include <Arduino.h>
#include "ServoController.h"

// ============================================================================
// Pins
// ============================================================================

// User potentiometers
#define POT_TARGET_PIN       A0
#define POT_SPEED_PIN        A1
#define POT_ACCEL_PIN        A2

// Servo feedback analog pin
#define SERVO_FEEDBACK_PIN   A4

// Servo PWM signal pin
#define SERVO_PIN            9

// Digital output used to enable / disable servo power through an external
// MOSFET or FET module.
#define SERVO_POWER_PIN      7

// Set to true if the power module is enabled with HIGH.
// Set to false if the module is enabled with LOW.
#define MOSFET_ACTIVE_HIGH   true

// ============================================================================
// Serial
// ============================================================================

#define BAUDRATE 115200

// ============================================================================
// ADC helpers
// ============================================================================

#define ADC_SCALE   1023.0f
#define POT_SAMPLES 4

// ============================================================================
// Startup timing
// ============================================================================

// Delay after enabling servo power, allowing electronics and feedback signal
// to stabilize before synchronization.
#define POWER_STABILIZE_MS 400

// ============================================================================
// Servo characterization (example: Hitec HS-805BB)
// ============================================================================
//
// Calibrated values measured experimentally:
//
// PWM:
//   700 us  -> 0°
//   2400 us -> 180°
//
// Feedback ADC:
//   101 -> 0°
//   383 -> 180°
//
// Maximum real speed used here:
//   17.5 deg/s
//
#define SERVO_MIN_DEG          0.0f
#define SERVO_MAX_DEG          180.0f

#define PWM_MIN_US             700
#define PWM_MAX_US             2400

#define FB_ADC_MIN             101
#define FB_ADC_MAX             383

#define SERVO_MAX_SPEED_DEGPS  17.5f

// ============================================================================
// Motion defaults
// ============================================================================

#define DEFAULT_SPEED_PCT  50
#define DEFAULT_ACCEL_PCT  50

#define REST_DEG           90.0f

// ============================================================================
// Global objects
// ============================================================================

ServoConfig servoCfg;
ServoController servo;

// Latched application-level shutdown flag.
// Once true, the sketch keeps the servo powered off until reset.
bool shutdownLatched = false;

// ============================================================================
// Utility helpers
// ============================================================================

// Clamp helper
static inline float clampf(float x, float lo, float hi)
{
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// Average multiple ADC readings to reduce noise from the potentiometers
float readAveragedADC(uint8_t pin, int samples)
{
  long sum = 0;

  for (int i = 0; i < samples; i++)
  {
    sum += analogRead(pin);
  }

  return (float)sum / samples;
}

// Convert ADC reading into target angle 0..180°
float targetDegFromAdc(float adc)
{
  float deg = (adc / ADC_SCALE) * 180.0f;
  return clampf(deg, 0.0f, 180.0f);
}

// Convert ADC reading into percentage 1..100
uint8_t percentFromAdc(float adc)
{
  float t = clampf(adc / ADC_SCALE, 0.0f, 1.0f);

  int pct = (int)(1.0f + t * 99.0f + 0.5f);

  if (pct < 1) pct = 1;
  if (pct > 100) pct = 100;

  return (uint8_t)pct;
}

// Helper to control the external servo power switch
void setServoPower(bool on)
{
  if (MOSFET_ACTIVE_HIGH)
  {
    digitalWrite(SERVO_POWER_PIN, on ? HIGH : LOW);
  }
  else
  {
    digitalWrite(SERVO_POWER_PIN, on ? LOW : HIGH);
  }
}

// ============================================================================
// Setup
// ============================================================================

void setup()
{
  Serial.begin(BAUDRATE);
  delay(500);

  // ------------------------------------------------------------
  // Configure the digital output that controls servo power
  // ------------------------------------------------------------
  pinMode(SERVO_POWER_PIN, OUTPUT);

  // Start with power disabled as a safe default
  setServoPower(false);
  delay(100);

  // ------------------------------------------------------------
  // Fill servo configuration
  // ------------------------------------------------------------
  servoCfg.name = "Servo1";

  servoCfg.pwm_pin = SERVO_PIN;

  servoCfg.servo_min_deg = SERVO_MIN_DEG;
  servoCfg.servo_max_deg = SERVO_MAX_DEG;

  servoCfg.allowed_min_deg = SERVO_MIN_DEG;
  servoCfg.allowed_max_deg = SERVO_MAX_DEG;

  servoCfg.rest_deg = REST_DEG;

  servoCfg.pwm_min_us = PWM_MIN_US;
  servoCfg.pwm_max_us = PWM_MAX_US;

  servoCfg.max_speed_degps = SERVO_MAX_SPEED_DEGPS;

  servoCfg.default_speed_pct = DEFAULT_SPEED_PCT;
  servoCfg.default_accel_pct = DEFAULT_ACCEL_PCT;

  // Feedback is enabled in this example
  servoCfg.feedback_adc_pin = SERVO_FEEDBACK_PIN;
  servoCfg.fb_adc_at_servo_min_deg = FB_ADC_MIN;
  servoCfg.fb_adc_at_servo_max_deg = FB_ADC_MAX;

  servoCfg.inverted = false;

  // Important: enable fault detection
  servoCfg.fault_detection_enabled = true;

  // ------------------------------------------------------------
  // Initialize the library object
  // ------------------------------------------------------------
  servo.begin(servoCfg);

  // ------------------------------------------------------------
  // Power on the servo and let the electronics stabilize
  // ------------------------------------------------------------
  setServoPower(true);
  delay(POWER_STABILIZE_MS);

  // ------------------------------------------------------------
  // Synchronize internal library state to the real measured angle
  // ------------------------------------------------------------
  servo.syncToFeedback();

  Serial.println("Servo safety shutdown demo");
  Serial.println("Target | Cmd | Meas | Err | Speed% | Accel% | Fault | Pwr");
}

// ============================================================================
// Main loop
// ============================================================================

void loop()
{
  // ------------------------------------------------------------
  // If shutdown is already latched, keep the servo powered off
  // ------------------------------------------------------------
  //
  // This makes the shutdown behaviour deterministic and safe:
  // once the servo is considered faulty, the sketch does not
  // automatically re-enable it.
  if (shutdownLatched)
  {
    setServoPower(false);

    // Still print state slowly so the user knows what happened
    Serial.print("--- | --- | ");
    Serial.print(servo.getMeasuredDeg(), 1);
    Serial.print(" | --- | --- | --- | 1 | 0");
    Serial.println();

    delay(200);
    return;
  }

  // ------------------------------------------------------------
  // Read user potentiometers
  // ------------------------------------------------------------
  float adcTarget = readAveragedADC(POT_TARGET_PIN, POT_SAMPLES);
  float adcSpeed  = readAveragedADC(POT_SPEED_PIN,  POT_SAMPLES);
  float adcAccel  = readAveragedADC(POT_ACCEL_PIN,  POT_SAMPLES);

  float   targetDeg = targetDegFromAdc(adcTarget);
  uint8_t speedPct  = percentFromAdc(adcSpeed);
  uint8_t accelPct  = percentFromAdc(adcAccel);

  // ------------------------------------------------------------
  // Send motion command to the library
  // ------------------------------------------------------------
  servo.setTarget(targetDeg, speedPct, accelPct);

  // ------------------------------------------------------------
  // Advance controller one step
  // ------------------------------------------------------------
  servo.update();

  // ------------------------------------------------------------
  // If the library has detected a fault, cut servo power immediately
  // ------------------------------------------------------------
  if (servo.hasFault())
  {
    shutdownLatched = true;
    setServoPower(false);
  }

  // ------------------------------------------------------------
  // Read back values for display
  // ------------------------------------------------------------
  float cmd  = servo.getCommandDeg();
  float meas = servo.getMeasuredDeg();
  float err  = cmd - meas;

  // ------------------------------------------------------------
  // Print telemetry
  // ------------------------------------------------------------
  Serial.print(targetDeg, 1);
  Serial.print(" | ");

  Serial.print(cmd, 1);
  Serial.print(" | ");

  Serial.print(meas, 1);
  Serial.print(" | ");

  Serial.print(err, 1);
  Serial.print(" | ");

  Serial.print(speedPct);
  Serial.print(" | ");

  Serial.print(accelPct);
  Serial.print(" | ");

  Serial.print(servo.hasFault() ? 1 : 0);
  Serial.print(" | ");

  Serial.println(shutdownLatched ? 0 : 1);

  delay(20);
}
