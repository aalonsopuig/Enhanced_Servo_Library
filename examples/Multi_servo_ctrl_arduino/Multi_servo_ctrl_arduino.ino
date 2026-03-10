/*
===============================================================================
Name:         Multi_servo_ctrl_arduino
Version:      1.0.0
Author:       Alejandro Alonso Puig + GPT
GitHub:       https://github.com/aalonsopuig
Date:         2026-03-10
License:      Apache 2.0
-------------------------------------------------------------------------------
Description:

Didactic example showing how to control multiple servos using the
ServoController library.

This sketch controls 9 standard hobby servos, all configured as generic
Futaba S3003-like servos, without feedback.

All servos receive the SAME:

- target angle
- speed percentage
- acceleration percentage

These values are read from three potentiometers:

A0 -> target angle
A1 -> speed percentage
A2 -> acceleration percentage

This example demonstrates the typical usage pattern of the library:

1) Create a configuration table (ServoConfig)
2) Create an array of ServoController objects
3) Initialize each servo in setup()
4) Send commands to all servos
5) Call update() periodically for each servo

Hardware note:
Servos must be powered from an external power supply.
Do NOT power multiple servos from the Arduino 5V pin.

===============================================================================
*/

#include <Arduino.h>
#include "ServoController.h"


// ============================================================================
// General configuration
// ============================================================================

// Number of servos used in this example.
// The example intentionally uses many servos to demonstrate that the
// library scales easily to multi-servo robotic systems.
#define NUM_SERVOS 9


// ============================================================================
// Pins
// ============================================================================

// Analog inputs used for the three control potentiometers.
#define POT_TARGET_PIN   A0   // Target position (0–180 degrees)
#define POT_SPEED_PIN    A1   // Speed percentage (1–100%)
#define POT_ACCEL_PIN    A2   // Acceleration percentage (1–100%)

// Digital pins used to drive the servo PWM signals.
// On most Arduino boards the Servo library can drive many pins.
const uint8_t SERVO_PINS[NUM_SERVOS] =
{
  2,3,4,5,6,7,8,9,10
};


// ============================================================================
// ADC helpers
// ============================================================================

// ADC scale for Arduino (10-bit ADC)
#define ADC_SCALE   1023.0f

// Number of samples used to average potentiometer readings.
// Averaging reduces visible jitter in the knobs.
#define POT_SAMPLES 4


// ============================================================================
// Servo characterization (Futaba S3003 reference)
// ============================================================================
//
// A typical Futaba S3003 servo at 4.8 V has a speed of:
//
// 0.23 seconds / 60°
//
// Converting to angular velocity:
//
// 60 / 0.23 ≈ 261 deg/s
//
// This value is used by the library as the physical maximum speed of
// the servo. Speed percentages are applied relative to this value.
//
#define SERVO_MAX_SPEED_DEGPS 261.0f


// ============================================================================
// PWM calibration
// ============================================================================
//
// For a generic example we assume:
//
// 1000 µs -> minimum angle
// 2000 µs -> maximum angle
//
// Many hobby servos accept roughly this range.
//
#define SERVO_CENTER_US    1500
#define SERVO_HALFSPAN_US   500

#define PWM_MIN_US (SERVO_CENTER_US - SERVO_HALFSPAN_US)
#define PWM_MAX_US (SERVO_CENTER_US + SERVO_HALFSPAN_US)


// ============================================================================
// Angle ranges
// ============================================================================
//
// servo_min_deg / servo_max_deg
//     Physical calibrated limits of the servo.
//
// allowed_min_deg / allowed_max_deg
//     Limits imposed by the application. These can be narrower than
//     the servo's mechanical limits.
//
#define SERVO_MIN_DEG   0.0f
#define SERVO_MAX_DEG   180.0f

#define ALLOWED_MIN_DEG 0.0f
#define ALLOWED_MAX_DEG 180.0f


// Rest position used at initialization.
// Servos are synchronized to this angle when the program starts.
#define REST_DEG 90.0f


// Default motion parameters used during initialization.
// They are quickly replaced by the potentiometer values in loop().
#define DEFAULT_SPEED_PCT  50
#define DEFAULT_ACCEL_PCT  50


// ============================================================================
// Global objects
// ============================================================================

// Configuration table describing each servo.
ServoConfig servoTable[NUM_SERVOS];

// One controller object per servo.
ServoController servos[NUM_SERVOS];


// ============================================================================
// Helper functions
// ============================================================================

// Clamp a float value to a given range.
static inline float clampf(float x, float lo, float hi)
{
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}


// Read an analog input several times and return the average value.
// This simple filtering reduces small ADC noise and knob jitter.
float readAveragedADC(uint8_t pin, int samples)
{
  long sum = 0;

  for (int i=0;i<samples;i++)
  {
    sum += analogRead(pin);
  }

  return (float)sum / samples;
}


// Convert ADC reading to target angle (0–180 degrees).
float targetDegFromAdc(float adc)
{
  float deg = (adc / ADC_SCALE) * 180.0f;

  return clampf(deg,0.0f,180.0f);
}


// Convert ADC reading to a percentage in the range 1–100.
uint8_t percentFromAdc(float adc)
{
  float t = clampf(adc / ADC_SCALE,0.0f,1.0f);

  int pct = (int)(1.0f + t*99.0f + 0.5f);

  if (pct < 1) pct = 1;
  if (pct > 100) pct = 100;

  return (uint8_t)pct;
}


// Helper function that builds a ServoConfig structure for one servo.
// In this example all servos share identical parameters except for
// their PWM pin and logical name.
ServoConfig makeServoConfig(const char* name,uint8_t pwmPin)
{
  ServoConfig cfg;

  cfg.name = name;
  cfg.pwm_pin = pwmPin;

  cfg.servo_min_deg = SERVO_MIN_DEG;
  cfg.servo_max_deg = SERVO_MAX_DEG;

  cfg.allowed_min_deg = ALLOWED_MIN_DEG;
  cfg.allowed_max_deg = ALLOWED_MAX_DEG;

  cfg.rest_deg = REST_DEG;

  cfg.pwm_min_us = PWM_MIN_US;
  cfg.pwm_max_us = PWM_MAX_US;

  cfg.max_speed_degps = SERVO_MAX_SPEED_DEGPS;

  cfg.default_speed_pct = DEFAULT_SPEED_PCT;
  cfg.default_accel_pct = DEFAULT_ACCEL_PCT;

  // No feedback is used in this example.
  cfg.feedback_adc_pin = -1;
  cfg.fb_adc_at_servo_min_deg = 0;
  cfg.fb_adc_at_servo_max_deg = 0;

  cfg.inverted = false;

  // Fault detection disabled since we have no feedback.
  cfg.fault_detection_enabled = false;

  return cfg;
}


// ============================================================================
// Setup
// ============================================================================

void setup()
{
  // Logical names used for the servos.
  // These are only used for debugging or future extensions.
  const char* names[NUM_SERVOS] =
  {
    "S1","S2","S3","S4","S5","S6","S7","S8","S9"
  };

  // Initialize each servo controller.
  // This loop demonstrates the typical multi-servo initialization pattern.
  for(int i=0;i<NUM_SERVOS;i++)
  {
    // Build configuration entry
    servoTable[i] = makeServoConfig(names[i],SERVO_PINS[i]);

    // Initialize controller
    servos[i].begin(servoTable[i]);

    // Synchronize software state with the physical servo position.
    // Since we do not have feedback, we assume the servo is already
    // at the rest position.
    servos[i].syncToAngle(REST_DEG);
  }
}


// ============================================================================
// Main loop
// ============================================================================

void loop()
{
  // Read control potentiometers
  float adcTarget = readAveragedADC(POT_TARGET_PIN,POT_SAMPLES);
  float adcSpeed  = readAveragedADC(POT_SPEED_PIN,POT_SAMPLES);
  float adcAccel  = readAveragedADC(POT_ACCEL_PIN,POT_SAMPLES);

  // Convert ADC readings into motion parameters
  float   targetDeg = targetDegFromAdc(adcTarget);
  uint8_t speedPct  = percentFromAdc(adcSpeed);
  uint8_t accelPct  = percentFromAdc(adcAccel);

  // Send the same command to all servos
  for(int i=0;i<NUM_SERVOS;i++)
  {
    servos[i].setTarget(targetDeg,speedPct,accelPct);
  }

  // Update motion of each servo controller
  for(int i=0;i<NUM_SERVOS;i++)
  {
    servos[i].update();
  }

  // Small delay to limit loop frequency
  delay(20);
}
