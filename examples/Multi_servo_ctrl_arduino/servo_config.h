/*
===============================================================================
File:         servo_config.h
Author:       Alejandro Alonso Puig + GPT
Date:         2026-03-10
License:      Apache 2.0
-------------------------------------------------------------------------------

Description:

This file contains the hardware configuration of the servos used in the
Multi_servo_ctrl_arduino example.

Separating configuration from the main sketch allows the user to modify
servo parameters without touching the application logic.

Typical parameters that belong here:

- PWM pins
- servo angle limits
- allowed motion limits
- servo speed characterization
- PWM calibration
- feedback configuration (if used)

In real robotic systems, this file typically represents the mechanical
characterization of the robot joints.

===============================================================================
*/

#ifndef SERVO_CONFIG_H
#define SERVO_CONFIG_H

#include "ServoController.h"


// ============================================================================
// Number of servos used in this example
// ============================================================================

#define NUM_SERVOS 9


// ============================================================================
// Servo PWM pins
// ============================================================================
//
// These pins correspond to the PWM signals sent to each servo.
// Modify them according to your wiring.
//

static const uint8_t SERVO_PINS[NUM_SERVOS] =
{
  2,3,4,5,6,7,8,9,10
};


// ============================================================================
// Servo names
// ============================================================================
//
// Names are optional but useful for debugging or telemetry.
//

static const char* SERVO_NAMES[NUM_SERVOS] =
{
  "S1","S2","S3","S4","S5","S6","S7","S8","S9"
};


// ============================================================================
// Servo characterization
// ============================================================================
//
// This section describes the physical behaviour of the servo type used.
//
// Example used here: Futaba S3003
//
// Typical speed:
// 0.23 s / 60°
//
// Converted:
//
// 60 / 0.23 ≈ 261 deg/s
//

#define SERVO_MAX_SPEED_DEGPS 261.0f


// ============================================================================
// PWM calibration
// ============================================================================
//
// Generic safe range used by many hobby servos.
//

#define PWM_MIN_US 1000
#define PWM_MAX_US 2000


// ============================================================================
// Servo mechanical limits
// ============================================================================
//
// These correspond to the full calibrated range of the servo.
//

#define SERVO_MIN_DEG 0.0f
#define SERVO_MAX_DEG 180.0f


// ============================================================================
// Allowed motion limits
// ============================================================================
//
// These limits define the range allowed by the application.
// They may be narrower than the physical limits.
//

#define ALLOWED_MIN_DEG 0.0f
#define ALLOWED_MAX_DEG 180.0f


// ============================================================================
// Default motion parameters
// ============================================================================

#define DEFAULT_SPEED_PCT 50
#define DEFAULT_ACCEL_PCT 50


// ============================================================================
// Rest position
// ============================================================================

#define REST_DEG 90.0f


// ============================================================================
// Servo configuration table
// ============================================================================
//
// This function fills a ServoConfig structure for one servo.
// All servos share the same parameters in this example.
//

inline ServoConfig makeServoConfig(const char* name,uint8_t pin)
{
    ServoConfig cfg;

    cfg.name = name;
    cfg.pwm_pin = pin;

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

    // No feedback used in this example
    cfg.feedback_adc_pin = -1;
    cfg.fb_adc_at_servo_min_deg = 0;
    cfg.fb_adc_at_servo_max_deg = 0;

    cfg.inverted = false;
    cfg.fault_detection_enabled = false;

    return cfg;
}

#endif