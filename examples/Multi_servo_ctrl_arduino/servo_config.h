/*
===============================================================================
File:         servo_config.h
Author:       Alejandro Alonso Puig + GPT
Date:         2026-03-10
License:      Apache 2.0
-------------------------------------------------------------------------------

Description:

This file defines the complete configuration table for all servos used
in the Multi_servo_ctrl_arduino example.

Each servo has its own explicit configuration entry.

Even if several servos share the same parameters, defining them
individually makes the example easier to adapt to real robotic systems
where every joint may have different:

- PWM calibration
- mechanical limits
- speed characteristics
- feedback configuration

Users typically modify only this file to adapt the example to their
own hardware.

===============================================================================
*/

#ifndef SERVO_CONFIG_H
#define SERVO_CONFIG_H

#include "ServoController.h"

#define NUM_SERVOS 9


// ============================================================================
// Servo configuration table
// ============================================================================
//
// Each entry corresponds to one servo.
//
// Fields:
//
// name                     : logical servo name
// pwm_pin                  : Arduino pin driving the servo
// servo_min_deg            : calibrated minimum servo angle
// servo_max_deg            : calibrated maximum servo angle
// allowed_min_deg          : application limit
// allowed_max_deg          : application limit
// rest_deg                 : rest position used at initialization
// pwm_min_us               : PWM value corresponding to servo_min_deg
// pwm_max_us               : PWM value corresponding to servo_max_deg
// max_speed_degps          : physical maximum servo speed
// default_speed_pct        : default motion speed
// default_accel_pct        : default motion acceleration
// feedback_adc_pin         : analog pin used for feedback (-1 if none)
// fb_adc_at_servo_min_deg  : ADC value at servo_min_deg
// fb_adc_at_servo_max_deg  : ADC value at servo_max_deg
// inverted                 : invert direction
// fault_detection_enabled  : enable stall detection
//

static ServoConfig servoConfigs[NUM_SERVOS] =
{

// name   pin  min  max  allowedMin allowedMax rest  pwmMin pwmMax speed  defV defA fbPin fbMin fbMax inv fault

{ "S1",   2,   0, 180,     0,       180,       90,   1000,  2000,  261,    50,  50,  -1,    0,    0,  false, false },

{ "S2",   3,   0, 180,     0,       180,       90,   1000,  2000,  261,    50,  50,  -1,    0,    0,  false, false },

{ "S3",   4,   0, 180,     0,       180,       90,   1000,  2000,  261,    50,  50,  -1,    0,    0,  false, false },

{ "S4",   5,   0, 180,     0,       180,       90,   1000,  2000,  261,    50,  50,  -1,    0,    0,  false, false },

{ "S5",   6,   0, 180,     0,       180,       90,   1000,  2000,  261,    50,  50,  -1,    0,    0,  false, false },

{ "S6",   7,   0, 180,     0,       180,       90,   1000,  2000,  261,    50,  50,  -1,    0,    0,  false, false },

{ "S7",   8,   0, 180,     0,       180,       90,   1000,  2000,  261,    50,  50,  -1,    0,    0,  false, false },

{ "S8",   9,   0, 180,     0,       180,       90,   1000,  2000,  261,    50,  50,  -1,    0,    0,  false, false },

{ "S9",  10,   0, 180,     0,       180,       90,   1000,  2000,  261,    50,  50,  -1,    0,    0,  false, false }

};

#endif