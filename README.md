# Enhanced Servo Library

Author: Alejandro Alonso Puig 

License: Apache 2.0 

Repository: https://github.com/aalonsopuig

------------------------------------------------------------------------

## Overview

Enhanced Servo Library is an Arduino library designed for advanced
control of hobby RC servos, especially in robotics applications.

Unlike basic servo libraries that only send position commands, this
library provides:

-   trapezoidal motion profiles
-   speed control expressed as percentage of the servo maximum speed
-   acceleration control
-   optional analog position feedback
-   servo stall / fault detection
-   support for multi‑servo robotic systems

The library is designed to remain generic and reusable while being
powerful enough for robotic platforms such as humanoid robots,
manipulators or research prototypes.

------------------------------------------------------------------------

## Main Features

### Motion profiling

Servos are not commanded with large instantaneous steps.\
Instead, the library generates smooth trapezoidal motion profiles using:

-   maximum speed
-   acceleration

This produces smoother movement and reduces mechanical stress.

------------------------------------------------------------------------

### Speed control

Speed is defined as a percentage of the maximum physical speed of the
servo.

Example:

servo.setTarget(120, 50, 40);

Meaning:

-   move to 120°
-   using 50% of the servo maximum speed
-   using 40% of the configured acceleration

------------------------------------------------------------------------

### Optional analog feedback

Some servos expose the internal potentiometer signal.

When connected to an analog input, the library can:

-   measure the real servo angle
-   compare commanded vs measured position
-   detect blocked servos

------------------------------------------------------------------------

### Fault detection

If feedback is enabled, the library can detect situations where the servo is commanded to move but the measured angle does not change

This can indicate:

-   mechanical blockage
-   broken gears
-   disconnected servo
-   electrical faults

When detected, the library raises a fault flag. The application sketch
can then decide how to react.

------------------------------------------------------------------------

## Library Architecture

The library uses two main components:

ServoConfig\
ServoController

------------------------------------------------------------------------

## ServoConfig

ServoConfig defines the characteristics and limits of a servo.

Example configuration:

ServoConfig cfg;

cfg.name = "Shoulder"; cfg.pwm_pin = 9;

cfg.servo_min_deg = 0; cfg.servo_max_deg = 180;

cfg.allowed_min_deg = 20; cfg.allowed_max_deg = 160;

cfg.rest_deg = 90;

cfg.pwm_min_us = 700; cfg.pwm_max_us = 2400;

cfg.max_speed_degps = 17.5;

cfg.default_speed_pct = 50; cfg.default_accel_pct = 50;

cfg.feedback_adc_pin = A4; cfg.fb_adc_at_servo_min_deg = 101;
cfg.fb_adc_at_servo_max_deg = 383;

cfg.inverted = false; cfg.fault_detection_enabled = true;

------------------------------------------------------------------------

## ServoController

ServoController is the runtime object responsible for controlling a
servo.

Example:

ServoController servo;

servo.begin(cfg); servo.syncToFeedback();

------------------------------------------------------------------------

## Basic Usage

Minimal example:

#include "ServoController.h"

ServoConfig cfg; ServoController servo;

void setup() { servo.begin(cfg); servo.syncToAngle(90); }

void loop() { servo.setTarget(120, 60, 40); servo.update(); }

update() must be called regularly (typically every 10--20 ms).

------------------------------------------------------------------------

## Public API

### begin()

void begin(const ServoConfig& cfg);

Initializes the servo controller using the configuration provided.

------------------------------------------------------------------------

### syncToAngle()

void syncToAngle(float angle);

Synchronizes the controller with a known angle when feedback is not
available.

------------------------------------------------------------------------

### syncToFeedback()

void syncToFeedback();

Synchronizes the controller with the real servo position using analog
feedback.

------------------------------------------------------------------------

### setTarget()

void setTarget(float targetDeg);

Moves the servo to a target angle using default speed and acceleration.

------------------------------------------------------------------------

### setTarget()

void setTarget(float targetDeg, uint8_t speedPct, uint8_t accelPct);

Moves the servo using explicit speed and acceleration percentages.

------------------------------------------------------------------------

### update()

void update();

Advances the motion profile and updates the servo PWM command.

This function must be called periodically in the main loop.

------------------------------------------------------------------------

## State Information

float getCommandDeg(); Returns the current commanded reference angle.

float getMeasuredDeg(); Returns the angle measured from feedback.

float getCurrentDeg(); Returns the best estimate of the servo angle.

bool hasFault(); Returns true if a stall or no‑movement condition has
been detected.

const char\* getName(); Returns the servo name defined in the
configuration.

------------------------------------------------------------------------

## Safety Philosophy

The library detects faults but does not enforce hardware shutdown.

The application sketch should decide how to react.

Example:

if (servo.hasFault()) { cutServoPower(); }

------------------------------------------------------------------------

## Examples

Several examples are provided in the examples/ folder.

Simple_servo_ctrl_arduino\
Basic example controlling one servo without feedback.

Multi_servo_ctrl_arduino\
Example controlling multiple servos simultaneously.

Servo_with_feedback_demo_arduino\
Example using analog feedback from the servo potentiometer.

Servo_safety_shutdown_demo_arduino\
Example demonstrating fault detection and safety shutdown.

------------------------------------------------------------------------

## Hardware Requirements

-   Arduino compatible board
-   Standard RC servo
-   Optional servo with accessible potentiometer feedback
-   External servo power supply recommended

