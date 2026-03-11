/*
===============================================================================
Name:         Multi_servo_ctrl_arduino
Version:      1.1.0
Author:       Alejandro Alonso Puig + GPT
GitHub:       https://github.com/aalonsopuig
Date:         2026-03-10
License:      Apache 2.0
-------------------------------------------------------------------------------

Description:

Example demonstrating how to control multiple servos using the
ServoController library.

This example intentionally separates the **servo hardware configuration**
from the main program logic.

Configuration parameters such as:

- servo pins
- PWM calibration
- motion limits
- servo speed characterization

are defined in the file:

servo_config.h

This approach improves maintainability and mirrors the configuration
separation used in larger robotics systems (ROS, robot firmware, etc.).

===============================================================================
*/

#include <Arduino.h>
#include "ServoController.h"
#include "servo_config.h"


// ============================================================================
// User control inputs
// ============================================================================
//
// Three potentiometers define the motion parameters applied to all servos.
//

#define POT_TARGET_PIN A0
#define POT_SPEED_PIN  A1
#define POT_ACCEL_PIN  A2


// ============================================================================
// ADC parameters
// ============================================================================

#define ADC_SCALE   1023.0f
#define POT_SAMPLES 4


// ============================================================================
// Servo objects
// ============================================================================
//
// One controller object per servo.
//

ServoController servos[NUM_SERVOS];
ServoConfig servoTable[NUM_SERVOS];


// ============================================================================
// Utility helpers
// ============================================================================

// Clamp a value between two limits
static inline float clampf(float x,float lo,float hi)
{
  if(x<lo) return lo;
  if(x>hi) return hi;
  return x;
}


// Average several ADC samples to reduce noise
float readAveragedADC(uint8_t pin,int samples)
{
  long sum=0;

  for(int i=0;i<samples;i++)
  {
    sum+=analogRead(pin);
  }

  return (float)sum/samples;
}


// Convert ADC value to target angle
float targetDegFromAdc(float adc)
{
  float deg=(adc/ADC_SCALE)*180.0f;

  return clampf(deg,0.0f,180.0f);
}


// Convert ADC value to percentage
uint8_t percentFromAdc(float adc)
{
  float t=clampf(adc/ADC_SCALE,0.0f,1.0f);

  int pct=(int)(1.0f+t*99.0f+0.5f);

  if(pct<1) pct=1;
  if(pct>100) pct=100;

  return (uint8_t)pct;
}


// ============================================================================
// Setup
// ============================================================================

void setup()
{
  // Initialize each servo controller using the configuration
  // defined in servo_config.h

  for(int i=0;i<NUM_SERVOS;i++)
  {
    servoTable[i]=makeServoConfig(SERVO_NAMES[i],SERVO_PINS[i]);

    servos[i].begin(servoTable[i]);

    // Since this example does not use feedback,
    // we synchronize the controller with the known rest angle.
    servos[i].syncToAngle(REST_DEG);
  }
}


// ============================================================================
// Main loop
// ============================================================================

void loop()
{
  // Read user control inputs

  float adcTarget=readAveragedADC(POT_TARGET_PIN,POT_SAMPLES);
  float adcSpeed =readAveragedADC(POT_SPEED_PIN,POT_SAMPLES);
  float adcAccel =readAveragedADC(POT_ACCEL_PIN,POT_SAMPLES);

  float   targetDeg=targetDegFromAdc(adcTarget);
  uint8_t speedPct =percentFromAdc(adcSpeed);
  uint8_t accelPct =percentFromAdc(adcAccel);

  // Send the same motion command to all servos

  for(int i=0;i<NUM_SERVOS;i++)
  {
    servos[i].setTarget(targetDeg,speedPct,accelPct);
  }

  // Update all servo controllers

  for(int i=0;i<NUM_SERVOS;i++)
  {
    servos[i].update();
  }

  delay(20);
}
