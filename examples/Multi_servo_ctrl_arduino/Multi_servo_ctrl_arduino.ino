/*
===============================================================================
Name:         Multi_servo_ctrl_arduino
Version:      1.2.0
Author:       Alejandro Alonso Puig + GPT
GitHub:       https://github.com/aalonsopuig
Date:         2026-03-10
License:      Apache 2.0
-------------------------------------------------------------------------------

Description:

Example demonstrating multi-servo control using the ServoController library.

The hardware configuration of the servos is stored in:

servo_config.h

Each servo has its own explicit configuration entry.

This approach is common in robotics systems where every joint
may require different calibration values.

===============================================================================
*/

#include <Arduino.h>
#include "ServoController.h"
#include "servo_config.h"


#define POT_TARGET_PIN A0
#define POT_SPEED_PIN  A1
#define POT_ACCEL_PIN  A2

#define ADC_SCALE 1023.0f
#define POT_SAMPLES 4


ServoController servos[NUM_SERVOS];


static inline float clampf(float x,float lo,float hi)
{
  if(x<lo) return lo;
  if(x>hi) return hi;
  return x;
}


float readAveragedADC(uint8_t pin,int samples)
{
  long sum=0;

  for(int i=0;i<samples;i++)
  {
    sum+=analogRead(pin);
  }

  return (float)sum/samples;
}


float targetDegFromAdc(float adc)
{
  float deg=(adc/ADC_SCALE)*180.0f;

  return clampf(deg,0.0f,180.0f);
}


uint8_t percentFromAdc(float adc)
{
  float t=clampf(adc/ADC_SCALE,0.0f,1.0f);

  int pct=(int)(1.0f+t*99.0f+0.5f);

  if(pct<1) pct=1;
  if(pct>100) pct=100;

  return (uint8_t)pct;
}


void setup()
{

  // Initialize each servo controller using the configuration table

  for(int i=0;i<NUM_SERVOS;i++)
  {
    servos[i].begin(servoConfigs[i]);

    servos[i].syncToAngle(servoConfigs[i].rest_deg);
  }

}


void loop()
{

  float adcTarget=readAveragedADC(POT_TARGET_PIN,POT_SAMPLES);
  float adcSpeed =readAveragedADC(POT_SPEED_PIN,POT_SAMPLES);
  float adcAccel =readAveragedADC(POT_ACCEL_PIN,POT_SAMPLES);

  float   targetDeg=targetDegFromAdc(adcTarget);
  uint8_t speedPct =percentFromAdc(adcSpeed);
  uint8_t accelPct =percentFromAdc(adcAccel);


  // Send identical commands to all servos

  for(int i=0;i<NUM_SERVOS;i++)
  {
    servos[i].setTarget(targetDeg,speedPct,accelPct);
  }


  // Update each servo controller

  for(int i=0;i<NUM_SERVOS;i++)
  {
    servos[i].update();
  }


  delay(20);

}
