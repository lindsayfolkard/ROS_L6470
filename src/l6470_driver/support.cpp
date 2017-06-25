#include "support.h"
#include <assert.h>
#include <math.h>

// AutoDriverSupport.cpp - Contains utility functions for converting real-world 
//  units (eg, steps/s) to values usable by the dsPIN controller. These are all
//  private members of class AutoDriver.

// The value in the ACC register is [(steps/s/s)*(tick^2)]/(2^-40) where tick is 
//  250ns (datasheet value)- 0x08A on boot.
// Multiply desired steps/s/s by .137438 to get an appropriate value for this register.
// This is a 12-bit value, so we need to make sure the value is at or below 0xFFF.
unsigned long accCalc(float stepsPerSecPerSec)
{
  float temp = stepsPerSecPerSec * 0.137438;
  if( (unsigned long) long(temp) > 0x00000FFF) return 0x00000FFF;
  else return (unsigned long) long(temp);
}


float accParse(unsigned long stepsPerSecPerSec)
{
    return (float) (stepsPerSecPerSec & 0x00000FFF) / 0.137438;
}

// The calculation for DEC is the same as for ACC. Value is 0x08A on boot.
// This is a 12-bit value, so we need to make sure the value is at or below 0xFFF.
unsigned long decCalc(float stepsPerSecPerSec)
{
  float temp = stepsPerSecPerSec * 0.137438;
  if( (unsigned long) long(temp) > 0x00000FFF) return 0x00000FFF;
  else return (unsigned long) long(temp);
}

float decParse(unsigned long stepsPerSecPerSec)
{
    return (float) (stepsPerSecPerSec & 0x00000FFF) / 0.137438;
}

// The value in the MAX_SPD register is [(steps/s)*(tick)]/(2^-18) where tick is 
//  250ns (datasheet value)- 0x041 on boot.
// Multiply desired steps/s by .065536 to get an appropriate value for this register
// This is a 10-bit value, so we need to make sure it remains at or below 0x3FF
unsigned long maxSpdCalc(float stepsPerSec)
{
  unsigned long temp = ceil(stepsPerSec * .065536);
  if( temp > 0x000003FF) return 0x000003FF;
  else return temp;
}


float maxSpdParse(unsigned long stepsPerSec)
{
    return (float) (stepsPerSec & 0x000003FF) / 0.065536;
}

// The value in the MIN_SPD register is [(steps/s)*(tick)]/(2^-24) where tick is 
//  250ns (datasheet value)- 0x000 on boot.
// Multiply desired steps/s by 4.1943 to get an appropriate value for this register
// This is a 12-bit value, so we need to make sure the value is at or below 0xFFF.
unsigned long minSpdCalc(float stepsPerSec)
{
  float temp = stepsPerSec / 0.238;
  if( (unsigned long) long(temp) > 0x00000FFF) return 0x00000FFF;
  else return (unsigned long) long(temp);
}

float minSpdParse(unsigned long stepsPerSec)
{
    return (float) ((stepsPerSec & 0x00000FFF) * 0.238);
}

// The value in the FS_SPD register is ([(steps/s)*(tick)]/(2^-18))-0.5 where tick is 
//  250ns (datasheet value)- 0x027 on boot.
// Multiply desired steps/s by .065536 and subtract .5 to get an appropriate value for this register
// This is a 10-bit value, so we need to make sure the value is at or below 0x3FF.
unsigned long FSCalc(float stepsPerSec)
{
  float temp = (stepsPerSec * .065536)-.5;
  if( (unsigned long) long(temp) > 0x000003FF) return 0x000003FF;
  else return (unsigned long) long(temp);
}

float FSParse(unsigned long stepsPerSec)
{
    return (((float) (stepsPerSec & 0x000003FF)) + 0.5) / 0.065536;
}

// The value in the INT_SPD register is [(steps/s)*(tick)]/(2^-24) where tick is 
//  250ns (datasheet value)- 0x408 on boot.
// Multiply desired steps/s by 4.1943 to get an appropriate value for this register
// This is a 14-bit value, so we need to make sure the value is at or below 0x3FFF.
unsigned long intSpdCalc(float stepsPerSec)
{
  float temp = stepsPerSec * 4.1943;
  if( (unsigned long) long(temp) > 0x00003FFF) return 0x00003FFF;
  else return (unsigned long) long(temp);
}

float intSpdParse(unsigned long stepsPerSec)
{
    return (float) (stepsPerSec & 0x00003FFF) / 4.1943;
}

// When issuing RUN command, the 20-bit speed is [(steps/s)*(tick)]/(2^-28) where tick is 
//  250ns (datasheet value).
// Multiply desired steps/s by 67.106 to get an appropriate value for this register
// This is a 20-bit value, so we need to make sure the value is at or below 0xFFFFF.
unsigned long spdCalc(float stepsPerSec)
{
  unsigned long temp = stepsPerSec * 67.106;
  if( temp > 0x000FFFFF) return 0x000FFFFF;
  else return temp;
}

float spdParse(unsigned long stepsPerSec)
{
    return (float) (stepsPerSec & 0x000FFFFF) / 67.106;
}

