#pragma once

// Support functions for converting from user units to L6470 units
unsigned long accCalc(float stepsPerSecPerSec);

unsigned long decCalc(float stepsPerSecPerSec);

unsigned long minSpdCalc(float stepsPerSec);

unsigned long maxSpdCalc(float stepsPerSec);

unsigned long FSCalc(float stepsPerSec);

unsigned long intSpdCalc(float stepsPerSec);

unsigned long spdCalc(float stepsPerSec);

// Support functions for converting from L6470 to user units
float accParse(unsigned long stepsPerSecPerSec);

float decParse(unsigned long stepsPerSecPerSec);

float minSpdParse(unsigned long stepsPerSec);

float maxSpdParse(unsigned long stepsPerSec);

float FSParse(unsigned long stepsPerSec);

float intSpdParse(unsigned long stepsPerSec);

float spdParse(unsigned long stepsPerSec);
