#pragma once

#include <stdint.h>
#include <string.h>
#include <iostream>
#include "types.h"
#include "config.h"
#include <boost/bimap.hpp>

// General Information regarding motor tuning can be found here :
// http://www.st.com/content/ccc/resource/technical/document/application_note/ad/fc/fb/f0/f7/c7/4c/48/DM00061093.pdf/files/DM00061093.pdf/jcr:content/translations/en.DM00061093.pdf
// (page 15 lists recommended values)

enum StepperMotorSize
{
    NEMA11,
    NEMA14,
    NEMA17,
    NEMA23,
    NEMA34
};
std::string toString(StepperMotorSize motorType);
inline std::ostream& operator<<(std::ostream& os,StepperMotorSize x)
{
    return os << toString(x);
}
boost::bimap <StepperMotorSize,std::string> getStepperMotorSizeBiMap();

struct StepperMotor
{
    // Constructor
    StepperMotor(){} // TODO remove me!
    StepperMotor(const std::string &cfg);
    StepperMotor(StepperMotorSize  _motorSize,
                 std::string       _motorModel,
                 double            _stepAngle,
                 double            _ratedCurrent,
                 double            _phaseResistance,
                 double            _phaseInductance,
                 double            _holdingTorque,
                 double            _ke,
                 double            _vbus,
                 double            _phaseCurrent);

    pt::ptree getPTree();
    void      readFromPTree(pt::ptree &root);

    void writeToFile(const std::string &file);

    // Motor Type information
    StepperMotorSize  motorSize;
    std::string motorModel;

    // Motor Information
    double stepAngle; // degrees
    double ratedCurrent; // Amps
    double phaseResistance; // ohms
    double phaseInductance; // mH

    // General info
    double holdingTorque;// NM

    // Motor Back EMF constant
    double Ke; // V/Hz --> see http://www.st.com/content/ccc/resource/technical/document/application_note/e8/ca/05/a0/9e/ff/4e/69/DM00039787.pdf/files/DM00039787.pdf/jcr:content/translations/en.DM00039787.pdf
             // for instructions on how to calculate for a given motor.

    double vbus; // Operating voltage of the stepper motor driver
    double phaseCurrent; // phase current to be used when operating with stepper driver
};
std::string toString(const StepperMotor &x);
inline std::ostream& operator<<(std::ostream& os,const StepperMotor &x)
{
    return os << toString(x);
}

// Converts the motor parameters from a given stepper motor to a compatible
// backemf config to be used by the L6470 Driver
// See : http://www.st.com/content/ccc/resource/technical/document/application_note/e8/ca/05/a0/9e/ff/4e/69/DM00039787.pdf/files/DM00039787.pdf/jcr:content/translations/en.DM00039787.pdf
// page 15
VoltageModeCfg BackEmfConfigFromStepper(const StepperMotor &stepperMotor);

///
/// Individual Stepper Motor Structs for specific motors.
///
/// NB: I define here in the hope that we end up with a rough list of params for
/// most used steppers. Feel free to add a model and submit a pull request.
///

/// Model  :
/// Type   :
/// Source : ???
struct Stepper_57H703 : public StepperMotor
{
    Stepper_57H703() : StepperMotor(
        NEMA23,
        "Nema23_57H703",
        1.8,
        3.0,
        1.8, // 0.9
        2.5, // 3.4
        2.012,
        0.1, // 0.073
        12,
        2.5){}
};

/// Model  :
/// Type   :
/// Source : ???
struct Stepper_42BYGHW811 : public StepperMotor
{
    Stepper_42BYGHW811() : StepperMotor(
       NEMA17,
       "Nema17_42BYGHW811",
       1.8,
       2.5,
       1.25,
       1.8,
       0.48,
       0.014,
       12,
       2.5){}
};

/// Model  :
/// Type   :
/// Source : ???
struct Stepper_57BYGH51 : public StepperMotor
{
    Stepper_57BYGH51() : StepperMotor(
        NEMA23,
        "Nema23_57BYGH51",
        1.8,
        1.5,
        3.5,
        12,
        1.0,
        0.054,
        12,
        2.5){}
};

