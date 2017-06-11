#pragma once

#include <stdint.h>
#include <string.h>
#include <iostream>

// General Information regarding motor tuning can be found here :
// http://www.st.com/content/ccc/resource/technical/document/application_note/ad/fc/fb/f0/f7/c7/4c/48/DM00061093.pdf/files/DM00061093.pdf/jcr:content/translations/en.DM00061093.pdf
// (page 15 lists recommended values)

///
/// \brief The BackEmfConfig struct
/// contains all information that is needed to
/// configure a motor to be used with a controller
///
struct BackEmfConfig
{
    uint8_t holdingKVal;
    uint8_t constantSpeedKVal;
    uint8_t accelStartingKVal;
    uint8_t decelStartingKVal;

    long intersectSpeed;
    long startSlope;
    long accelFinalSlope;
    long decelFinalSlope;
};
std::string toString(const BackEmfConfig &backEmfConfig);
inline std::ostream& operator<<(std::ostream& os,const BackEmfConfig &x)
{
    return os << toString(x);
}

enum StepperMotorSize
{
    NEMA11,
    NEMA14,
    NEMA17,
    NEMA23
};
std::string toString(StepperMotorSize motorType);
inline std::ostream& operator<<(std::ostream& os,StepperMotorSize x)
{
    return os << toString(x);
}

enum StepperMotorModel
{
    StepperModel_Nema23_57H703,
    StepperModel_Nema23_57BYGH51,
    StepperModel_Nema17_42BYGHW811
};
std::string toString(StepperMotorModel stepperModel);
inline std::ostream& operator<<(std::ostream& os,StepperMotorModel x)
{
    return os << toString(x);
}

struct StepperMotor
{
    // Constructor
    StepperMotor(StepperMotorSize _motorSize,
                 StepperMotorModel _motorModel,
                 double            _stepAngle,
                 double            _ratedCurrent,
                 double            _phaseResistance,
                 double            _phaseInductance,
                 double            _holdingTorque,
                 double            _ke);

    // Motor Type information
    const StepperMotorSize  motorSize;
    const StepperMotorModel motorModel;

    // Motor Information
    const double stepAngle; // degrees
    const double ratedCurrent; // Amps
    const double phaseResistance; // ohms
    const double phaseInductance; // mH

    // General info
    const double holdingTorque;// NM

    // Motor Back EMF constant
    const double Ke; // V/Hz --> see http://www.st.com/content/ccc/resource/technical/document/application_note/e8/ca/05/a0/9e/ff/4e/69/DM00039787.pdf/files/DM00039787.pdf/jcr:content/translations/en.DM00039787.pdf
		     // for instructions on how to calculate for a given motor.
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
BackEmfConfig BackEmfConfigFromStepper(const StepperMotor & stepperMotor , double vbus , double phaseCurrent );

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
        StepperModel_Nema23_57H703,
        1.8,
        3.0,
        0.9,
        3.4,
        2.012,
        0.073){}
};

/// Model  :
/// Type   :
/// Source : ???
struct Stepper_42BYGHW811 : public StepperMotor
{
    Stepper_42BYGHW811() : StepperMotor(
       NEMA17,
       StepperModel_Nema17_42BYGHW811,
       1.8,
       2.5,
       1.25,
       1.8,
       0.48,
       0.014){}
};

/// Model  :
/// Type   :
/// Source : ???
struct Stepper_57BYGH51 : public StepperMotor
{
    Stepper_57BYGH51() : StepperMotor(
        NEMA23,
        StepperModel_Nema23_57BYGH51,
        1.8,
        1.5,
        3.5,
        12,
        1.0,
        0.054){}
};

