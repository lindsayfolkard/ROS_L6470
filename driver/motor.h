#pragma once

// General Information regarding motor tuning can be found here :
// http://www.st.com/content/ccc/resource/technical/document/application_note/ad/fc/fb/f0/f7/c7/4c/48/DM00061093.pdf/files/DM00061093.pdf/jcr:content/translations/en.DM00061093.pdf
// (page 15 lists recommended values)
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

enum StepperMotorType
{
    NEMA11,
    NEMA14,
    NEMA17,
    NEMA23
};
std::string toString(StepperMotorType motorType);
inline std::ostream& operator<<(std::ostream& os,StepperMotorType x)
{
    return os << toString(x);
}

enum StepperMotorModel
{
    StepperModel_Nema23_57H703,
    StepperModel_Nema23_57BYGH51
};
std::string toString(StepperMotorModel stepperModel);
inline std::ostream& operator<<(std::ostream& os,StepperMotorModel x)
{
    return os << toString(x);
}

struct StepperMotor
{
    // Motor Type information
    const StepperMotorType  motorType;
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

/// Source : ???
struct Stepper_57H703 : StepperMotor
{
    Stepper_57H703():
	motorType(NEMA17),
	motorModel(StepperModel_Nema23_57H703),
	stepAngle(1.8),
	ratedCurrent(3.0),
	phaseResistance(0.9),
	phaseInductance(3.4),
	holdingTorque(2.012),
	Ke(0.073){}
};
