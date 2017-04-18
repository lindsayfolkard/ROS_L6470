#include "motor.h"
#include <sstream>
#include <math.h>

std::string toString(const BackEmfConfig &backEmfConfig)
{
    std::stringstream ss;
    ss << "KVAL_HOLD  : " << backEmfConfig.holdingKVal	     << std::endl;
    ss << "KVAL_RUN   : " << backEmfConfig.constantSpeedKVal << std::endl;
    ss << "KVAL_ACC   : " << backEmfConfig.accelStartingKVal << std::endl;
    ss << "KVAL_DEC   : " << backEmfConfig.decelStartingKVal << std::endl;
    ss << "INT_SPEED  : " << backEmfConfig.intersectSpeed    << std::endl;
    ss << "ST_SLP     : " << backEmfConfig.startSlope        << std::endl;
    ss << "FN_SLP_ACC : " << backEmfConfig.accelFinalSlope   << std::endl;
    ss << "FN_SLP_DEC : " << backEmfConfig.decelFinalSlope   << std::endl;
    return ss.str();
}

std::string toString(StepperMotorSize motorType)
{
    switch (motorType)
    {
    case NEMA11 : return "NEMA11";
    case NEMA14 : return "NEMA14";
    case NEMA17 : return "NEMA17";
    case NEMA23 : return "NEMA23";
    default : assert (!"Invalid motorType");
    }
}

// Constructor
StepperMotor::StepperMotor(StepperMotorSize  _motorSize,
                           StepperMotorModel _motorModel,
                           double            _stepAngle,
                           double            _ratedCurrent,
                           double            _phaseResistance,
                           double            _phaseInductance,
                           double            _holdingTorque,
                           double            _ke):
    motorSize(_motorSize),
    motorModel(_motorModel),
    stepAngle(_stepAngle),
    ratedCurrent(_ratedCurrent),
    phaseResistance(_phaseResistance),
    phaseInductance(_phaseInductance),
    holdingTorque(_holdingTorque),
    Ke(_ke){}


std::string toString(StepperMotorModel stepperModel)
{
    switch (stepperModel)
    {
    case StepperModel_Nema23_57H703 : return "NEMA23_57H703";
    case StepperModel_Nema23_57BYGH51 : return "NEMA23_57BYGH51";
    default : assert (!"Invalid Model");
    }
}

std::string toString(const StepperMotor &x)
{
    std::stringstream ss;
    ss << "Motor Type          : " << x.motorSize  << std::endl;
    ss << "Motor Model         : " << x.motorModel << std::endl;
    ss << "Step Angle          : " << x.stepAngle  << " deg" << std::endl;
    ss << "Rated Current       : " << x.ratedCurrent << " A" << std::endl;
    ss << "Phase Resistance    : " << x.phaseResistance << " ohms" << std::endl;
    ss << "Phase Inductance    : " << x.phaseInductance << " mH" << std::endl;
    ss << "Back Emf Const (Ke) : " << x.Ke << " V/Hz" << std::endl;
    ss << "Holding Torque      : " << x.holdingTorque << "Nm" << std::endl;
    return ss.str();
}

BackEmfConfig BackEmfConfigFromStepper(const StepperMotor & stepperMotor , double vbus , double phaseCurrent )
{
    BackEmfConfig backEmfConfig;

    // Calculate the respective KVals
    const kValMultiplier = 256; // 2^8
    const double kVal = ((stepperMotor.phaseResistance*phaseCurrent)/vbus)*kValMultiplier;
    assert(kVal > 0 && kval < 0xFF);
    backEmfConfig.accelStartingKVal = kVal;
    backEmfConfig.holdingKVal = kVal;
    backEmfConfig.constantSpeedKVal = kVal;
    backEmfConfig.decelStartingKVal = kVal;

    // Calculate the approximate intersect speed
    const int tTick = 250; // tick time in nanoseconds
    // TODO - not sure if ttickns should be converted to seconds
    backEmfConfig.intersectSpeed = ((4 * stepperMotor.phaseResistance)/(2 * M_PI * stepperMotor.phaseInductance)) * pow(2,26) * tTick;

    // Calculate the starting slope
    backEmfConfig.startSlope = ((stepperMotor.Ke/4)/vbus) * pow(2,16);

    // Calculate the Final Slope
    const int finalSlope = ((((2 * M_PI * phaseCurrent) + stepperMotor.Ke)/4)/vbus) * pow(2,16);
    backEmfConfig.accelFinalSlope = finalSlope;
    backEmfConfig.decelFinalSlope = finalSlope;

    return backEmfConfig;
}
