#include "motor.h"
#include <sstream>
#include <math.h>
#include <assert.h>
#include "types.h"

std::string toString(MotorDriverType motorDriverType)
{
    switch (motorDriverType)
    {
    case PowerStep01:
        return "PowerStep01";
    case L6470:
        return "L6470";
    case L6472:
        return "L6472";
    default:
        assert(!"Invalid MotorDriverType in toString");
    }
}

boost::bimap <StepperMotorSize,std::string> getStepperMotorSizeBiMap()
{
    typedef boost::bimap<StepperMotorSize,std::string> MotorSizeMap;  
    typedef MotorSizeMap::value_type element;

    MotorSizeMap newMap;
    
    newMap.insert(element (NEMA11,"NEMA11"));
    newMap.insert(element (NEMA14,"NEMA14"));
    newMap.insert(element (NEMA17,"NEMA17"));
    newMap.insert(element (NEMA23,"NEMA23"));

    return newMap;
}

std::string toString(StepperMotorSize motorType)
{
    return getStepperMotorSizeBiMap().left.at(motorType);
}

// Constructor
StepperMotor::StepperMotor(StepperMotorSize  _motorSize,
                           std::string       _motorModel,
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

std::string toString(const StepperMotor &x)
{
    std::stringstream ss;
    ss << "Motor Size       : " << x.motorSize  << std::endl;
    ss << "Motor Model      : " << x.motorModel << std::endl;
    ss << "Step Angle       : " << x.stepAngle  << " deg" << std::endl;
    ss << "Rated Current    : " << x.ratedCurrent << " A" << std::endl;
    ss << "Phase Resistance : " << x.phaseResistance << " ohms" << std::endl;
    ss << "Phase Inductance : " << x.phaseInductance << " mH" << std::endl;
    ss << "BackEmf Constant : " << x.Ke << " V/Hz" << std::endl;
    ss << "Holding Torque   : " << x.holdingTorque << "Nm" << std::endl;
    return ss.str();
}

void tryReadDoubleFromCfg(const std::string &cfg, const std::string &marker , double &value)
{
   std::string argument = getArgument(cfg,marker);
   
   if (argument != "")
	value=std::stod(argument);
   else
	std::cout << "Unable to read marker --> " << marker;

}

StepperMotor stepperFromString(const std::string &cfg)
{
    StepperMotor motor;
    
    std::string argument = getArgument(cfg,"Motor Size");
    if (argument != "")
    motor.motorSize = getStepperMotorSizeBiMap().right.at(argument);

    argument = getArgument(cfg,"Motor Model");
    if (argument != "") 
        motor.motorModel = argument;

    tryReadDoubleFromCfg(cfg,"Step Angle",motor.stepAngle);
    tryReadDoubleFromCfg(cfg,"Rated Current",motor.ratedCurrent);
    tryReadDoubleFromCfg(cfg,"Phase Resistance",motor.phaseResistance);
    tryReadDoubleFromCfg(cfg,"Phase Inductance",motor.phaseInductance);
    tryReadDoubleFromCfg(cfg,"BackEmf Constant",motor.Ke);
    tryReadDoubleFromCfg(cfg,"Holding Torque",motor.holdingTorque);
    
    return motor;
}


VoltageModeCfg BackEmfConfigFromStepper(const StepperMotor & stepperMotor , double vbus , double phaseCurrent )
{
    VoltageModeCfg backEmfConfig;

    // Calculate the respective KVals
    const int kValMultiplier = 256; // 2^8
    const uint8_t kVal = ((stepperMotor.phaseResistance*phaseCurrent)/vbus)*kValMultiplier;
    assert(kVal > 0 && kVal < 0xFF);
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
