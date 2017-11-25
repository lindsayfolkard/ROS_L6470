#include "motor.h"
#include <sstream>
#include <math.h>
#include <assert.h>
#include "types.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>


namespace pt = boost::property_tree;

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

StepperMotor::StepperMotor(const std::string &cfg)
{
    // Let's do this in json format (easier to parse)
    pt::ptree root;

    try
    {
        pt::read_json(cfg,root);

        motorSize  = getStepperMotorSizeBiMap().right.at(root.get<std::string>("motorSize"));
        motorModel = root.get<std::string>("motorModel");
        stepAngle  = root.get<double>("stepAngle");
        phaseResistance = root.get<double>("phaseResistance");
        phaseInductance = root.get<double>("phaseInductance");
        Ke              = root.get<double>("backEmfConstant");
        holdingTorque   = root.get<double>("holdingTorque");
    }
    catch (std::exception &e)
    {
        std::cout << "Exception caught while trying to read stepper motor cfg file " << cfg << " with reason " << e.what() << std::endl;
        throw;
    }
}

void
StepperMotor::writeToFile(const std::string &file)
{
    pt::ptree root;

    root.put("motorModel",motorModel);
    root.put("motorSize",motorSize);
    root.put("stepAngle",stepAngle);
    root.put("ratedCurrent",ratedCurrent);
    root.put("phaseResistance",phaseResistance);
    root.put("phaseInductance",phaseInductance);
    root.put("backEmfConstant",Ke);
    root.put("holdingTorque",holdingTorque);

    // Open the file and write to json
    std::ofstream outFile;
    outFile.open(file);
        //throw; // TODO - fix to real exception
    pt::write_json(outFile,root);
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
