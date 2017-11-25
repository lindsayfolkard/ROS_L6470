#include "driverfactory.h"

std::unique_ptr<AbstractDriver> factoryMakeDriver(const OverallCfg &overallCfg)
{
    // Read the required stepper motor and config files
    std::vector<StepperMotor> motors;
    std::vector<PowerStepCfg> PowerStepCfgs;

    for (const CfgFile &cfgFile : overallCfg.cfgFiles_)
    {
        //motors.push_back(StepperMotor());
    }
}
