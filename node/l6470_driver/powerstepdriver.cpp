#include "powerstepdriver.h"

void
PowerStepCfg::set(CommsDriver &commsDriver, int motor)
{
    currentModeCfg_.set(commsDriver,motor);
    // default to voltage mode config
    voltageModeCfg_.set(commsDriver,motor);
}

void
PowerStepCfg::writeToFile(const std::string &cfgFilePath)
{
}

void
PowerStepCfg::setCurrentModeCfg(CommsDriver &commsDriver, int motor)
{
    currentModeCfg_.set(commsDriver,motor);
}

void
PowerStepCfg::setVoltageModeCfg(CommsDriver &commsDriver, int motor)
{
    voltageModeCfg_.set(commsDriver,motor);
}

PowerStepDriver::PowerStepDriver(const std::vector<StepperMotor> &motors, int spiBus, CommsDebugLevel commsDebugLevel):
    BaseDriver(motors,spiBus,commsDebugLevel)
{
    assert(!"TODO - powerstep driver constructor");
}

PowerStepDriver::PowerStepDriver(const std::vector<StepperMotor> &motors, const std::vector <PowerStepCfg> &cfgs, int spiBus, CommsDebugLevel commsDebugLevel)
{
    assert (!"TODO - powerstep driver constructor");
}
