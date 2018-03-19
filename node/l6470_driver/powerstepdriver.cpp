#include "powerstepdriver.h"

PowerStepCfg::PowerStepCfg (CommsDriver &commsDriver , int motor)
{
    commonCfg_ = CommonConfig(commsDriver,motor);
}

PowerStepCfg::PowerStepCfg (const StepperMotor   &stepperMotor,
                            const CommonConfig   &commonConfig,
                            const CurrentModeCfg &currentModeConfig,
                            const VoltageModeCfg &voltageModeConfig):
    stepperMotor_(stepperMotor),
    commonCfg_(commonConfig),
    currentModeCfg_(currentModeConfig),
    voltageModeCfg_(voltageModeConfig)
{}

void
PowerStepCfg::set(CommsDriver &commsDriver, int motor)
{
    std::cout << "============================" << std::endl;
    std::cout << "Power Step Config : motor " << motor << std::endl;
    std::cout << "----------------------------" << std::endl;
    std::cout <<
    commonCfg_.set(commsDriver,motor);
    if (commonCfg_.controlMode == CurrentControlMode)
    {
        std::cout << "Set current mode config" << std::endl;
        currentModeCfg_.set(commsDriver,motor);
    }
    else
    {
        // default to voltage mode config
        std::cout << "Set the voltage mode config" << std::endl;
        voltageModeCfg_.set(commsDriver,motor);
    }
    std::cout << "============================" << std::endl;
}

void
PowerStepCfg::readFromFile(const std::string &filePath)
{
    currentModeCfg_.readFromFile(filePath);
    voltageModeCfg_.readFromFile(filePath);
}

void
PowerStepCfg::writeToFile(const std::string &cfgFilePath)
{

}

void
PowerStepCfg::unitTest(CommsDriver &commsDriver, int motor)
{
    commonCfg_.unitTest(commsDriver,motor);
    voltageModeCfg_.unitTest(commsDriver,motor);
}

void
PowerStepDriver::setConfig(AbstractConfig &cfg , int motor)
{
    cfg.set(*commsDriver_,motor);
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

PowerStepCfg
PowerStepDriver::getConfig(int motor)
{
    return PowerStepCfg(*commsDriver_,motor);
}

std::string toString (const PowerStepCfg &cfg)
{
    std::stringstream ss;
    ss << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    ss << "--------------------------------------------------------------" << std::endl;
    ss << "StepperMotor : " << std::endl << cfg.stepperMotor_ << std::endl;
    ss << "--------------------------------------------------------------" << std::endl;
    ss << "CommonCfg : " << std::endl << cfg.commonCfg_ << std::endl;
    ss << "--------------------------------------------------------------" << std::endl;
    ss << "VoltageModeCfg :" << std::endl << cfg.voltageModeCfg_ << std::endl;
    ss << "--------------------------------------------------------------" << std::endl;
    ss << "CurrentModeCfg : " << std::endl << cfg.currentModeCfg_ << std::endl;
    ss << "--------------------------------------------------------------" << std::endl;
    ss << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
    return ss.str();
}

PowerStepDriver::PowerStepDriver(const std::vector<StepperMotor> &motors, int spiBus, CommsDebugLevel commsDebugLevel):
    BaseDriver(motors,PowerStep01,spiBus,commsDebugLevel)
{
}

PowerStepDriver::PowerStepDriver(const std::vector<StepperMotor> &motors, std::vector <PowerStepCfg> &cfgs, int spiBus, CommsDebugLevel commsDebugLevel):
    PowerStepDriver(motors,spiBus,commsDebugLevel)
{
    // configure the stepper motor drivers as required
    int motor=0;
    for (PowerStepCfg &cfg : cfgs)
    {
        cfg.commonCfg_.set(*commsDriver_,motor);

        if (cfg.commonCfg_.controlMode == VoltageControlMode)
            cfg.voltageModeCfg_.set(*commsDriver_,motor);
        else
            cfg.currentModeCfg_.set(*commsDriver_,motor);
	
        ++motor;
    }
}
