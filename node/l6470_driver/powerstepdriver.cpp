#include "powerstepdriver.h"

PowerStepCfg::PowerStepCfg (const CfgFile &cfgFile)
{
//    if (cfgFile.commonConfigFile_!="")
//        commonCfg_ = CommonConfig(cfgFile.commonConfigFile_);

//    if (cfgFile.voltageModeConfigFile_ != "")
//        voltageModeCfg_ = VoltageModeCfg(cfgFile.voltageModeConfigFile_);
//    else if (cfgFile.stepperMotorFile_ != "")
//        voltageModeCfg_ = BackEmfConfigFromStepper(StepperMotor(cfgFile.stepperMotorFile_));

//    if (cfgFile.currentModeConfigFile_ != "")
//        currentModeCfg_ = CurrentModeCfg(cfgFile.currentModeConfigFile_);
//
}

void
PowerStepCfg::set(CommsDriver &commsDriver, int motor)
{
    commonCfg_.set(commsDriver,motor);
    currentModeCfg_.set(commsDriver,motor);
    // default to voltage mode config
    voltageModeCfg_.set(commsDriver,motor);
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
    // TODO ?
}

void
PowerStepDriver::setConfig(const AbstractConfig &cfg , int motor)
{
    // TODO
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
    BaseDriver(motors,PowerStep01,spiBus,commsDebugLevel)
{
}

PowerStepDriver::PowerStepDriver(const std::vector<StepperMotor> &motors, std::vector <PowerStepCfg> &cfgs, int spiBus, CommsDebugLevel commsDebugLevel):
    PowerStepDriver(motors,spiBus,commsDebugLevel)
{
    // configure the stepper motor drivers as required
    int motor=1;
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
