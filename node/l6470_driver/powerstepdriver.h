#pragma once

#include "basedriver.h"
#include "config.h"

class PowerStepCfg : public AbstractConfig,
                     public WriteableConfig
{

public:

    PowerStepCfg (const std::string &cfgFilePath);

    virtual void set(CommsDriver &commsDriver, int motor) override;
    virtual void readFromFile(const std::string &filePath) override;
    virtual void writeToFile(const std::string &cfgFilePath) override;

    void    setCurrentModeCfg(CommsDriver &commsDriver, int motor);
    void    setVoltageModeCfg(CommsDriver &commsDriver, int motor);

    CommonConfig    commonCfg_;
    CurrentModeCfg  currentModeCfg_;
    VoltageModeCfg  voltageModeCfg_;
};

class PowerStepDriver : public AbstractDriver,
                        public BaseDriver
{
public:

    PowerStepDriver(const std::vector<StepperMotor> &motors, int spiBus = 0, CommsDebugLevel commsDebugLevel = CommsDebugNothing);
    PowerStepDriver(const std::vector<StepperMotor> &motors, const std::vector <PowerStepCfg> &cfgs, int spiBus, CommsDebugLevel commsDebugLevel);

    virtual void setConfig(const AbstractConfig &cfg , int motor) override;

private:

};
