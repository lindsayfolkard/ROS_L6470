#pragma once

#include "basedriver.h"
#include "config.h"

class PowerStepCfg : public AbstractConfig,
                     public WriteableConfig
{

public:

    PowerStepCfg (const CfgFile &cfgFile);
    PowerStepCfg (CommsDriver &commsDriver , int motor);

    virtual void set(CommsDriver &commsDriver, int motor) override;
    virtual void readFromFile(const std::string &filePath) override;
    virtual void writeToFile(const std::string &cfgFilePath) override;

    void    setCurrentModeCfg(CommsDriver &commsDriver, int motor);
    void    setVoltageModeCfg(CommsDriver &commsDriver, int motor);

    CommonConfig    commonCfg_;
    CurrentModeCfg  currentModeCfg_;
    VoltageModeCfg  voltageModeCfg_;
};

std::string toString (const PowerStepCfg &cfg);
inline std::ostream& operator<<(std::ostream &os , const PowerStepCfg &cfg)
{
    os << toString(cfg);
}

class PowerStepDriver : public BaseDriver
{
public:

    PowerStepDriver(const std::vector<StepperMotor> &motors, int spiBus = 0, CommsDebugLevel commsDebugLevel = CommsDebugNothing);
    PowerStepDriver(const std::vector<StepperMotor> &motors, std::vector<PowerStepCfg> &cfgs, int spiBus = 0, CommsDebugLevel commsDebugLevel = CommsDebugNothing);

    virtual void setConfig(const AbstractConfig &cfg , int motor) override;
    PowerStepCfg getConfig(int motor);

private:

};
