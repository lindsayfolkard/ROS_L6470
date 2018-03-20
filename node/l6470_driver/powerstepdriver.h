#pragma once

#include "basedriver.h"
#include "config.h"

class PowerStepCfg : public AbstractConfig,
                     public WriteableConfig
{

public:

    PowerStepCfg() {}

    PowerStepCfg(const std::string &cfgFile) {readFromFile(cfgFile);}

    PowerStepCfg (const CommonConfig   &commonConfig,
                  const CurrentModeCfg &currentModeConfig = CurrentModeCfg(),
                  const VoltageModeCfg &voltageModeConfig = VoltageModeCfg());

    PowerStepCfg (CommsDriver &commsDriver , int motor);

    // Get representation of config in ptree (i.e JSON ready) format
    virtual pt::ptree   getPTree() override;
    virtual void        readFromPTree(pt::ptree &root) override;

    virtual void set(CommsDriver &commsDriver, int motor) override;
    virtual void readFromFile(const std::string &filePath) override;
    virtual void writeToFile(const std::string &cfgFilePath) override;
    virtual void unitTest(CommsDriver &commsDriver, int motor) override;

    void    setCurrentModeCfg(CommsDriver &commsDriver, int motor);
    void    setVoltageModeCfg(CommsDriver &commsDriver, int motor);

    // All Configuration items relevant to the powerstepdriver
    CommonConfig    commonCfg_;
    CurrentModeCfg  currentModeCfg_;
    VoltageModeCfg  voltageModeCfg_;
};

std::string toString (const PowerStepCfg &cfg);
inline std::ostream& operator<<(std::ostream &os , const PowerStepCfg &cfg)
{
    os << toString(cfg);
    return os;
}

class PowerStepDriver : public BaseDriver
{
public:

    PowerStepDriver(const std::vector<StepperMotor> &motors, int spiBus = 0, CommsDebugLevel commsDebugLevel = CommsDebugNothing);
    PowerStepDriver(const std::vector<StepperMotor> &motors, std::vector<PowerStepCfg> &cfgs, int spiBus = 0, CommsDebugLevel commsDebugLevel = CommsDebugNothing);

    virtual void setConfig(AbstractConfig &cfg , int motor) override;
    PowerStepCfg getConfig(int motor);

private:
};
