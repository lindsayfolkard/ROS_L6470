#pragma once

#include "basedriver.h"
#include "config.h"

class PowerStepCfg : public AbstractConfig
{

public:

    virtual void set(CommsDriver &commsDriver, int motor) override;
    void         setCurrentModeCfg(CommsDriver &commsDriver, int motor);
    void         setVoltageModeCfg(CommsDriver &commsDriver, int motor);

    CommonConfig    commonCfg_;
    CurrentModeCfg  currentModeCfg_;
    VoltageModeCfg  voltageModeCfg_;
};

class PowerStepDriver : public BaseDriver
{
public:
    PowerStepDriver(const std::vector<StepperMotor> &motors, int spiBus = 0, CommsDebugLevel commsDebugLevel = CommsDebugNothing);
    //void setConfig(const std::vector<> &cfgs);
private:

};
