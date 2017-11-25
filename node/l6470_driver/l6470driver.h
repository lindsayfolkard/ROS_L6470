#pragma once

#include "abstractdriver.h"
#include "basedriver.h"

// TODO !!

//class L6470Cfg : public AbstractConfig,
//                 public WriteableConfig
//{
//    //L6470Cfg (const CfgFile &cfgFile);

//    virtual void set(CommsDriver &commsDriver, int motor) override;
//    virtual void readFromFile(const std::string &filePath) override;
//    virtual void writeToFile(const std::string &cfgFilePath) override;

//    //void    setVoltageModeCfg(CommsDriver &commsDriver, int motor);

//    CommonConfig    commonCfg_;
//    VoltageModeCfg  voltageModeCfg_;
//};

///
/// \brief The L6470Driver class
///
//class L6470Driver : public BaseDriver
//{
//public:
//    //L6470Driver(const std::vector<StepperMotor> &motors, const std::vector <L6470Cfg> &cfgs, int spiBus = 0, CommsDebugLevel commsDebugLevel = CommsDebugNothing);
//};
