#include <iostream>
#include "../config.h"
#include "../motor.h"
#include "../types.h"


int main (int argc, char ** argv)
{

    std::cout << "T1 : test generation and parsing of OverallCfg" << std::endl;
    CfgFile cfgM1("nema17.cfg","motor1.cfg","NEMA17H101");
    CfgFile cfgM2("nema23.cfg","motor2.cfg","NEMA23H201");
    std::vector <CfgFile> cfgs = {cfgM1,cfgM2};
    OverallCfg overallCfg(cfgs,PowerStep01,CommsDebugNothing);
    //std::cout << "Original Config : " << overallCfg << std::endl;
    overallCfg.writeToFile("test_overall.cfg");
    OverallCfg duplicateCfg("test_overall.cfg");
    duplicateCfg.writeToFile("new_overall.cfg");
    //std::cout << "Reread Config : " << duplicateCfg;
    std::cout << "T1 : passed!" << std::endl;

    std::cout << "T2 : test parsing of commonConfig" << std::endl;
    CommonConfig commonConfig;
    std::cout << "Original commonConfig : " << commonConfig << std::endl;
    commonConfig.writeToFile("test_commonConfig.cfg");
    CommonConfig otherConfig("test_commonConfig.cfg");
    std::cout << "Reparse commonConfig : " << otherConfig;
    otherConfig.writeToFile("test_t2_commonConfig.cfg");
    std::cout << "T2 : passed!" << std::endl;
    
    std::cout << "T2 : test parsing of Voltage Mode Cfg" << std::endl;
    VoltageModeCfg voltageModeConfig;
    voltageModeConfig.writeToFile("voltage_mode_config.cfg");
    VoltageModeCfg newCfg("voltage_mode_config.cfg");
    std::cout << "Original config : " << voltageModeConfig << std::endl;
    std::cout << "Reparsed config : " << newCfg << std::endl;
    std::cout << "T2 : passed!" << std::endl;
    
    


}
