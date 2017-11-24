#include <iostream>
#include "../config.h"
#include "../motor.h"
#include "../types.h"


int main (int argc, char ** argv)
{

    std::cout << "T1 : test generation and parsing of OverallCfg" << std::endl;
    CfgFile cfgM1("nema17.cfg","motor1.cfg");
    CfgFile cfgM2("nema23.cfg","motor2.cfg");
    std::vector <CfgFile> cfgs = {cfgM1,cfgM2};
    OverallCfg overallCfg(cfgs,PowerStep01,CommsDebugNothing);
    overallCfg.writeToFile("test_overall.cfg");
    OverallCfg duplicateCfg("test_overall.cfg");
    std::cout << "T1 : passed!" << std::endl;

}
