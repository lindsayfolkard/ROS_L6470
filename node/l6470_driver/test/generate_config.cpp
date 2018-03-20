#include "../config.h"
#include "../motor.h"
#include "../powerstepdriver.h"
#include <boost/property_tree/json_parser.hpp>

int main(int argc, char ** argv)
{
    Stepper_42BYGHW811 nema17Stepper;
    Stepper_57BYGH51   nema23SmallStepper;

    // Let's make the powerstepcfg
    PowerStepCfg cfg;
    cfg.commonCfg_.overCurrentThreshold = OCD_TH_1125m;
    cfg.voltageModeCfg_ = BackEmfConfigFromStepper(nema23SmallStepper);

    // Let's write the file in a nice manner
    pt::ptree overall;
    overall.add_child("StepperMotor",nema23SmallStepper.getPTree());
    overall.add_child("PowerStepCfg",cfg.getPTree());

    // Open the file and write to json

    {
        std::ofstream outFile;
        outFile.open("Nema23.cfg");
        //throw; // TODO - fix to real exception
        pt::write_json(outFile,overall);
    }

    // Open the file and read it all back again
    {
        // Let's do this in json format (easier to parse)
        pt::ptree root;

        try
        {
            pt::read_json("Nema23.cfg",root);
            StepperMotor stepper;
            stepper.readFromPTree(root.get_child("StepperMotor"));
            PowerStepCfg readBackCfg;
            readBackCfg.readFromPTree(root.get_child("PowerStepCfg"));

            std::cout << "Stepper motor read back is : " << std::endl << stepper << std::endl << std::endl;
            std::cout << "Power step Config read back is : " << std::endl << readBackCfg << std::endl << std::endl;
        }
        catch (std::exception &e)
        {
            std::cout << "Exception thrown while trying to read VoltageModeCfg from file with reason " << e.what();
            throw;
        }
    }


    // Let's make a
    //    OverallCfg overallCfg;
    //    overallCfg.commsDebugLevel_ = CommsDebugNothing;
    //    overallCfg.controllerType_  = PowerStep01;
    //    overallCfg.spiBus_ = 0;
    //    overallCfg.cfgFiles_.push_back();
    //    overallCfg.cfgFiles_.push_back(voltageMode17Cfg);

}
