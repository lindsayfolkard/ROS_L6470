#include "../config.h"
#include "../motor.h"
#include "../powerstepdriver.h"
#include <boost/property_tree/json_parser.hpp>

int main(int argc, char ** argv)
{
    (void) argc;
    (void) argv;

    //Stepper_42BYGHW811 stepper;
    //Stepper_57BYGH51 stepper;
    Stepper_Medium   stepper;

    // Let's make the powerstepcfg
    PowerStepCfg cfg;
    cfg.commonCfg_.overCurrentThreshold = OCD_TH_11250m;
    cfg.commonCfg_.stepMode = STEP_SEL_1_8;
    cfg.voltageModeCfg_ = BackEmfConfigFromStepper(stepper);

    // Let's write the file in a nice manner
    pt::ptree overall;
    overall.add_child("StepperMotor",stepper.getPTree());
    overall.add_child("PowerStepCfg",cfg.getPTree());

    // Open the file and write to json
    const std::string fileName = stepper.motorModel + ".cfg";
    {
        std::ofstream outFile;
        outFile.open(fileName);
        //throw; // TODO - fix to real exception
        pt::write_json(outFile,overall);
    }

    // Open the file and read it all back again
    {
        // Let's do this in json format (easier to parse)
        pt::ptree root;

        try
        {
            pt::read_json(fileName,root);
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


    // Let's make an overall config
    OverallCfg overallCfg;
    overallCfg.commsDebugLevel_ = CommsDebugNothing;
    overallCfg.controllerType_  = PowerStep01;
    overallCfg.spiBus_ = 0;
    overallCfg.cfgFiles_.push_back("Motor1.cfg");
    overallCfg.cfgFiles_.push_back("Motor2.cfg");
    overallCfg.cfgFiles_.push_back("Motor3.cfg");

    overallCfg.writeToFile("Node.cfg");

}
