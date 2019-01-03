#include "simdriver.h"

namespace
{
    template <typename T> void printMap (const std::map<int,T> &map)
    {
        for (const auto &element : map)
            std::cout << "Motor " << element.first << " --> " << element.second << std::endl;
    }

    std::string getVectorString(const std::vector<int> &motors)
    {
        std::stringstream ss;

        ss << "[ ";

        for (const auto &element : motors)
        {
            ss << element << " ";
        }

        ss << "]";

        return ss.str();
    }
}

SimDriver::SimDriver(const std::vector<StepperMotor> &motors, bool debugEnabled):
    motors_(motors),
    debugEnabled_(debugEnabled),
    positions_(motors.size(),0),
    speeds_(motors_.size(),0),
    status_(motors.size(),Status())
{
    // TODO - initialise the current command
}

SimDriver::SimDriver(const std::vector<StepperMotor> &motors,
                           std::vector<PowerStepCfg> &cfgs, bool debugEnabled):
    SimDriver(motors,debugEnabled)
{
    cfgs_ = cfgs;

    std::cout << "========= Motor Configs ==========" << std::endl;
    int count=0;
    for (const auto &motor : motors_)
    {
        std::cout << "Motor " << count << std::endl << motor;
        ++count;
    }
    std::cout << "==================================" << std::endl;

    std::cout << "======= Power Step Configs =======" << std::endl;
    count=0;
    for (const auto &cfg : cfgs_)
    {
        std::cout << "Motor " << count << std::endl << cfg;
        ++count;
    }
    std::cout << "==================================" << std::endl;

}

// Contains all data in status command
// and also the current position and speed
std::vector<Status> SimDriver::getStatus()
{
    if (debugEnabled_)
        std::cout << "SimDriver : " << __func__  << std::endl;

    updateState();

    return status_;
}

Status
SimDriver::getStatus(int motor)
{
    if (debugEnabled_)
        std::cout << "SimDriver : " << __func__  << std::endl;

    updateState();

    return status_[motor];
}

std::vector<Status>
SimDriver::clearStatus()
{
    if (debugEnabled_)
        std::cout << "SimDriver : " << __func__  << std::endl;

    for (auto &element : status_)
        element = Status();

    return getStatus();
}

// Individual get functions if only very specific data needed
std::vector<bool>
SimDriver::isBusy()
{
    if (debugEnabled_)
        std::cout << "SimDriver : " << __func__  << std::endl;

    std::vector<bool> isBusyVector(motors_.size());
    return isBusyVector;
}

bool
SimDriver::isBusy(int motor)
{
    if (debugEnabled_)
        std::cout << "SimDriver : " << __func__  << " for motor" << motor << std::endl;

    return false;
}

std::vector<int32_t>
SimDriver::getPos()
{
    if (debugEnabled_)
        std::cout << "SimDriver : " << __func__  << std::endl;

    updateState();

    return positions_;
}

int32_t
SimDriver::getPos(int motor)
{
    if (debugEnabled_)
        std::cout << "SimDriver : " << __func__  << std::endl;

    updateState();

    return positions_[motor];
}

std::vector<uint32_t>
SimDriver::getSpeed()
{
    if (debugEnabled_)
        std::cout << "SimDriver : " << __func__  << std::endl;

    updateState();

    return speeds_;
} // steps/s

uint32_t
SimDriver::getSpeed(int motor)
{
    if (debugEnabled_)
        std::cout << "SimDriver : " << __func__  << std::endl;

    updateState();

    return speeds_[motor];
}

std::vector<int32_t>
SimDriver::getMark()
{
    if (debugEnabled_)
        std::cout << "SimDriver : " << __func__  << std::endl;

    std::vector <int32_t> positions(motors_.size(),0);
    return positions;
}  

int32_t
SimDriver::getMark(int motor)
{
    if (debugEnabled_)
        std::cout << "SimDriver : " << __func__  << " for motor " << motor << std::endl;

    return 0;
}

////////////////////////////////////////
/// Profile Configuration Commands
////////////////////////////////////////

void
SimDriver::setConfig(AbstractConfig &config , int motor)
{
    if (debugEnabled_)
    {
        std::cout << "SimDriver : " << __func__  << std::endl;
        std::cout << "SimDriver : set config to motor " << motor << std::endl;
    }

    // Do nothing...
    (void)config;

}

void
SimDriver::setProfileCfg(const std::map<int,ProfileCfg> &cfgs)
{
    if (debugEnabled_)
        std::cout << "SimDriver : " << __func__  << std::endl;

    profileCfgs_ = cfgs;
}

void
SimDriver::setProfileCfg(const ProfileCfg &cfg , int motor)
{
    if (debugEnabled_)
        std::cout << "SimDriver : " << __func__  << std::endl;

    profileCfgs_.insert(std::pair<int,ProfileCfg>(motor,cfg));

    // TODO - change speed of motors if moving
}

//Profile Raw parameters
void
SimDriver::setAcc(std::map<int,float> &accelerations)
{
    if (debugEnabled_)
    {
        std::cout << "SimDriver : " << __func__  << std::endl;
        std::cout << "size of map is :  " << accelerations.size() << std::endl;
    }
    // TODO - actually set in

}

void SimDriver::setAcc(float stepsPerSecondPerSecond , int motor )
{
    if (debugEnabled_)
        std::cout << "SimDriver : " << __func__  << std::endl;

    std::map<int,float> accelerations;
    accelerations.insert(std::pair<int,float>(motor,stepsPerSecondPerSecond));
    setAcc(accelerations);
}

void
SimDriver::setDec(std::map<int,float> &decelerations)
{
    if (debugEnabled_)
    {
        std::cout << "SimDriver : " << __func__  << std::endl;
        std::cout << "size of map is :  " << decelerations.size() << std::endl;
    }
    // TODO
}

void
SimDriver::setDec(float stepsPerSecondPerSecond , int motor )
{
    if (debugEnabled_)
        std::cout << "SimDriver : " << __func__  << std::endl;

    std::map<int,float> decelMap;
    decelMap.insert(std::pair<int,float>(motor,stepsPerSecondPerSecond));
    setDec(decelMap);
}

void
SimDriver::setMaxSpeed(const std::map <int,float> &maxSpeeds)
{
    if (debugEnabled_)
    {
        std::cout << "SimDriver : " << __func__  << std::endl;
        std::cout << "size of map is :  " << maxSpeeds.size() << std::endl;
    }

    // TODO
}

void
SimDriver::setMaxSpeed(float stepsPerSecond , int motor)
{
    if (debugEnabled_)
    {
        std::cout << "SimDriver : " << __func__  << std::endl;
        std::cout << "Set max speed = " << stepsPerSecond << " steps/s for motor " << motor << std::endl;
    }
    // TODO
}

void
SimDriver::setMinSpeed(const std::map <int,float> &minSpeeds)
{
    if (debugEnabled_)
    {
        std::cout << "SimDriver : " << __func__  << std::endl;
        std::cout << "SimDriver : " << "Debug - minSpeeds length is " << minSpeeds.size() << std::endl;
    }
    // TODO
}

void SimDriver::setMinSpeed(float stepsPerSecond , int motor)
{
    if (debugEnabled_)
    {
        std::cout << "SimDriver : " << __func__  << std::endl;
        std::cout << "SimDriver : " << "set min speed to " << stepsPerSecond << " steps/s for motor " << motor << std::endl;
    }

    // TODO
}

ProfileCfg
SimDriver::getProfileCfg(int motor)
{
    if (debugEnabled_)
        std::cout << "SimDriver : " << __func__  << std::endl;

    return profileCfgs_[motor];
}

// void setFullSpeed(float stepsPerSecond){ }

//////////////////////////////
/// Operational Commands
/////////////////////////////

// Speed Commands
void
SimDriver::run(const std::map<int, RunCommand> &runCommands)
{
    if (debugEnabled_)
    {
        std::cout << "SimDriver : " << __func__  << std::endl;
        std::cout << "RunCommands (size = " << runCommands.size() << ")" << std::endl;
    }
    // TODO
}

void
SimDriver::run(const RunCommand &runCommand , int motor)
{
    if (debugEnabled_)
    {
        std::cout << "SimDriver : " << __func__  << std::endl;
        std::cout << "SimDriver : have " << runCommand << " for motor " << motor << std::endl;
    }
    // TODO
}

void
SimDriver::goUntil(const std::map<int, GoUntilCommand> &goUntilCommands)
{
    if (debugEnabled_)
    {
        std::cout << "SimDriver : " << __func__  << std::endl;
        std::cout << "SimDriver : goUntilCommands size = " << goUntilCommands.size() << std::endl;
    }
    // TODO
}

void
SimDriver::goUntil(const GoUntilCommand &command , int motor)
{
    if (debugEnabled_)
    {
        std::cout << "SimDriver : " << __func__  << std::endl;
        std::cout << "Command = " << command << " for motor " << motor << std::endl;
    }
    // TODO
}

//    void releaseSw(const std::map <int,ReleaseSwCommand> &releaseSWCommands){ }
//    void releaseSw(const ReleaseSwCommand &command , int motor){ }

// Position Commands
void
SimDriver::move(const std::map<int, MoveCommand> &moveCommands)
{
    if (debugEnabled_)
    {
        std::cout << "SimDriver : " << __func__  << std::endl;
        std::cout << "Map is : " << std::endl;
        printMap<MoveCommand>(moveCommands);
        std::cout << std::endl;
    }
    // TODO
}

void
SimDriver::move(const MoveCommand &command , int motor)
{
    if (debugEnabled_)
    {
        std::cout << "SimDriver : " << __func__  << std::endl;
        std::cout << "MoveCommands = " << command << " for motor " << motor << std::endl;
    }
    // TODO
}

void
SimDriver::goTo(const std::map<int, GoToCommand> &goToCommands)
{
    if (debugEnabled_)
    {
        std::cout << "SimDriver : " << __func__  << std::endl;
        std::cout << "Map is : " << std::endl;
        printMap<GoToCommand>(goToCommands);
        std::cout << std::endl;
    }

    // TODO
}

void
SimDriver::goTo(const GoToCommand &command , int motor)
{
    if (debugEnabled_)
    {
        std::cout << "SimDriver : " << __func__  << std::endl;
        std::cout << "Command = " << command << " for motor " << motor << std::endl;
    }
    // TODO
}

void
SimDriver::goToDir(const std::map<int, GoToDirCommand> &goToDirCommands)
{
    if (debugEnabled_)
    {
        std::cout << "SimDriver : " << __func__  << std::endl;
        std::cout << "Map is : " << std::endl;
        printMap<GoToDirCommand>(goToDirCommands);
        std::cout << std::endl;
    }

    // TODO
}

void
SimDriver::goToDir(const GoToDirCommand &command , int motor)
{
    if (debugEnabled_)
    {
        std::cout << "SimDriver : " << __func__  << std::endl;
        std::cout << "Command = " << command << " for motor " << motor << std::endl;
    }
    // TODO
}

void
SimDriver::goHome(const std::vector <int> &motors)
{
    if (debugEnabled_)
    {
        std::cout << "SimDriver : " << __func__  << std::endl;
        std::cout << "Go Home for motors " << getVectorString(motors) << std::endl;
    }
    // TODO
}
void
SimDriver::goHome(int motor)
{
    if (debugEnabled_)
        std::cout << "SimDriver : " << __func__  << " for motor " << motor << std::endl;
    // TODO
}

void
SimDriver::goMark(const std::vector <int> &motors)
{
    if (debugEnabled_)
    {
        std::cout << "SimDriver : " << __func__  << std::endl;
        std::cout << "GoMark for motors " << getVectorString(motors) << std::endl;
    }

    // TODO
}

void
SimDriver::goMark(int motor)
{
    if (debugEnabled_)
        std::cout << "SimDriver : " << __func__  << " for motor " << motor << std::endl;
    // TODO
}

// Stop Commands

void
SimDriver::stopAllHard()
{
    if (debugEnabled_)
        std::cout << "SimDriver : " << __func__  << std::endl;
    // TODO
}

void
SimDriver::stopAllSoft()
{
    if (debugEnabled_)
        std::cout << "SimDriver : " << __func__  << std::endl;
    // TODO
}

void
SimDriver::softStop(const std::vector <int> &motors)
{
    if (debugEnabled_)
    {
        std::cout << "SimDriver : " << __func__  << std::endl;
        std::cout << "SoftStop motors " << getVectorString(motors) << std::endl;
    }
    // TODO
}

void
SimDriver::softStop(int motor)
{
    if (debugEnabled_)
        std::cout << "SimDriver : " << __func__  << " for motor " << motor << std::endl;
    // TODO
}

void
SimDriver::hardStop(const std::vector <int> &motors)
{
    if (debugEnabled_)
    {
        std::cout << "SimDriver : " << __func__  << std::endl;
        std::cout << "HardStop Motors  : " << getVectorString(motors) << std::endl;
    }

    // TODO
}

void
SimDriver::hardStop(int motor)
{
    if (debugEnabled_)
        std::cout << "SimDriver : " << __func__  << " for motor " << motor << std::endl;

    // TODO
}

void
SimDriver::softHiZ(const std::vector <int> &motors)
{
    if (debugEnabled_)
    {
        std::cout << "SimDriver : " << __func__  << std::endl;
        std::cout << "SoftHiz for motors " << getVectorString(motors) << std::endl;
    }
    // TODO
}

void
SimDriver::softHiZ(int motor)
{
    if (debugEnabled_)
        std::cout << "SimDriver : " << __func__  << " for motor " << motor << std::endl;
    // TODO
}

void
SimDriver::hardHiZ(const std::vector <int> &motors)
{
    if (debugEnabled_)
    {
        std::cout << "SimDriver : " << __func__  << std::endl;
        std::cout << "HardHiZ for motors : " << getVectorString(motors) << std::endl;
    }
    // TODO
}

void
SimDriver::hardHiZ(int motor)
{
    if (debugEnabled_)
        std::cout << "SimDriver : " << __func__  << " for motor " << motor << std::endl;
    // TODO
}

// Set Commands
//void setMark(const std::map<int, long> &marks){ }
//void setPos(const std::map<int,long> &newPositions){ }
void
SimDriver::setPos  (int32_t pos , int motor)
{
    if (debugEnabled_)
        std::cout << "SimDriver : " << __func__  << " with pos = " << pos << " for motor " << motor << std::endl;
    // TODO
}

void
SimDriver::setMark(int32_t pos, int motor)
{
    if (debugEnabled_)
        std::cout << "SimDriver : " << __func__  << " with pos = " << pos << " for motor " << motor << std::endl;
    // TODO
    // do nothing
}

void
SimDriver::setAllPos(int32_t pos)
{
    if (debugEnabled_)
        std::cout << "SimDriver : " << __func__  << " with pos = " << pos << std::endl;

    // TODO -
}

void
SimDriver::resetPos(const std::vector <int> &motors)
{
    if (debugEnabled_)
        std::cout << "SimDriver : " << __func__  << " for motor " << getVectorString(motors) << std::endl;
    // TODO
}

void
SimDriver::resetPos(int motor)
{
    if (debugEnabled_)
        std::cout << "SimDriver : " << __func__  << " for motor " << motor << std::endl;
    // TODO
}

void
SimDriver::resetDev(const std::vector <int> &motors)
{
    if (debugEnabled_)
        std::cout << "SimDriver : " << __func__  << " for motors " << getVectorString(motors) << std::endl;
    // TODO
}

void
SimDriver::updateState()
{

//    for (int motor=0; motor < currentCommand_.size(); ++motor)
//    {
//        if (!currentCommand_[motor].second)
//        {
//            std::cout << "WTF - cannot updateState for motor " << motor << " since the unique ptr is not defined..";
//            // throw ? probs
//        }
//        const Command cmdFlag = currentCommand_[motor].second->cmd;

//        if (cmdFlag == MOVE)
//        {
//            MoveCommand moveCommand = dynamic_cast<MoveCommand>(*currentCommand_[motor].second);

//            const std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
//            const std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - currentCommand_[motor].first);

//            // Handle when it is accelerating...

//            positions_[motor] = positions_[motor] + std::chrono::duration()

//        }
//        else if (cmdFlag == GOTO)
//        {
//            GoToCommand goToCommand = dynamic_cast<GoToCommand>(*currentCommand_[motor].second);
//        }
//        else if ( cmdFlag == GOTO_DIR)
//        {
//            GoToDirCommand goToDirCommand = dynamic_cast<GoToDirCommand>(*currentCommand_[motor].second);
//        }
//        else if ( cmdFlag == GO_UNTIL)
//        {
//            GoUntilCommand goUntilCommand = dynamic_cast<GoUntilCommand>(*currentCommand_[motor].second);
//        }
//        //        else if ( RELEASE_SW  )
//        //        else if ( GO_HOME     ) {
//        //        else if ( GO_MARK     ) {
//        //        else if ( RESET_POS   ) {
//        //        else if ( RESET_DEVICE) {
//        //        else if ( SOFT_STOP   ) {
//        //        else if ( HARD_STOP   ) {
//        //        else if ( SOFT_HIZ    ) {
//        //        else if ( HARD_HIZ    ) {

//        }
}

