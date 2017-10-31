#include "basedriver.h"
#include "commands.h"
#include <mraa.hpp>
#include <exception>

BaseDriver::BaseDriver(const std::vector<StepperMotor> &motors, int spiBus, CommsDebugLevel commsDebugLevel):
    motors_(motors),
    commsDebugLevel_(commsDebugLevel)
{
    commsDriver_.reset(new CommsDriver(motors_.size(),spiBus));
}

//void
//BaseDriver::checkMotorIsValid(int motor)
//{
//    if (motor < 0 || (unsigned int)motor > motors_.size() )
//    {
//        throw; // TODO - fix which exception that is thrown
//    }
//}

//BaseDriver::BaseDriver(const std::vector<StepperMotor> &motors,
//                         const std::vector<Config>       &configs,
//                         int                              chipSelectPin,
//                         int                              resetPin,
//                         int                              busyPin,
//                         CommsDebugLevel                  commsDebugLevel):
//    BaseDriver(motors,chipSelectPin,resetPin,busyPin,commsDebugLevel)
//{
//    int motor=0;
//    for (const Config &cfg : configs)
//    {
//        setConfig(cfg,motor);
//        ++motor;
//    }

//}

/////////////////////////
/// Status Commands
/////////////////////////

// Contains all data in status command
// and also the current position and speed
std::vector<Status>
BaseDriver::getStatus()
{
    if (commsDebugLevel_ >= CommsDebugOnlyActions) 
    {
	std::cout << "++++++++++++++++ (CommsDebug) : Get Status ++++++++++++++++++++++" << std::endl << std::endl;
    }

    // Send the request
    // TODO - change to the command being sent (will clear any error flags)???
    commsDriver_->getParam(GET_STATUS);

    // Get the responses
    std::map<int,uint32_t> emptyMap;
    std::vector<uint32_t> states = commsDriver_->getParam(emptyMap,toBitLength(STATUS));

    // Parse the responses
    std::vector<Status> statusVector;
    for (auto state : states)
    {
        statusVector.push_back(parseStatus(state));
    }

    if (commsDebugLevel_ >= CommsDebugOnlyActions) 
    {
	std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl << std::endl;
    }

    return statusVector;
}

Status
BaseDriver::getStatus(int motor)
{
    checkMotorIsValid(motor);
    std::vector<Status> states = getStatus();
    return states[motor];
}

// Individual get functions if only very specific data needed
std::vector<bool>
BaseDriver::isBusy()
{
    std::vector<Status> statusVector =  getStatus();
    std::vector<bool> isBusyVector = {false};
    for (unsigned int i=0; i < statusVector.size() ; ++i)
    {
        isBusyVector[i] = statusVector[i].isBusy;
    }
    return isBusyVector;
}

bool
BaseDriver::isBusy(int motor)
{
    checkMotorIsValid(motor);
    std::vector<bool> busyVector = isBusy();

    return busyVector[motor];
}

std::vector<int32_t> BaseDriver::getPos()
{
    std::vector <uint32_t> positions = commsDriver_->getParam(ABS_POS);
    std::vector <int32_t> convertedPositions;
    for (auto element : positions)
    {
        // Since ABS_POS is a 22-bit 2's comp value, we need to check bit 21 and, if
        //  it's set, set all the bits ABOVE 21 in order for the value to maintain
        //  its appropriate sign.
        //if (element & 0x00200000) element |= 0xffc00000;
        convertedPositions.push_back(toSignedInt(element,toBitLength(ABS_POS)));
    }
    return convertedPositions;
}

int32_t BaseDriver::getPos(int motor)
{
    checkMotorIsValid(motor);
    return getPos()[motor];
}

std::vector<uint32_t>
BaseDriver::getSpeed()
{
    std::vector <uint32_t> speeds = commsDriver_->getParam(SPEED);

    for (auto &element : speeds)
    {
        element = spdParse(element);
    }
    return speeds;
}

uint32_t
BaseDriver::getSpeed(int motor)
{
    checkMotorIsValid(motor);
    std::vector<uint32_t> speeds = getSpeed();
    return speeds[motor];
}

std::vector<int32_t> BaseDriver::getMark()
{
    std::vector <uint32_t> marks = commsDriver_->getParam(MARK);
    std::vector <int32_t> convertedMarks;
    for (auto element : marks)
    {
        // Since ABS_POS is a 22-bit 2's comp value, we need to check bit 21 and, if
        //  it's set, set all the bits ABOVE 21 in order for the value to maintain
        //  its appropriate sign.
        //if (element & 0x00200000) element |= 0xffC00000;
        convertedMarks.push_back(toSignedInt(element,toBitLength(MARK)));
    }
    return convertedMarks;
}

int32_t BaseDriver::getMark(int motor)
{
    checkMotorIsValid(motor);
    std::vector<int32_t> marks = getMark();
    return marks[motor];
}

/////////////////////////
/// Operational Commands
////////////////////////

// Speed Commands

//  RUN sets the motor spinning in a direction (defined by the constants
//  FWD and REV). Maximum speed and minimum speed are defined
//  by the MAX_SPEED and MIN_SPEED registers; exceeding the FS_SPD value
//  will switch the device into full-step mode.
//  The spdCalc() function is provided to convert steps/s values into
//  appropriate integer values for this function.
void
BaseDriver::run(const std::map<int,DataCommand> &runCommands)
{
    commsDriver_->sendCommands(runCommands);
}

void
BaseDriver::run(const RunCommand &runCommand , int motor)
{
    commsDriver_->sendCommand(runCommand,motor);
}

// GoUntil will set the motor running with direction dir (REV or
//  FWD) until a falling edge is detected on the SW pin. Depending
//  on bit SW_MODE in CONFIG, either a hard stop or a soft stop is
//  performed at the falling edge, and depending on the value of
//  act (either RESET or COPY) the value in the ABS_POS register is
//  either RESET to 0 or COPY-ed into the MARK register.
void
BaseDriver::goUntil(const std::map <int,DataCommand> &goUntilCommands)
{
    commsDriver_->sendCommands(goUntilCommands);
}

void
BaseDriver::goUntil(const GoUntilCommand &command, int motor)
{
    commsDriver_->sendCommand(command,motor);
}

// Similar in nature to GoUntil, ReleaseSW produces motion at the
//  higher of two speeds: the value in MIN_SPEED or 5 steps/s.
//  The motor continues to run at this speed until a rising edge
//  is detected on the switch input, then a hard stop is performed
//  and the ABS_POS register is either COPY-ed into MARK or RESET to
//  0, depending on whether RESET or COPY was passed to the function
//  for act.

/// TODO !
//void
//BaseDriver::releaseSw(const std::map<int, ReleaseSwCommand> &releaseSWCommands)
//{
//    commsDriver_->commsDriver_->sendCommands(releaseSWCommands);
//}

//void
//BaseDriver::releaseSw(const ReleaseSwCommand &command , int motor)
//{
//    commsDriver_->sendCommand(command,motor);
//}

// Position Commands
void
BaseDriver::move(const std::map<int, DataCommand> &moveCommands)
{
    commsDriver_->sendCommands(moveCommands);
}

void
BaseDriver::move(const MoveCommand &command , int motor)
{
    commsDriver_->sendCommand(command,motor);
}

void
BaseDriver::goTo(const std::map<int, DataCommand> &goToCommands)
{
    commsDriver_->sendCommands(goToCommands);
}

void
BaseDriver::goTo(const GoToCommand &command , int motor)
{
    commsDriver_->sendCommand(command,motor);
}

void
BaseDriver::goToDir(const std::map<int, DataCommand> &goToDirCommands)
{
    commsDriver_->sendCommands(goToDirCommands);
}

void
BaseDriver::goToDir(const GoToDirCommand &command , int motor)
{
    commsDriver_->sendCommand(command,motor);
}

void
BaseDriver::goHome(const std::vector<int> &motors)
{
    commsDriver_->sendCommands(motors,GO_HOME);
}

void
BaseDriver::goHome(int motor)
{
    goHome(std::vector<int>{motor});
}

void
BaseDriver::goMark(const std::vector<int> &motors)
{
    commsDriver_->sendCommands(motors,GO_MARK);
}

void
BaseDriver::goMark(int motor)
{
    goMark(std::vector<int>{motor});
}

// Set Commands
//void
//BaseDriver::setMark(const std::map <int,long> &marks)
//{
//    setParam(MARK,marks);
//}

//void
//BaseDriver::setMark(long mark, int motor)
//{
//    std::map<int,long> marks = {{motor,mark}};
//}

//void
//BaseDriver::setPos(const std::map<int,long> &newPositions)
//{
//    setParam(ABS_POS,newPositions);
//}

void
BaseDriver::setPos(int32_t pos , int motor)
{
    setParam(ABS_POS , toTwosComplementUint(pos,toBitLength(ABS_POS)) , motor);
}

void
BaseDriver::resetPos(const std::vector<int> &motors)
{
    commsDriver_->sendCommands(motors,RESET_POS);
}

void
BaseDriver::resetPos(int motor)
{
    checkMotorIsValid(motor);
    std::vector<int> motors = {motor};
    resetPos(motors);
}

void
BaseDriver::resetDev(const std::vector<int> &motors)
{
    commsDriver_->sendCommands(motors,RESET_DEVICE);
}

// Stop Commands
void
BaseDriver::softStop(const std::vector<int> &motors)
{
    commsDriver_->sendCommands(motors,SOFT_STOP);
}

void
BaseDriver::softStop(int motor)
{
    checkMotorIsValid(motor);
    softStop(std::vector<int>{motor});
}

void
BaseDriver::hardStop(const std::vector<int> &motors)
{
    commsDriver_->sendCommands(motors,HARD_STOP);
}

void
BaseDriver::hardStop(int motor)
{
    checkMotorIsValid(motor);
    hardStop(std::vector<int>{motor});
}

void
BaseDriver::softHiZ(const std::vector<int> &motors)
{
    commsDriver_->sendCommands(motors,SOFT_HIZ);
}

void
BaseDriver::softHiZ(int motor)
{
    checkMotorIsValid(motor);
    softHiZ(std::vector<int>{motor});
}

void
BaseDriver::hardHiZ(const std::vector<int> &motors)
{
    commsDriver_->commsDriver_->sendCommands(motors,HARD_HIZ);
}

void
BaseDriver::hardHiZ(int motor)
{
    checkMotorIsValid(motor);
    hardHiZ(std::vector<int> {motor});
}
