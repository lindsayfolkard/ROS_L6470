#include "multidriver.h"
#include "commands.h"
#include <mraa.hpp>
#include <exception>

MultiDriver::MultiDriver(const std::vector<StepperMotor> &motors, int chipSelectPin, int resetPin, int busyPin):
    motors_(motors),
    chipSelectPin_(chipSelectPin),
    resetPin_(resetPin),
    busyPin_(busyPin)
{

  // Try to initialise the mraa::SPI port
  SPI_.reset(new mraa::Spi(0));
  SPI_->mode(mraa::SPI_MODE3);
  SPI_->frequency(4000000); // Can this be a bit higher ?
}

void
MultiDriver::checkMotorIsValid(int motor)
{
    if (motor < 0 || (unsigned int)motor > motors_.size() )
    {
        throw; // TODO - fix which exception that is thrown
    }
}

MultiDriver::MultiDriver(const std::vector<StepperMotor> &motors,
                         const std::vector<Config>       &configs,
                         int                              chipSelectPin,
                         int                              resetPin,
                         int                              busyPin):
    MultiDriver(motors,chipSelectPin,resetPin,busyPin)
{
    int motor=0;
    for (const Config &cfg : configs)
    {
        setConfig(cfg,motor);
        ++motor;
    }

}

/////////////////////////
/// Status Commands
/////////////////////////

// Contains all data in status command
// and also the current position and speed
std::vector<Status>
MultiDriver::getStatus()
{
    // Send the request
    SPIXfer(GET_STATUS);

    // Get the responses
    std::map<int,uint32_t> emptyMap;
    std::vector<uint32_t> states = SPIXfer(emptyMap,toBitLength(STATUS));

    // Parse the responses
    std::vector<Status> statusVector;
    for (auto state : states)
    {
        statusVector.push_back(parseStatus(state));
    }

    return statusVector;
}

Status
MultiDriver::getStatus(int motor)
{
    checkMotorIsValid(motor);
    std::vector<Status> states = getStatus();
    return states[motor];
}

// Individual get functions if only very specific data needed
std::vector<bool>
MultiDriver::isBusy()
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
MultiDriver::isBusy(int motor)
{
    checkMotorIsValid(motor);
    std::vector<bool> busyVector = isBusy();

    return busyVector[motor];
}

std::vector<long>
MultiDriver::getPos()
{
    std::vector <uint32_t> positions = getParam(ABS_POS);
    std::vector <long> convertedPositions;
    for (auto element : positions)
    {
        // Since ABS_POS is a 22-bit 2's comp value, we need to check bit 21 and, if
        //  it's set, set all the bits ABOVE 21 in order for the value to maintain
        //  its appropriate sign.
        if (element & 0x00200000) element |= 0xffc00000;
        convertedPositions.push_back(element);
    }
    return convertedPositions;
}

long
MultiDriver::getPos(int motor)
{
    checkMotorIsValid(motor);
    return getPos()[motor];
}

std::vector<long>
MultiDriver::getSpeed()
{
    std::vector <uint32_t> speeds = getParam(SPEED);
    std::vector <long> convertedSpeeds;
    for (auto element : speeds)
    {
        convertedSpeeds.push_back(spdCalc(element));
    }
    return convertedSpeeds;
}

long
MultiDriver::getSpeed(int motor)
{
    checkMotorIsValid(motor);
    std::vector<long> speeds = getSpeed();
    return speeds[motor];
}

std::vector<long>
MultiDriver::getMark()
{
    std::vector <uint32_t> marks = getParam(MARK);
    std::vector <long> convertedMarks;
    for (auto element : marks)
    {
        // Since ABS_POS is a 22-bit 2's comp value, we need to check bit 21 and, if
        //  it's set, set all the bits ABOVE 21 in order for the value to maintain
        //  its appropriate sign.
        if (element & 0x00200000) element |= 0xffC00000;
        convertedMarks.push_back(element);
    }
    return convertedMarks;
}

long
MultiDriver::getMark(int motor)
{
    checkMotorIsValid(motor);
    std::vector<long> marks = getMark();
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
MultiDriver::run(const std::map<int,DataCommand> &runCommands)
{
    sendCommands(runCommands);
}

void
MultiDriver::run(const RunCommand &runCommand , int motor)
{
    sendCommand(runCommand,motor);
}

// GoUntil will set the motor running with direction dir (REV or
//  FWD) until a falling edge is detected on the SW pin. Depending
//  on bit SW_MODE in CONFIG, either a hard stop or a soft stop is
//  performed at the falling edge, and depending on the value of
//  act (either RESET or COPY) the value in the ABS_POS register is
//  either RESET to 0 or COPY-ed into the MARK register.
void
MultiDriver::goUntil(const std::map <int,DataCommand> &goUntilCommands)
{
    sendCommands(goUntilCommands);
}

void
MultiDriver::goUntil(const GoUntilCommand &command, int motor)
{
    sendCommand(command,motor);
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
//MultiDriver::releaseSw(const std::map<int, ReleaseSwCommand> &releaseSWCommands)
//{
//    sendCommands(releaseSWCommands);
//}

//void
//MultiDriver::releaseSw(const ReleaseSwCommand &command , int motor)
//{
//    sendCommand(command,motor);
//}

// Position Commands
void
MultiDriver::move(const std::map<int, DataCommand> &moveCommands)
{
    sendCommands(moveCommands);
}

void
MultiDriver::move(const MoveCommand &command , int motor)
{
    sendCommand(command,motor);
}

void
MultiDriver::goTo(const std::map<int, DataCommand> &goToCommands)
{
    sendCommands(goToCommands);
}

void
MultiDriver::goTo(const GoToCommand &command , int motor)
{
    sendCommand(command,motor);
}

void
MultiDriver::goToDir(const std::map<int, DataCommand> &goToDirCommands)
{
    sendCommands(goToDirCommands);
}

void
MultiDriver::goToDir(const GoToDirCommand &command , int motor)
{
    sendCommand(command,motor);
}

void
MultiDriver::goHome(const std::vector<int> &motors)
{
    sendCommands(motors,GO_HOME);
}

void
MultiDriver::goHome(int motor)
{
    goHome(std::vector<int>{motor});
}

void
MultiDriver::goMark(const std::vector<int> &motors)
{
    sendCommands(motors,GO_MARK);
}

void
MultiDriver::goMark(int motor)
{
    goMark(std::vector<int>{motor});
}

// Set Commands
//void
//MultiDriver::setMark(const std::map <int,long> &marks)
//{
//    setParam(MARK,marks);
//}

//void
//MultiDriver::setMark(long mark, int motor)
//{
//    std::map<int,long> marks = {{motor,mark}};
//}

//void
//MultiDriver::setPos(const std::map<int,long> &newPositions)
//{
//    setParam(ABS_POS,newPositions);
//}

void
MultiDriver::setPos(long pos , int motor)
{
    /// TODO - handle the 2-s complement bigEndian transformation ???
    setParam(ABS_POS , (uint32_t)pos , motor);
}

void
MultiDriver::resetPos(const std::vector<int> &motors)
{
    sendCommands(motors,RESET_POS);
}

void
MultiDriver::resetPos(int motor)
{
    checkMotorIsValid(motor);
    std::vector<int> motors = {motor};
    resetPos(motors);
}

void
MultiDriver::resetDev(const std::vector<int> &motors)
{
    sendCommands(motors,RESET_DEVICE);
}

// Stop Commands
void
MultiDriver::softStop(const std::vector<int> &motors)
{
    sendCommands(motors,SOFT_STOP);
}

void
MultiDriver::softStop(int motor)
{
    checkMotorIsValid(motor);
    softStop(std::vector<int>{motor});
}

void
MultiDriver::hardStop(const std::vector<int> &motors)
{
    sendCommands(motors,HARD_STOP);
}

void
MultiDriver::hardStop(int motor)
{
    checkMotorIsValid(motor);
    hardStop(std::vector<int>{motor});
}

void
MultiDriver::softHiZ(const std::vector<int> &motors)
{
    sendCommands(motors,SOFT_HIZ);
}

void
MultiDriver::softHiZ(int motor)
{
    checkMotorIsValid(motor);
    softHiZ(std::vector<int>{motor});
}

void
MultiDriver::hardHiZ(const std::vector<int> &motors)
{
    sendCommands(motors,HARD_HIZ);
}

void
MultiDriver::hardHiZ(int motor)
{
    checkMotorIsValid(motor);
    hardHiZ(std::vector<int> {motor});
}
