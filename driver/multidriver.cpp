#include "multidriver.h"
#include <mraa.hpp>

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

MultiDriver::MultiDriver(const std::vector<StepperMotor> &motors,
                         const std::map <int,Config>     &config,
                         int                              chipSelectPin,
                         int                              resetPin,
                         int                              busyPin):
    MultiDriver(motors,chipSelectPin,resetPin,busyPin)
{
    setConfig(config);
}


/////////////////////////
/// Status Commands
/////////////////////////

// Contains all data in status command
// and also the current position and speed
std::vector<Status>
MultiDriver::getStatus()
{

}

// Individual get functions if only very specific data needed
std::vector<bool>
MultiDriver::isBusy()
{
    std::vector<Status> statusVector =  getStatus();
    std::vector<bool> isBusyVector = {false};
    for (int i=0; i < statusVector.size() ; ++i)
    {
        isBusyVector[i] = statusVector[i].isBusy;
    }
    return isBusyVector;
}

std::vector<long>
MultiDriver::getPos()
{
    std::vector <long> positions = getParam(ABS_POS);
    for (auto element : positions)
    {
        // Since ABS_POS is a 22-bit 2's comp value, we need to check bit 21 and, if
        //  it's set, set all the bits ABOVE 21 in order for the value to maintain
        //  its appropriate sign.
        if (element & 0x00200000) element |= 0xffc00000;
    }
    return positions;
}

std::vector<long>
MultiDriver::getSpeed()
{
    std::vector <long> speeds = getParam(SPEED);
    for (auto element : speeds)
    {
        element = spdCalc(element);
    }
    return speeds;
}

std::vector<long>
MultiDriver::getMark()
{
    long temp = getParam(MARK);

    std::vector <long> marks = getParam(MARK);
    for (auto element : marks)
    {
        // Since ABS_POS is a 22-bit 2's comp value, we need to check bit 21 and, if
        //  it's set, set all the bits ABOVE 21 in order for the value to maintain
        //  its appropriate sign.
        if (temp & 0x00200000) temp |= 0xffC00000;
    }
    return marks;
}

/////////////////////////
/// Configuration Commands
/////////////////////////

void
MultiDriver::setConfig(const std::map<int,Config> &cfg)
{
 // TODO
}

std::vector<Config>
MultiDriver::getConfig()
{
 // TODO if needed
}

void
MultiDriver::setProfileCfg(const std::map<int,ProfileCfg> &cfg)
{
// TODO
}

std::vector<ProfileCfg>
MultiDriver::getProfileCfg()
{
// TODO
}

/////////////////////////
/// Operational Commands
////////////////////////

// Speed Commands
void
MultiDriver::run(const std::map<int,RunCommand> &runCommands)
{

}

void
MultiDriver::goUntil(const std::map <int,GoUntilCommand> &goUntilCommands)
{

}

void
MultiDriver::stepClock(const std::map <int,MotorSpinDirection> &directions)
{

}

void
MultiDriver::releaseSw(const std::map <int,GoUntilCommand> &releaseSWCommands)
{

}

// Position Commands
void
MultiDriver::move(const std::map <int,MoveCommand> &moveCommands)
{

}

void
MultiDriver::goTo(const std::map <int,long> &positions)
{

}

void
MultiDriver::goToDir(const std::map <int,GoToCommand> &goToCommands)
{

}

void
MultiDriver::goHome(const std::map <int,bool> &goHome)
{

}

void
MultiDriver::goMark(const std::map <int,bool> &goMark)
{

}

// Set Commands
void
MultiDriver::setMaxSpeed(const std::map <int,float> &maxSpeeds)
{

}

void
MultiDriver::setMinSpeed(const std::map <int,float> &minSpeeds)
{

}

void
MultiDriver::setMark(const std::map <int,long> &marks)
{

}

void
MultiDriver::setPos(const std::map<int,long> &newPositions)
{

}

void
MultiDriver::resetPos(const std::map<int,bool> &resetPosition)
{

}

void
MultiDriver::resetDev(const std::map<int,bool> &resetDev)
{

}

// Stop Commands
void
MultiDriver::softStop(const std::map<int,bool> &resetPosition)
{

}

void
MultiDriver::hardStop(const std::map<int,bool> &resetPosition)
{

}

void
MultiDriver::softHiZ(const std::map<int,bool> &resetPosition)
{

}

void
MultiDriver::hardHiZ(const std::map<int,bool> &resetPosition)
{

}

/////////////////////////
/// Raw Access Set/Get Commands
////////////////////////
void
MultiDriver::setParam(ParamRegister param, std::vector <unsigned long> &values)
{

}

// Realize the "get parameter" function, to read from the various registers in
//  the dSPIN chip.
std::vector<long>
MultiDriver::getParam(ParamRegister param)
{
      uint8_t sendByte=GET_PARAM;
      sendByte |= param;

      // Send the param request
      std::vector<uint8_t> paramRequest(motors_.size(),(param | GET_PARAM));
      SPIXfer(paramRequest);

      // Parse the response
      return SPIXfer<long>();

      // TODO ???
      //return paramHandler(param, 0);
}

//std::vector<uint8_t>
//MultiDriver::SPIXfer(const std::map<int, uint8_t> &data)
//{
//    // Check map validity
//    checkMapIsValid<uint8_t>(data);

//    uint8_t sendPacket[motors_.size()] = {NOP};
//    uint8_t recvPacket[motors_.size()] = {NOP};

//    for (auto element : data)
//    {
//        sendPacket[data.first] = data.second;
//    }

//    SPI_->transfer(sendPacket,recvPacket,_numBoards);
//    //std::cout << "Transfer byte " << (int) sendPacket[_position] << std::endl;
//    //std::cout << "Result from transfer is " << (int) result << std::endl;
//    //std::cout << "return value is " << (int) recvPacket[_position] << std::endl;
//    return std::vector<uint8_t>(recvPacket,motors_.size());
//}

// R == recieve data type
// S == send data type
// TODO - should pretty much be the same data type/length everytime. TODO - check
template <typename R , typename S> std::vector<R>
MultiDriver::SPIXfer(const std::map<int, S> &data)
{
    // Check map validity
    checkMapIsValid<S>(data);

    // The L6470 Accepts data daisy chained in 1 byte blocks
    // (i.e controller1byte1,controller2byte1,...,<BREAK>,controller1byte2,controller2byte2,... etc..)s

    // NB: need to be careful here that we are dealing with contiguos memory ??
    std::vector<R> recvData(motors_.size(),0); // should init to all zeroes...

    for (int i=0; i < sizeof(S) ; ++i)
    {
        uint8_t byteSendPacket [motors_.size()]={NOP};
        uint8_t byteRecvPacket [motors_.size()]={NOP};

        for (const auto element : data)
        {
            byteSendPacket[data.first] = (data.second << (i*8)) & 0xFF;
        }

        SPI_->transfer(byteSendPacket,byteRecvPacket,_numBoards);

        for (int j=0; j < motors_.size();++j)
        {
            recvData[j] |= (byteRecvPacket[j] >> (i*8));
        }
    }


    //std::cout << "Transfer byte " << (int) sendPacket[_position] << std::endl;
    //std::cout << "Result from transfer is " << (int) result << std::endl;
    //std::cout << "return value is " << (int) recvPacket[_position] << std::endl;
    return recvData;
}

template <typename T> std::vector<T>
MultiDriver::SPIXfer(const std::vector<T> &data = std::vector<T>())
{
// TODO
}

//std::vector<long> MultiDriver::xferParam(const std::map<int,unsigned long> &parameters, uint8_t bitLen)
//{
//    uint8_t byteLen = bitLen/8;   // How many BYTES do we have?
//    if (bitLen%8 > 0) byteLen++;  // Make sure not to lose any partial uint8_t values.

//    uint8_t temp;

//    unsigned long retVal = 0;

//    for (int i = 0; i < byteLen; i++)
//    {
//      retVal = retVal << 8;
//      temp = SPIXfer((uint8_t)(value>>((byteLen-i-1)*8)));
//      retVal |= temp;
//    }

//    unsigned long mask = 0xffffffff >> (32-bitLen);
//    return retVal & mask;
//}

//long
//MultiDriver::paramHandler(uint8_t param, unsigned long value)
//{

//}
