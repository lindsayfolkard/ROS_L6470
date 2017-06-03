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
    // Send the request
    SPIXfer(GET_STATUS);

    // Get the responses
    std::map<int,uint16_t> emptyMap();
    std::vector<uint16_t> states = SPIXfer(emptyMap());

    // Parse the responses
    // TODO - once it is abstracted
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
MultiDriver::setParam(ParamRegister param, std::map <int,T> &values)
{
    uint8_t sendByte=SET_PARAM;
    sendByte |= param;

    // Send the param request
    SPIXfer(sendByte);

    // Set the value of each parameter as needed
    xferParam(values,toBitLength(param));
}

// Realize the "get parameter" function, to read from the various registers in
//  the dSPIN chip.
std::vector<T>
MultiDriver::getParam(ParamRegister param)
{
      uint8_t sendByte=GET_PARAM;
      sendByte |= param;

      // Send the param request
      SPIXfer(sendByte);

      // Parse the response from multiple boards
      std::map<int,T> emptyMap();
      return xferParam(emptyMap(),toBitLength(param));
}

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

    // < S or < R ???
    for (int i=0; i < sizeof(R) ; ++i)
    {
        uint8_t byteSendPacket [motors_.size()]={NOP};
        uint8_t byteRecvPacket [motors_.size()]={NOP};

        for (const auto element : data)
        {
            byteSendPacket[data.first] = (data.second << (i*8)) & 0xFF;
        }

        SPI_->transfer(byteSendPacket,byteRecvPacket,motors_.size());

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

template <typename R> std::vector<R>
MultiDriver::SPIXfer(uint8_t requestValue)
{
    // Create an appropriate map
    std::map<int,uint8_t> request;
    for (int i=0 ; i < motors_.size() ; ++i)
    {
        map.insert(std::pair<int,uint8_t>(i,requestValue));
    }

    // Send the request
    SPIXfer(request);
}

std::vector<R>
MultiDriver::xferParam(const std::map<int,S> &parameters, uint8_t bitLen)
{
    uint8_t byteLen = bitLen/8;   // How many BYTES do we have?
    if (bitLen%8 > 0) byteLen++;  // Make sure not to lose any partial uint8_t values.

    // Send and recieve the response
    std::vector<R> response = SPIXfer(parameters);

    // Trim the response
    unsigned long mask = 0xffffffff >> (32-bitLen);
    for (auto element : response)
    {
        element = element & mask;
    }

    return response;
}
