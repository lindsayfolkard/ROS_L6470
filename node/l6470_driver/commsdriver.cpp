#include "commsdriver.h"
#include <iostream>
#include "types.h"
#include <map>
#include <vector>

std::string toString(CommsDebugLevel commsDebugLevel)
{
    switch (commsDebugLevel)
    {
    case CommsDebugNothing:
        return "CommsDebugNothing";
    case CommsDebugOnlyActions:
        return "CommsDebugActions";
    case CommsDebugEverything:
        return "CommsDebugEverything";
    default:
        throw; // !TODO - throw a meaningful exception
    }
}

CommsDriver::CommsDriver(int numMotors, int spiBus, CommsDebugLevel commsDebugLevel) :
    numMotors_(numMotors),
    commsDebugLevel_(commsDebugLevel)
{
    // Try to initialise the mraa::SPI port
    SPI_.reset(new mraa::Spi(spiBus));
    SPI_->mode(mraa::SPI_MODE3);
    SPI_->frequency(4000000); // Can this be a bit higher ?
}

///
/// \brief Functions for handling comms with daisy chained L6470H boards
///
void
CommsDriver::checkMotorIsValid(int motor)
{
    if (motor < 0 || motor > numMotors_)
        throw; // TODO - throw propere exception
}

void
CommsDriver::setParam(uint8_t paramRegister, uint8_t bitLength, std::map<int, uint32_t> &values)
{
    checkMapIsValid<uint32_t>(values);

    if (commsDebugLevel_ >= CommsDebugOnlyActions)
    {
        std::cout << "||||||||||||||||||||||||||||||||||||||||||||||||||||||" << std::endl;
        std::cout << "(CommsDebug) : Set Param [" << (ParamRegister) paramRegister << "] with map : "
                  << std::endl << toMapString(values,toBitLength(paramRegister)) << std::endl;
    }

    // Structure the command request
    uint8_t sendByte=SET_PARAM;
    sendByte |= paramRegister;

    // Structure the request
    std::map<int,uint32_t> commandSet;
    for (const auto element : values)
    {
        commandSet.insert(std::pair<int,uint32_t>(element.first,sendByte));
    }

    // Send the command set
    SPIXfer(commandSet,8/*bitLength*/);

    // Set the value of each parameter as needed
    xferParam(values,bitLength);

    if (commsDebugLevel_ >= CommsDebugOnlyActions)
        std::cout << "||||||||||||||||||||||||||||||||||||||||||||||||||||||" << std::endl << std::endl;

}
void
CommsDriver::setParam(uint8_t paramRegister, uint8_t bitLength, uint32_t value , int motor)
{
    checkMotorIsValid(motor);
    std::map<int,uint32_t> map = {{motor,value}};
    setParam(paramRegister,bitLength,map);
}

// Realize the "get parameter" function, to read from the various registers in
//  the dSPIN chip.
std::vector<uint32_t>
CommsDriver::getParam(uint8_t paramRegister, uint8_t bitLength)
{
    uint8_t sendByte=GET_PARAM;
    sendByte |= paramRegister;

    if (commsDebugLevel_ >= CommsDebugOnlyActions)
    {
        std::cout << "___________________________________________" << std::endl;
        std::cout << "(CommsDebug) : Get Param [" << (ParamRegister) paramRegister << "] " << std::endl << std::endl;
    }

    // Send the param request
    SPIXfer(sendByte);

    if (commsDebugLevel_ >= CommsDebugOnlyActions)
    {
        std::cout << "(CommsDebug) : Got Param [" << (ParamRegister) paramRegister << "]" << std::endl;
        std::cout << "___________________________________________" << std::endl << std::endl;
    }

    // Parse the response from multiple boards
    const std::map<int,uint32_t> emptyMap;
    return xferParam(emptyMap,bitLength);
}

uint32_t
CommsDriver::getParam(uint8_t paramRegister, uint8_t bitLength, int motor)
{
    checkMotorIsValid(motor);

    std::vector<uint32_t> values = getParam(paramRegister,bitLength);

    return values[motor];
}

std::vector<uint32_t>
CommsDriver::getParam(uint8_t paramRegister)
{
    return getParam(paramRegister,toBitLength(paramRegister));
}


std::vector<uint32_t>
CommsDriver::SPIXfer(const std::map<int, uint32_t> &data , int bitLength)
{
    // Check map validity
    checkMapIsValid<uint32_t>(data);

    if (commsDebugLevel_ == CommsDebugEverything)
    {

        std::cout << "-------------- (CommsDebug) : SPIXfer -----------------" << std::endl
                  << "bitLength = " << bitLength << " ,  Data (length = " << data.size() << ") -->"
                  << std::endl << toMapString(data,bitLength) << std::endl;
    }

    // The L6470 Accepts data daisy chained in 1 byte blocks
    // (i.e controller1byte1,controller2byte1,...,<BREAK>,controller1byte2,controller2byte2,... etc..)
    // This is kinda annoying, but fuck it also is useful in getting somewhat exact timing with control

    // NB: need to be careful here that we are dealing with contiguos memory ??
    std::vector<uint32_t> recvData(numMotors_,0); // should init to all zeroes...

    // Determine the number of bytes to send
    uint8_t byteLength = bitLength/8;   // How many BYTES do we have?
    if (bitLength%8 > 0) byteLength++;  // Make sure not to lose any partial uint8_t values.

    // Send the data to the boards one byte at a time
    // The L6470 expects numbers in big-endian format (because it is fucking annoying like that), however this
    // is running on a linux which is little-endian. Hence we need to reverse the order in which we send bytes
    for (int i=0; i < byteLength ; ++i)
    {
        std::vector<uint8_t> byteSendPacket(numMotors_,NOP);
        std::vector<uint8_t> byteRecvPacket(numMotors_,NOP);

        for (const auto element : data)
        {
            byteSendPacket[element.first] = (element.second >> ((byteLength-i-1)*8));// & 0xFF;
        }

        // Debug if needed
        if (commsDebugLevel_ == CommsDebugEverything)
        {
            std::cout << "(CommsDebug) : SPI Transfer Packet " << i+1 <<" of " << (int)byteLength << " --> " << toLineString(byteSendPacket) << std::endl;
        }

        SPI_->transfer(&byteSendPacket[0],&byteRecvPacket[0],numMotors_);

        // Debug if needed
        if (commsDebugLevel_ == CommsDebugEverything)
        {
            std::cout << "(CommsDebug) : SPI Recieve Packet " << i+1 <<" of " << (int)byteLength << " --> " << toLineString(byteRecvPacket) << std::endl;
        }

        // I presume the L6470 also responds in big-endian format
        for (int j=0; j < numMotors_;++j)
        {
            // Hack - was not commented
            recvData[j] = recvData[j] << 8;
            recvData[j] |= byteRecvPacket[j];

            // Hack - was commented
            //recvData[j] |= (byteRecvPacket[j] >> (i*8));
        }
    }

    if (commsDebugLevel_ == CommsDebugEverything)
    {

        std::cout << "------------------------------------------------------" << std::endl <<std::endl;
    }

    return recvData;
}

// Will send request for all motors
std::vector<uint32_t>
CommsDriver::SPIXfer(uint8_t requestValue)
{
    // Create an appropriate map
    std::map<int,uint32_t> requests;
    for (int i=0 ; i < numMotors_ ; ++i)
    {
        requests.insert(std::pair<int,uint32_t>(i,requestValue));
    }

    // Send the request
    return SPIXfer(requests,8 /*bitLength*/);
}

uint32_t
CommsDriver::SPIXfer(uint8_t requestValue, int motor)
{
    checkMotorIsValid(motor);

    // Create an appropriate map
    std::map<int,uint32_t> requests;
    requests.insert(std::pair<int,uint32_t>(motor,requestValue));

    // Send the request
    std::vector<uint32_t> replies = SPIXfer(requests,8 /*bitLength*/);
    return replies[motor];
}

std::vector<uint32_t>
CommsDriver::xferParam(const std::map<int,uint32_t> &parameters, uint8_t bitLength)
{
    checkMapIsValid<uint32_t>(parameters);

    uint8_t byteLength = bitLength/8;   // How many BYTES do we have?
    if (bitLength%8 > 0) byteLength++;  // Make sure not to lose any partial uint8_t values.

    // Send and recieve the response
    std::vector<uint32_t> response = SPIXfer(parameters,bitLength);

    // Trim the response
    uint32_t mask = 0xffffffff >> (32-bitLength);
    for (auto element : response)
    {
        element = element & mask;
    }

    return response;
}

template <class T> void
CommsDriver::sendCommands(const std::map <int,T> &dataCommands)
{
    checkMapIsValid<T>(dataCommands);

    if (commsDebugLevel_ >= CommsDebugOnlyActions)
    {
        std::cout << "///////////////////////////////////////////////////" << std::endl;
        //std::cout << "(CommsDebug) : Send CommandMap --> " << std::endl << dataCommands << std::endl;
    }

    // Send the go until commands
    std::map<int,uint32_t> commands;
    std::map<int,uint32_t> data;
    int bitLength;

    for (const auto &element : dataCommands)
    {
        commands.insert(std::pair<int,uint32_t>(element.first,element.second.toCommand()));
        bitLength = element.second.dataBitLength;
        data.insert(std::pair<int,uint32_t>(element.first,element.second.data));
    }

    // Send the byte command
    SPIXfer(commands,8 /*bitLength*/);

    // Send the required data
    SPIXfer(data,bitLength);

    if (commsDebugLevel_ >= CommsDebugOnlyActions)
    {
        std::cout << "///////////////////////////////////////////////////" << std::endl;
    }
}

template void CommsDriver::sendCommands <class RunCommand>     (const std::map <int,RunCommand> &dataCommands);
template void CommsDriver::sendCommands <class GoToCommand>    (const std::map <int,GoToCommand> &dataCommands);
template void CommsDriver::sendCommands <class GoUntilCommand> (const std::map <int,GoUntilCommand> &dataCommands);
template void CommsDriver::sendCommands <class DataCommand>    (const std::map <int,DataCommand> &dataCommands);
template void CommsDriver::sendCommands <class GoToDirCommand> (const std::map <int,GoToDirCommand> &dataCommands);
template void CommsDriver::sendCommands <class MoveCommand>    (const std::map <int,MoveCommand> &dataCommands);

// Assumed that no data is needed to be transferred
void
CommsDriver::sendCommands(const std::vector<int> &motors , uint8_t commandByte)
{
    if (commsDebugLevel_ >= CommsDebugOnlyActions)
    {
        std::cout << "///////////////////////////////////////////////////" << std::endl;
        std::cout << "(CommsDebug) : Send Command [0x" <<std::hex << (unsigned int) commandByte << std::dec << "] to ";// << toLineString(&motors[0],motors.size()) << std::endl;
    }

    // Check validity
    for (int motor : motors)
    {
        checkMotorIsValid(motor);
    }

    // Send the commands
    std::map<int,uint32_t> commands;

    for (int motor : motors)
    {
        commands.insert(std::pair<int,uint32_t>(motor,commandByte));
    }

    SPIXfer(commands,8 /*bitLength*/);

    if (commsDebugLevel_ >= CommsDebugOnlyActions)
    {
        std::cout << "///////////////////////////////////////////////////" << std::endl;
    }
}

void
CommsDriver::sendCommand(const DataCommand &dataCommand , int motor)
{
    checkMotorIsValid(motor);
    std::map<int,uint32_t> commandMap;
    std::map<int,uint32_t> dataMap;

    if (commsDebugLevel_ >= CommsDebugOnlyActions)
    {
        std::cout << "///////////////////////////////////////////////////" << std::endl;
        std::cout << "(CommsDebug) : SendCommand [" << dataCommand.cmd << "] to motor " << motor << std::endl;
    }

    // Send the command bytes
    commandMap.insert(std::pair<int,uint32_t>(motor,dataCommand.toCommand()));
    SPIXfer(commandMap,8/*bitLength*/);

    // Send the associated data needed for the command
    dataMap.insert(std::pair<int,uint32_t>(motor,dataCommand.data));
    SPIXfer(dataMap,dataCommand.dataBitLength);

    if (commsDebugLevel_ >= CommsDebugOnlyActions)
    {
        std::cout << "///////////////////////////////////////////////////" << std::endl;
    }
}
