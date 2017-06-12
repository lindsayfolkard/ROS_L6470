#include "multidriver.h"
#include "types.h"
#include <map>
#include <vector>

///
/// \brief Functions for handling comms with daisy chained L6470H boards
///

void
MultiDriver::setParam(ParamRegister param, std::map <int,uint32_t> &values)
{
    checkMapIsValid<uint32_t>(values);

    uint8_t sendByte=SET_PARAM;
    sendByte |= param;

    // Send the param request
    std::map<int,uint32_t> commandSet;
    for (const auto element : values)
    {
        commandSet.insert(std::pair<int,uint32_t>(element.first,sendByte));
    }

    SPIXfer(commandSet,8/*bitLength*/);

    // Set the value of each parameter as needed
    xferParam(values,toBitLength(param));
}
void
MultiDriver::setParam(ParamRegister param , uint32_t value , int motor)
{
    checkMotorIsValid(motor);
    std::map<int,uint32_t> map = {{motor,value}};
    setParam(param,map);
}

// Realize the "get parameter" function, to read from the various registers in
//  the dSPIN chip.
std::vector<uint32_t>
MultiDriver::getParam(ParamRegister param)
{
    uint8_t sendByte=GET_PARAM;
    sendByte |= param;

    // Send the param request
    SPIXfer(sendByte);

    // Parse the response from multiple boards
    const std::map<int,uint32_t> emptyMap;
    return xferParam(emptyMap,toBitLength(param));
}

uint32_t
MultiDriver::getParam(ParamRegister param , int motor)
{
    checkMotorIsValid(motor);
    std::vector<uint32_t> values = getParam(param);
    return values[motor];
}

std::vector<uint32_t>
MultiDriver::SPIXfer(const std::map<int, uint32_t> &data , int bitLength)
{
    // Check map validity
    checkMapIsValid<uint32_t>(data);

    // The L6470 Accepts data daisy chained in 1 byte blocks
    // (i.e controller1byte1,controller2byte1,...,<BREAK>,controller1byte2,controller2byte2,... etc..)
    // This is kinda annoying, but fuck it also is useful in getting somewhat exact timing with control

    // NB: need to be careful here that we are dealing with contiguos memory ??
    std::vector<uint32_t> recvData(motors_.size(),0); // should init to all zeroes...

    // Determine the number of bytes to send
    uint8_t byteLength = bitLength/8;   // How many BYTES do we have?
    if (bitLength%8 > 0) byteLength++;  // Make sure not to lose any partial uint8_t values.

    for (int i=0; i < byteLength ; ++i)
    {
        uint8_t byteSendPacket [motors_.size()]={NOP};
        uint8_t byteRecvPacket [motors_.size()]={NOP};

        for (const auto element : data)
        {
            byteSendPacket[element.first] = (element.second << (i*8)) & 0xFF;
        }

        SPI_->transfer(byteSendPacket,byteRecvPacket,motors_.size());

        for (unsigned int j=0; j < motors_.size();++j)
        {
            recvData[j] |= (byteRecvPacket[j] >> (i*8));
        }
    }

    //std::cout << "Transfer byte " << (int) sendPacket[_position] << std::endl;
    //std::cout << "Result from transfer is " << (int) result << std::endl;
    //std::cout << "return value is " << (int) recvPacket[_position] << std::endl;
    return recvData;
}

// Will send request for all motors
std::vector<uint32_t>
MultiDriver::SPIXfer(uint8_t requestValue)
{
    // Create an appropriate map
    std::map<int,uint32_t> requests;
    for (unsigned int i=0 ; i < motors_.size() ; ++i)
    {
        requests.insert(std::pair<int,uint32_t>(i,requestValue));
    }

    // Send the request
    return SPIXfer(requests,8 /*bitLength*/);
}

uint32_t
MultiDriver::SPIXfer(uint8_t requestValue, int motor)
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
MultiDriver::xferParam(const std::map<int,uint32_t> &parameters, uint8_t bitLength)
{
    checkMapIsValid<uint32_t>(parameters);

    uint8_t byteLength = bitLength/8;   // How many BYTES do we have?
    if (bitLength%8 > 0) byteLength++;  // Make sure not to lose any partial uint8_t values.

    // Send and recieve the response
    std::vector<uint32_t> response = SPIXfer(parameters,bitLength);

    // Trim the response ???
    /// TODO - ????
    unsigned long mask = 0xffffffff >> (32-bitLength);
    for (auto element : response)
    {
        element = element & mask;
    }

    return response;
}

void
MultiDriver::sendCommands(const std::map <int,DataCommand> &dataCommands)
{
    checkMapIsValid<DataCommand>(dataCommands);

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
}

// Assumed that no data is needed to be transferred
void
MultiDriver::sendCommands(const std::vector<int> &motors , uint8_t commandByte)
{    
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
}

void
MultiDriver::sendCommand(const DataCommand &dataCommand , int motor)
{
    checkMotorIsValid(motor);
    std::map<int,uint32_t> commandMap;
    std::map<int,uint32_t> dataMap;

    // Send the command bytes
    commandMap.insert(std::pair<int,uint32_t>(motor,dataCommand.toCommand()));
    SPIXfer(commandMap,8/*bitLength*/);

    // Send the associated data needed for the command
    dataMap.insert(std::pair<int,uint32_t>(motor,dataCommand.data));
    SPIXfer(dataMap,dataCommand.dataBitLength);
}
