#include "multidriver.h"
#include <map>
#include <vector>
#include "types.h"

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
    SPIXfer(sendByte,8);

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
    SPIXfer(sendByte,8 /*bitLength*/);

    // Parse the response from multiple boards
    std::map<int,uint32_t> emptyMap();
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
    // (i.e controller1byte1,controller2byte1,...,<BREAK>,controller1byte2,controller2byte2,... etc..)s

    // NB: need to be careful here that we are dealing with contiguos memory ??
    std::vector<uint32_t> recvData(motors_.size(),0); // should init to all zeroes...

    // Determine the number of bytes to send
    uint8_t byteLen = bitLength/8;   // How many BYTES do we have?
    if (bitLegth%8 > 0) byteLength++;  // Make sure not to lose any partial uint8_t values.

    for (int i=0; i < byteLen ; ++i)
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

std::vector<uint32_t>
MultiDriver::SPIXfer(uint8_t requestValue)
{
    // Create an appropriate map
    std::map<int,uint8_t> request;
    for (int i=0 ; i < motors_.size() ; ++i)
    {
        map.insert(std::pair<int,uint32_t>(i,requestValue));
    }

    // Send the request
    return SPIXfer(request,8 /*bitLength*/);
}

std::vector<uint32_t>
MultiDriver::xferParam(const std::map<int,uint32_t> &parameters, uint8_t bitLength)
{
    checkMapIsValid<uint32_t>(parameters);

    uint8_t byteLen = bitLen/8;   // How many BYTES do we have?
    if (bitLen%8 > 0) byteLen++;  // Make sure not to lose any partial uint8_t values.

    // Send and recieve the response
    std::vector<uint32_t> response = SPIXfer(parameters,bitLength);

    // Trim the response ???
    unsigned long mask = 0xffffffff >> (32-bitLen);
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
    std::map<int,uint8_t> commands;
    std::map<int,uint32_t> data;
    for (auto command : dataCommands)
    {
        commands.insert(std::pair<int,uint8>(element.first,command.toCommand()));
        int bitLength;
        data.insert(std::pair<int,uint32_t>(element.first,command.toData(bitLength)));
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
    std::map<int,uint8_t> commands;

    for (int motor : motors)
    {
        commands.insert(std::pair<int,uint8>(motor,commandByte));
    }

    SPIXfer(commands,8 /*bitLength*/);
}

void
MultiDriver::sendCommand(const DataCommand &dataCommand , int motor)
{
    checkMotorIsValid(motor);
    std::map<int,uint8_t>  commandMap;
    std::map<int,uint32_T> dataMap;

    // Send the command bytes
    commandMap.insert(std::pair<int,uint8_t>(motor,dataCommand.toCommand()));
    SPIXfer(commandMap,8/*bitLength*/);

    // Send the associated data needed for the command
    int bitLength;
    dataMap.insert(std::pair<int,uint32_t>(motor,dataCommand.toData(bitLength)));
    SPIXfer(dataMap,bitLength);
}
