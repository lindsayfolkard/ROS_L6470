#include "multidriver.h"
#include <map>
#include <vector>
#include "types.h"

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
    for (int i=0; i < sizeof(S) ; ++i)
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

void
MultiDriver::sendCommands(const std::map <int,DataCommand> &dataCommands)
{
    // Send the go until commands
    std::map<int,uint8_t> commands;
    std::map<int,uint32_t> data;
    for (auto command : dataCommands)
    {
        commands.insert(std::pair<int,uint8>(element.first,command.toCommand()));
        int bitLength;
        data.insert(std::pair<int,uint32_t>(element.first,command.toData(bitLength)));
    }
    SPIXfer(commands);

    // Send the required data
    /// TODO - how do we propagate bitlength nicely..???
    SPIXfer(data);
}

void
MultiDriver::sendCommands(const std::vector<int> &motors , uint8_t commandByte)
{
    // Send the go until commands
    std::map<int,uint8_t> commands;

    for (int motor : motors)
    {
        commands.insert(std::pair<int,uint8>(motor,commandByte));
    }
    SPIXfer(commands);

}

void
MultiDriver::sendCommand(const DataCommand &dataCommand , int motor)
{
    checkMotorIsValid(motor);
    std::map<int,uint8_t>  commandMap;
    std::map<int,uint32_T> dataMap;

    commandMap.insert(std::pair<int,uint8_t>(motor,dataCommand.toCommand()));
    SPIXfer(commandMap);
    int bitLength;
    dataMap.insert(std::pair<int,uint32_t>(motor,dataCommand.toData(bitLength)));
    SPIXfer(dataMap);
}
