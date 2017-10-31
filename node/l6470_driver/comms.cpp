#include "multidriver.h"
#include <iostream>
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
	
    if (commsDebugLevel_ >= CommsDebugOnlyActions) 
    {
	std::cout << "||||||||||||||||||||||||||||||||||||||||||||||||||||||" << std::endl;
        std::cout << "(CommsDebug) : Set Param [" << param << "] with map : " 
                  << std::endl << toMapString(values,toBitLength(param)) << std::endl;
    } 

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

    if (commsDebugLevel_ >= CommsDebugOnlyActions) 
    {
	std::cout << "||||||||||||||||||||||||||||||||||||||||||||||||||||||" << std::endl << std::endl;
    } 
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

    if (commsDebugLevel_ >= CommsDebugOnlyActions) 
    {
	std::cout << "___________________________________________" << std::endl;
        std::cout << "(CommsDebug) : Get Param [" << param << "] " << std::endl << std::endl;
    } 

    std::vector<uint32_t> values = getParam(param);
    
    if (commsDebugLevel_ >= CommsDebugOnlyActions) 
    {
	std::cout << "(CommsDebug) : Got Param [" << param << "] --> " << values[motor] << "(0x" << std::hex << param << std::dec << ")" << std::endl;
        std::cout << "___________________________________________" << std::endl << std::endl;
    } 

    return values[motor];
}

std::vector<uint32_t>
MultiDriver::SPIXfer(const std::map<int, uint32_t> &data , int bitLength)
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
    std::vector<uint32_t> recvData(motors_.size(),0); // should init to all zeroes...

    // Determine the number of bytes to send
    uint8_t byteLength = bitLength/8;   // How many BYTES do we have?
    if (bitLength%8 > 0) byteLength++;  // Make sure not to lose any partial uint8_t values.

    // Send the data to the boards one byte at a time
    // The L6470 expects numbers in big-endian format (because it is fucking annoying like that), however this
    // is running on a linux which is little-endian. Hence we need to reverse the order in which we send bytes
    for (int i=0; i < byteLength ; ++i)
    {
        std::vector<uint8_t> byteSendPacket(motors_.size(),NOP);//={NOP};
        std::vector<uint8_t> byteRecvPacket(motors_.size(),NOP);//={NOP};
        // TODO - double check that this is ok (should be)

        for (const auto element : data)
        {
            byteSendPacket[element.first] = (element.second >> ((byteLength-i-1)*8));// & 0xFF;
        }
        
        // Debug if needed
	if (commsDebugLevel_ == CommsDebugEverything) 
    	{
        std::cout << "(CommsDebug) : SPI Transfer Packet " << i+1 <<" of " << (int)byteLength << " --> " << toLineString(byteSendPacket) << std::endl;
    	}

        SPI_->transfer(&byteSendPacket[0],&byteRecvPacket[0],motors_.size());

        // Debug if needed
	if (commsDebugLevel_ == CommsDebugEverything) 
    	{
        std::cout << "(CommsDebug) : SPI Recieve Packet " << i+1 <<" of " << (int)byteLength << " --> " << toLineString(byteRecvPacket) << std::endl;
    	}

        // I presume the L6470 also responds in big-endian format
        for (unsigned int j=0; j < motors_.size();++j)
        {
            recvData[j] = recvData[j] << 8;
            recvData[j] |= byteRecvPacket[j];
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

    // Trim the response
    uint32_t mask = 0xffffffff >> (32-bitLength);
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

    if (commsDebugLevel_ >= CommsDebugOnlyActions) 
    {
	//std::cout << "(CommsDebug) : Send CommandMap --> " <<  std::endl;
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
}

// Assumed that no data is needed to be transferred
void
MultiDriver::sendCommands(const std::vector<int> &motors , uint8_t commandByte)
{    
    if (commsDebugLevel_ >= CommsDebugOnlyActions) 
    {
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
}

void
MultiDriver::sendCommand(const DataCommand &dataCommand , int motor)
{
    checkMotorIsValid(motor);
    std::map<int,uint32_t> commandMap;
    std::map<int,uint32_t> dataMap;

    if (commsDebugLevel_ >= CommsDebugOnlyActions) 
    {
	std::cout << "(CommsDebug) : SendCommand [" << dataCommand.cmd << "] to motor " << motor << std::endl;
    } 

    // Send the command bytes
    commandMap.insert(std::pair<int,uint32_t>(motor,dataCommand.toCommand()));
    SPIXfer(commandMap,8/*bitLength*/);

    // Send the associated data needed for the command
    dataMap.insert(std::pair<int,uint32_t>(motor,dataCommand.data));
    SPIXfer(dataMap,dataCommand.dataBitLength);
}
