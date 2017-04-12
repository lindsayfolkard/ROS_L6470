#include "driver.h"

//commands.ino - Contains high-level command implementations- movement
//   and configuration commands, for example.

// Realize the "set parameter" function, to write to the various registers in
//  the dSPIN chip.
void AutoDriver::setParam(uint8_t param, unsigned long value)
{
  param |= SET_PARAM;
  SPIXfer((uint8_t)param);
  paramHandler(param, value);
}

// Realize the "get parameter" function, to read from the various registers in
//  the dSPIN chip.
long AutoDriver::getParam(ParamRegister param)
{
  SPIXfer(param | GET_PARAM);
  return paramHandler(param, 0);
}

// Returns the content of the ABS_POS register, which is a signed 22-bit number
//  indicating the number of steps the motor has traveled from the HOME
//  position. HOME is defined by zeroing this register, and it is zero on
//  startup.
long AutoDriver::getPos()
{
  long temp = getParam(ABS_POS);
  
  // Since ABS_POS is a 22-bit 2's comp value, we need to check bit 21 and, if
  //  it's set, set all the bits ABOVE 21 in order for the value to maintain
  //  its appropriate sign.
  if (temp & 0x00200000) temp |= 0xffc00000;
  return temp;
}

// Just like getPos(), but for MARK.
long AutoDriver::getMark()
{
  long temp = getParam(MARK);
  
  // Since ABS_POS is a 22-bit 2's comp value, we need to check bit 21 and, if
  //  it's set, set all the bits ABOVE 21 in order for the value to maintain
  //  its appropriate sign.
  if (temp & 0x00200000) temp |= 0xffC00000;
  return temp;
}

// RUN sets the motor spinning in a direction (defined by the constants
//  FWD and REV). Maximum speed and minimum speed are defined
//  by the MAX_SPEED and MIN_SPEED registers; exceeding the FS_SPD value
//  will switch the device into full-step mode.
// The spdCalc() function is provided to convert steps/s values into
//  appropriate integer values for this function.
void AutoDriver::run(MotorSpinDirection direction, float stepsPerSec)
{
  SPIXfer(RUN | dir);
  unsigned long integerSpeed = spdCalc(stepsPerSec);
  if (integerSpeed > 0xFFFFF) integerSpeed = 0xFFFFF;
  
  // Now we need to push this value out to the dSPIN. The 32-bit value is
  //  stored in memory in little-endian format, but the dSPIN expects a
  //  big-endian output, so we need to reverse the uint8_t-order of the
  //  data as we're sending it out. Note that only 3 of the 4 bytes are
  //  valid here.
  
  // We begin by pointing bytePointer at the first uint8_t in integerSpeed.
  uint8_t* bytePointer = (uint8_t*)&integerSpeed;
  // Next, we'll iterate through a for loop, indexing across the bytes in
  //  integerSpeed starting with uint8_t 2 and ending with uint8_t 0.
  for (int8_t i = 2; i >= 0; i--)
  {
    SPIXfer(bytePointer[i]);
  }
}

// STEP_CLOCK puts the device in external step clocking mode. When active,
//  pin 25, STCK, becomes the step clock for the device, and steps it in
//  the direction (set by the FWD and REV constants) imposed by the call
//  of this function. Motion commands (RUN, MOVE, etc) will cause the device
//  to exit step clocking mode.
void AutoDriver::stepClock(MotorSpinDirection direction)
{
  SPIXfer(STEP_CLOCK | direction);
}

// MOVE will send the motor numStep full steps in the
//  direction imposed by dir (FWD or REV constants may be used). The motor
//  will accelerate according the acceleration and deceleration curves, and
//  will run at MAX_SPEED. Stepping mode will adhere to FS_SPD value, as well.
void AutoDriver::move(MotorSpinDirection direction, unsigned long numSteps)
{
  SPIXfer(MOVE | direction);
  if (numSteps > 0x3FFFFF) numSteps = 0x3FFFFF;
  // See run() for an explanation of what's going on here.
  uint8_t* bytePointer = (uint8_t*)&numSteps;
  for (int8_t i = 2; i >= 0; i--)
  {
    SPIXfer(bytePointer[i]);
  }
}

// GOTO operates much like MOVE, except it produces absolute motion instead
//  of relative motion. The motor will be moved to the indicated position
//  in the shortest possible fashion.
void AutoDriver::goTo(long pos)
{
  SPIXfer(GOTO);
  if (pos > 0x3FFFFF) pos = 0x3FFFFF;
  // See run() for an explanation of what's going on here.
  uint8_t* bytePointer = (uint8_t*)&pos;
  for (int8_t i = 2; i >= 0; i--)
  {
    SPIXfer(bytePointer[i]);
  }
}

// Same as GOTO, but with user constrained rotational direction.
void AutoDriver::goToDir(MotorSpinDirection direction, long pos)
{
  SPIXfer(GOTO_DIR | direction);
  if (pos > 0x3FFFFF) pos = 0x3FFFFF;
  // See run() for an explanation of what's going on here.
  uint8_t* bytePointer = (uint8_t*)&pos;
  for (int8_t i = 2; i >= 0; i--)
  {
    SPIXfer(bytePointer[i]);
  }
}

// GoUntil will set the motor running with direction dir (REV or
//  FWD) until a falling edge is detected on the SW pin. Depending
//  on bit SW_MODE in CONFIG, either a hard stop or a soft stop is
//  performed at the falling edge, and depending on the value of
//  act (either RESET or COPY) the value in the ABS_POS register is
//  either RESET to 0 or COPY-ed into the MARK register.
void AutoDriver::goUntil(uint8_t action, MotorSpinDirection direction, float stepsPerSec)
{
  SPIXfer(GO_UNTIL | action | direction);
  unsigned long integerSpeed = spdCalc(stepsPerSec);
  if (integerSpeed > 0x3FFFFF) integerSpeed = 0x3FFFFF;
  // See run() for an explanation of what's going on here.
  uint8_t* bytePointer = (uint8_t*)&integerSpeed;
  for (int8_t i = 2; i >= 0; i--)
  {
    SPIXfer(bytePointer[i]);
  }
}

// Similar in nature to GoUntil, ReleaseSW produces motion at the
//  higher of two speeds: the value in MIN_SPEED or 5 steps/s.
//  The motor continues to run at this speed until a rising edge
//  is detected on the switch input, then a hard stop is performed
//  and the ABS_POS register is either COPY-ed into MARK or RESET to
//  0, depending on whether RESET or COPY was passed to the function
//  for act.
void AutoDriver::releaseSw(MotorSpinDirection direction, Action action)
{
  SPIXfer(RELEASE_SW | action | direction);
}

// GoHome is equivalent to GoTo(0), but requires less time to send.
//  Note that no direction is provided; motion occurs through shortest
//  path. If a direction is required, use GoTo_DIR().
void AutoDriver::goHome()
{
  SPIXfer(GO_HOME);
}

// GoMark is equivalent to GoTo(MARK), but requires less time to send.
//  Note that no direction is provided; motion occurs through shortest
//  path. If a direction is required, use GoTo_DIR().
void AutoDriver::goMark()
{
  SPIXfer(GO_MARK);
}

// setMark() and setHome() allow the user to define new MARK or
//  ABS_POS values.
void AutoDriver::setMark(long newMark)
{
  setParam(MARK, newMark);
}

void AutoDriver::setPos(long newPos)
{
  setParam(ABS_POS, newPos);
}

// Sets the ABS_POS register to 0, effectively declaring the current
//  position to be "HOME".
void AutoDriver::resetPos()
{
  SPIXfer(RESET_POS);
}

// Reset device to power up conditions. Equivalent to toggling the STBY
//  pin or cycling power.
void AutoDriver::resetDev()
{
  SPIXfer(RESET_DEVICE);
}
  
// Bring the motor to a halt using the deceleration curve.
void AutoDriver::softStop()
{
  SPIXfer(SOFT_STOP);
}

// Stop the motor with infinite deceleration.
void AutoDriver::hardStop()
{
  SPIXfer(HARD_STOP);
}

// Decelerate the motor and put the bridges in Hi-Z state.
void AutoDriver::softHiZ()
{
  SPIXfer(SOFT_HIZ);
}

// Put the bridges in Hi-Z state immediately with no deceleration.
void AutoDriver::hardHiZ()
{
  SPIXfer(HARD_HIZ);
}

// Fetch and return the 16-bit value in the STATUS register. Resets
//  any warning flags and exits any error states. Using GetParam()
//  to read STATUS does not clear these values.
int AutoDriver::getStatus()
{
  int temp = 0;
  uint8_t* bytePointer = (uint8_t*)&temp;
  SPIXfer(GET_STATUS);
  bytePointer[1] = SPIXfer(0);
  bytePointer[0] = SPIXfer(0);
  return temp;
}

// Much of the functionality between "get parameter" and "set parameter" is
//  very similar, so we deal with that by putting all of it in one function
//  here to save memory space and simplify the program.
long AutoDriver::paramHandler(uint8_t param, unsigned long value)
{
  long retVal = 0;   // This is a temp for the value to return.

  // This switch structure handles the appropriate action for each register.
  //  This is necessary since not all registers are of the same length, either
  //  bit-wise or uint8_t-wise, so we want to make sure we mask out any spurious
  //  bits and do the right number of transfers. That is handled by the xferParam()
  //  function, in most cases, but for 1-uint8_t or smaller transfers, we call
  //  SPIXfer() directly.
  switch (param)
  {
    // ABS_POS is the current absolute offset from home. It is a 22 bit number expressed
    //  in two's complement. At power up, this value is 0. It cannot be written when
    //  the motor is running, but at any other time, it can be updated to change the
    //  interpreted position of the motor.
    case ABS_POS:
      retVal = xferParam(value, 22);
      break;
    // EL_POS is the current electrical position in the step generation cycle. It can
    //  be set when the motor is not in motion. Value is 0 on power up.
    case EL_POS:
      retVal = xferParam(value, 9);
      break;
    // MARK is a second position other than 0 that the motor can be told to go to. As
    //  with ABS_POS, it is 22-bit two's complement. Value is 0 on power up.
    case MARK:
      retVal = xferParam(value, 22);
      break;
    // SPEED contains information about the current speed. It is read-only. It does
    //  NOT provide direction information.
    case SPEED:
      retVal = xferParam(0, 20);
      break;
    // ACC and DEC set the acceleration and deceleration rates. Set ACC to 0xFFF
    //  to get infinite acceleration/decelaeration- there is no way to get infinite
    //  deceleration w/o infinite acceleration (except the HARD STOP command).
    //  Cannot be written while motor is running. Both default to 0x08A on power up.
    // AccCalc() and DecCalc() functions exist to convert steps/s/s values into
    //  12-bit values for these two registers.
    case ACC:
      retVal = xferParam(value, 12);
      break;
    case DECEL:
      retVal = xferParam(value, 12);
      break;
    // MAX_SPEED is just what it says- any command which attempts to set the speed
    //  of the motor above this value will simply cause the motor to turn at this
    //  speed. Value is 0x041 on power up.
    // MaxSpdCalc() function exists to convert steps/s value into a 10-bit value
    //  for this register.
    case MAX_SPEED:
      retVal = xferParam(value, 10);
      break;
    // MIN_SPEED controls two things- the activation of the low-speed optimization
    //  feature and the lowest speed the motor will be allowed to operate at. LSPD_OPT
    //  is the 13th bit, and when it is set, the minimum allowed speed is automatically
    //  set to zero. This value is 0 on startup.
    // MinSpdCalc() function exists to convert steps/s value into a 12-bit value for this
    //  register. SetLSPDOpt() function exists to enable/disable the optimization feature.
    case MIN_SPEED:
      retVal = xferParam(value, 13);
      break;
    // FS_SPD register contains a threshold value above which microstepping is disabled
    //  and the dSPIN operates in full-step mode. Defaults to 0x027 on power up.
    // FSCalc() function exists to convert steps/s value into 10-bit integer for this
    //  register.
    case FS_SPD:
      retVal = xferParam(value, 10);
      break;
    // KVAL is the maximum voltage of the PWM outputs. These 8-bit values are ratiometric
    //  representations: 255 for full output voltage, 128 for half, etc. Default is 0x29.
    // The implications of different KVAL settings is too complex to dig into here, but
    //  it will usually work to max the value for RUN, ACC, and DEC. Maxing the value for
    //  HOLD may result in excessive power dissipation when the motor is not running.
    case KVAL_HOLD:
      retVal = xferParam(value, 8);
      break;
    case KVAL_RUN:
      retVal = xferParam(value, 8);
      break;
    case KVAL_ACC:
      retVal = xferParam(value, 8);
      break;
    case KVAL_DEC:
      retVal = xferParam(value, 8);
      break;
    // INT_SPD, ST_SLP, FN_SLP_ACC and FN_SLP_DEC are all related to the back EMF
    //  compensation functionality. Please see the datasheet for details of this
    //  function- it is too complex to discuss here. Default values seem to work
    //  well enough.
    case INT_SPD:
      retVal = xferParam(value, 14);
      break;
    case ST_SLP:
      retVal = xferParam(value, 8);
      break;
    case FN_SLP_ACC:
      retVal = xferParam(value, 8);
      break;
    case FN_SLP_DEC:
      retVal = xferParam(value, 8);
      break;
    // K_THERM is motor winding thermal drift compensation. Please see the datasheet
    //  for full details on operation- the default value should be okay for most users.
    case K_THERM:
      value &= 0x0F;
      retVal = xferParam(value, 8);
      break;
    // ADC_OUT is a read-only register containing the result of the ADC measurements.
    //  This is less useful than it sounds; see the datasheet for more information.
    case ADC_OUT:
      retVal = xferParam(value, 8);
      break;
    // Set the overcurrent threshold. Ranges from 375mA to 6A in steps of 375mA.
    //  A set of defined constants is provided for the user's convenience. Default
    //  value is 3.375A- 0x08. This is a 4-bit value.
    case OCD_TH:
      value &= 0x0F;
      retVal = xferParam(value, 8);
      break;
    // Stall current threshold. Defaults to 0x40, or 2.03A. Value is from 31.25mA to
    //  4A in 31.25mA steps. This is a 7-bit value.
    case STALL_TH:
      value &= 0x7F;
      retVal = xferParam(value, 8);
      break;
    // STEP_MODE controls the microstepping settings, as well as the generation of an
    //  output signal from the dSPIN. Bits 2:0 control the number of microsteps per
    //  step the part will generate. Bit 7 controls whether the BUSY/SYNC pin outputs
    //  a BUSY signal or a step synchronization signal. Bits 6:4 control the frequency
    //  of the output signal relative to the full-step frequency; see datasheet for
    //  that relationship as it is too complex to reproduce here.
    // Most likely, only the microsteps per step value will be needed; there is a set
    //  of constants provided for ease of use of these values.
    case STEP_MODE:
      retVal = xferParam(value, 8);
      break;
    // ALARM_EN controls which alarms will cause the FLAG pin to fall. A set of constants
    //  is provided to make this easy to interpret. By default, ALL alarms will trigger the
    //  FLAG pin.
    case ALARM_EN:
      retVal = xferParam(value, 8);
      break;
    // CONFIG contains some assorted configuration bits and fields. A fairly comprehensive
    //  set of reasonably self-explanatory constants is provided, but users should refer
    //  to the datasheet before modifying the contents of this register to be certain they
    //  understand the implications of their modifications. Value on boot is 0x2E88; this
    //  can be a useful way to verify proper start up and operation of the dSPIN chip.
    case CONFIG:
      retVal = xferParam(value, 16);
      break;
    // STATUS contains read-only information about the current condition of the chip. A
    //  comprehensive set of constants for masking and testing this register is provided, but
    //  users should refer to the datasheet to ensure that they fully understand each one of
    //  the bits in the register.
    case STATUS:  // STATUS is a read-only register
      retVal = xferParam(0, 16);;
      break;
    default:
      SPIXfer((uint8_t)value);
      break;
  }
  return retVal;
}

// Generalization of the subsections of the register read/write functionality.
//  We want the end user to just write the value without worrying about length,
//  so we pass a bit length parameter from the calling function.
long
AutoDriver::xferParam(unsigned long value, uint8_t bitLen)
{
  uint8_t byteLen = bitLen/8;      // How many BYTES do we have?
  if (bitLen%8 > 0) byteLen++;  // Make sure not to lose any partial uint8_t values.

  uint8_t temp;

  unsigned long retVal = 0;

  for (int i = 0; i < byteLen; i++)
  {
    retVal = retVal << 8;
    temp = SPIXfer((uint8_t)(value>>((byteLen-i-1)*8)));
    retVal |= temp;
  }

  unsigned long mask = 0xffffffff >> (32-bitLen);
  return retVal & mask;
}

uint8_t
AutoDriver::SPIXfer(uint8_t data)
{
    //  uint8_t dataPacket[_numBoards];
    //  int i;
    //  for (i=0; i < _numBoards; i++)
    //  {
    //    dataPacket[i] = 0;
    //  }
    //  dataPacket[_position] = data;
  //_SPI->transfer(dataPacket, _numBoards);
  //_SPI->endTransaction();
  //digitalWrite(_CSPin, HIGH);
  //return dataPacket[_position];
    return _SPI->writeByte(data);
}s
