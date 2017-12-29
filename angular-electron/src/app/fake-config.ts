import {RosConfig} from './config'
import {OverallConfig} from './config'
import {AlarmStates} from './config'
import {CurrentThresholds} from './config'
import {StepModes} from './config'
import {SyncSelectModes} from './config'
import {OscillatorSelect} from './config'
import {SwitchAction} from './config'

export const FakeRosConfig : RosConfig  = {
  localCfgFile : "",
  interfaceType: "SPI",
  bus : 0,
  driverLogLevel: "Nothing",
  rosLogLevel: "INFO",
  driverConfigState:"Ok"
}

export const FakeAlarmStates : AlarmStates =
{
  overCurrent: true,
  thermalShutdown: true,
  themalWarning: true,
  underVoltage: true,
  stallDetectA: true,
  stallDetectB: true,
  switchTurnOn: true,
  badCommand : true,
}

export const FakeOverallConfig: OverallConfig =
{
  id:1,
  fullStepThresholdSpeed: 500,
  thermalDriftCoefficient: 0.1,
  controlMode: true, // true = voltagemode, false = currentmode
  currentThreshold: CurrentThresholds[0], // we have a bunch of string values for the different thresholds (easier to read and handle)
  stallThreshold: CurrentThresolds[0], // ditto
  overCurrentAction: true, // look for enum
  stepMode: StepModes[0],
  syncSelect: SyncSelectModes[0],
  syncEnabled: true,
  oscillator: OscillatorSelect[0],
  switchAction: SwitchAction[0],
  alarmStates: FakeAlarmStates,
}
