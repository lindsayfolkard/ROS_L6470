export class RosConfig {
  localCfgFile : string;
  interfaceType: string;
  bus : number;
  driverLogLevel: string;
  rosLogLevel: string;
  driverConfigState: string;
};

export const DriverTypes : string[] = ["Powerstep01","L6470","L6472","L6474","L6480"];
export const DriverDebugLevel : string[] = ["Everything","Only Actions","Nothing"];
export const DriverInterfaces : string[] = ["SPI","USB"];// TODO

// Driver Config Representations

export class AlarmStates
{
  overCurrent: boolean;
  thermalShutdown: boolean;
  themalWarning: boolean;
  underVoltage: boolean;
  stallDetectA: boolean;
  stallDetectB: boolean;
  switchTurnOn: boolean;
  badCommand : boolean;
}

export const CurrentThresholds : string[] = ["375 ma","500 ma","750 ma"];
export const StepModes : string[] = ["Full","Half","Quarter","1/8","1/16","1/32","1/64","1/128"];
export const SyncSelectModes : string[] = ["1/2","1/4"];
export const OscillatorSelect : string[] = ["INT_16MHZ","EXT_16MHZ","EXT_32MHZ"];
export const SwitchAction : string[] = ["Hard Stop","Soft Stop","Hard HiZ","Soft HiZ"];

export class OverallConfig {
  id : number;
  fullStepThresholdSpeed: number;
  thermalDriftCoefficient: number;
  controlMode: boolean; // true = voltagemode, false = currentmode
  currentThreshold: string; // we have a bunch of string values for the different thresholds (easier to read and handle)
  stallThreshold: string; // ditto
  overCurrentAction: boolean; // look for enum
  stepMode: string;
  syncSelect: string;
  syncEnabled: boolean;
  oscillator: string;
  switchAction: string;
  alarmStates: AlarmStates;
}
