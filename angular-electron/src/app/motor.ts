export class StatusFlags
{
  // General State
  isHighZ: boolean;
  isBusy:  boolean;
  switchClosed: boolean;
  switchEvent: boolean;
  stepClock: boolean;

  // Warnings
  lastCommandSuccessful: boolean;
  lastCommandValid: boolean;
  hasThermalWarning: boolean;
  hasThermalShutdown: boolean;
  stalledPhaseA: boolean;
  stalledPhaseB: boolean;
  overCurrent: boolean;

  // isOk(): boolean {
  //   return this.lastCommandSuccessful && this.lastCommandValid && !this.hasThermalWarning && !this.hasThermalShutdown && !this.stalledPhaseA
  //          && !this.stalledPhaseB && !this.overCurrent;
  // }
}

export function areStatusFlagsOk(statusFlags: StatusFlags) : boolean {
  return statusFlags.lastCommandSuccessful && statusFlags.lastCommandValid && !statusFlags.hasThermalWarning && !statusFlags.hasThermalShutdown && !statusFlags.stalledPhaseA
         && !statusFlags.stalledPhaseB && !statusFlags.overCurrent;
}

export class Motor {
  id: number;
  name: string;
  type: string; // maybe create explicit motor typescript

  // Current Pose Information
  position: number;
  speed: number;
  dirCW: boolean;
  isSwitchClosed: boolean;

  // General STATUS
  statusFlags: StatusFlags;
}

export class RosConfig
{
  file: string;
  interface: boolean;
  bus: number;
  driverDebugLevel: number;
  rosMsgLevel: number;
}
