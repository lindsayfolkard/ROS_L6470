import { Motor } from './motor';
import { StatusFlags } from './motor';

const FakeState:StatusFlags = {
  isHighZ: true,
  isBusy:  true,
  switchClosed: false,
  switchEvent: false,
  stepClock: false,

  // Warnings
  lastCommandSuccessful: true,
  lastCommandValid: true,
  hasThermalWarning: false,
  hasThermalShutdown: false,
  stalledPhaseA: false,
  stalledPhaseB: false,
  overCurrent: false};

  const FakeStateB:StatusFlags = {
    isHighZ: true,
    isBusy:  true,
    switchClosed: false,
    switchEvent: false,
    stepClock: false,

    // Warnings
    lastCommandSuccessful: true,
    lastCommandValid: true,
    hasThermalWarning: true,
    hasThermalShutdown: false,
    stalledPhaseA: false,
    stalledPhaseB: false,
    overCurrent: false};

export const MOTORS: Motor[] = [
  { id: 1, name: 'Joint 1',type:'NEMA17',position:0.1,speed:0.2,dirCW:true,isSwitchClosed:false,statusFlags:FakeState},
  { id: 2, name: 'Joint 2',type:'NEMA17',position:0.3,speed:0.4,dirCW:false,isSwitchClosed:true,statusFlags:FakeStateB}
];
