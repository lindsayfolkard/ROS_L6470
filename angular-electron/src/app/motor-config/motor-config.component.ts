import { Component, OnInit } from '@angular/core';
import {OverallConfig} from '../config'
import {FakeOverallConfig} from '../fake-config'
import {CurrentThresholds} from '../config'
import {StepModes} from '../config'
import {SyncSelectModes} from '../config'
import {OscillatorSelect} from '../config'
import {SwitchAction} from '../config'

@Component({
  selector: 'app-motor-config',
  templateUrl: './motor-config.component.html',
  styleUrls: ['./motor-config.component.css']
})
export class MotorConfigComponent implements OnInit {

  motorConfigs : OverallConfig[] = [FakeOverallConfig];

  constructor() { }

  ngOnInit() {
  }

}
