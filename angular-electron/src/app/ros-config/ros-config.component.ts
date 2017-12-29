import { Component, OnInit } from '@angular/core';
import { RosConfig } from '../config'
import { FakeRosConfig } from '../fake-config'
import { DriverTypes } from '../config'
import { DriverDebugLevel } from '../config'
import { DriverInterfaces } from '../config'
import { ConfigService } from '../config.service'

@Component({
  selector: 'app-ros-config',
  templateUrl: './ros-config.component.html',
  styleUrls: ['./ros-config.component.css']
})
export class RosConfigComponent implements OnInit {

  //rosConfig: RosConfig;
  rosConfig = FakeRosConfig;
  driverTypes = DriverTypes;
  driverDebugLevels = DriverDebugLevel;
  driverInterfaces = DriverInterfaces;

  // Configuration state

  //constructor(private configService: ConfigService) { }

  ngOnInit(){}
  // ngOnInit() {
  //   this.getRosConfig();
  // }
  //
  // getRosConfig(): void {
  //   this.configService.getRosConfig()
  //       .subscribe(rosConfig => this.rosConfig = rosConfig);
  // }

  reloadCfgFile(): void {

  }

  saveCfgFile(): void {

  }

  reConfigureDriver(): void {

  }

}
