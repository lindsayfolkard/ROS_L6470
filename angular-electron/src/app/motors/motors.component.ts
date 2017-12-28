import { Component, OnInit } from '@angular/core';
import { Motor } from '../motor';
import { MotorService } from '../motor.service'

@Component({
  selector: 'app-motors',
  templateUrl: './motors.component.html',
  styleUrls: ['./motors.component.css']
})
export class MotorsComponent implements OnInit {

  selectedMotor: Motor;
  motors: Motor[];
  constructor(private motorService: MotorService) { }

  ngOnInit() {
    this.getMotors();
  }

  getMotors(): void {
    this.motorService.getMotors()
        .subscribe(motors => this.motors = motors);
  }

  onSelect(motor: Motor): void {
    this.selectedMotor = motor;
  }

}
