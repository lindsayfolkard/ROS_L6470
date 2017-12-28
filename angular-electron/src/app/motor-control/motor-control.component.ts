import { Component, OnInit } from '@angular/core';
import { Motor } from '../motor'
import { Profile } from '../motor'
import { FormsModule } from '@angular/forms'
import { MotorService } from '../motor.service'

@Component({
  selector: 'app-motor-control',
  templateUrl: './motor-control.component.html',
  styleUrls: ['./motor-control.component.css']
})
export class MotorControlComponent implements OnInit {

  motors: Motor[];

  constructor(private motorService: MotorService) { }

  ngOnInit() {
    this.getMotors();
  }

  getMotors(): void {
    this.motorService.getMotors()
        .subscribe(motors => this.motors = motors);
  }

  hardStop(motor : Motor) : void {
    console.log("Hard Stop Motor" + motor.id)
  }

  softStop(motor : Motor) : void {

  }

  hardHiZ(motor : Motor) : void {

  }

  softHiZ(motor : Motor) : void {

  }

  setProfile(motor : Motor , profile : Profile)
  {

  }

  goToPosition(motor : Motor , positions : number)
  {

  }

  goToPositionDir(motor : Motor , positions : number , dirCW : boolean)
  {

  }

  runAtSpeed(motor : Motor , speed : number, dirCW : boolean)
  {

  }

  goUntil(motor : Motor , speed : number, dirCW : boolean , actions : boolean)
  {

  }

}
