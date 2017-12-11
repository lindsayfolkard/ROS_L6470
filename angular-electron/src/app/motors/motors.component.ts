import { Component, OnInit } from '@angular/core';
import { Motor } from '../motor';
import { MOTORS } from '../fake-motors'

@Component({
  selector: 'app-motors',
  templateUrl: './motors.component.html',
  styleUrls: ['./motors.component.css']
})
export class MotorsComponent implements OnInit {

  motors = MOTORS;
  constructor() { }

  ngOnInit() {
  }

}
