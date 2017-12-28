import { Component, OnInit, Input } from '@angular/core';
import { Motor } from '../motor';

@Component({
  selector: 'app-motor-detail',
  templateUrl: './motor-detail.component.html',
  styleUrls: ['./motor-detail.component.css']
})
export class MotorDetailComponent implements OnInit {

  @Input() motor: Motor;

  constructor() {}

  ngOnInit() {
  }

}
