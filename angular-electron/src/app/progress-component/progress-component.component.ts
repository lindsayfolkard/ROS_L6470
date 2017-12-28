import { Component, OnInit, Input } from '@angular/core';
import { Observable } from 'rxjs/Observable';
import 'rxjs/add/observable/interval';
import 'rxjs/add/operator/map';
import 'rxjs/add/operator/takeWhile';
import 'rxjs/add/operator/do';

@Component({
  selector: 'app-progress-component',
  templateUrl: './progress-component.component.html',
  styleUrls: ['./progress-component.component.css']
})
export class ProgressComponentComponent implements OnInit {

  max     = 1;
  @Input() current: number;
  @Input() label: string;

  constructor() { }

  ngOnInit() {
  }

  start() {
    const interval = Observable.interval(100);

    interval
      .takeWhile(_ => !this.isFinished )
      .do(i => this.current += 0.1)
      .subscribe();
  }

   /// finish timer
  finish() {
    this.current = this.max;
  }

  /// reset timer
  reset() {
    this.current = 0;
  }


  /// Getters to prevent NaN errors

  get maxVal() {
    return isNaN(this.max) || this.max < 0.1 ? 0.1 : this.max;
  }

  get currentVal() {
    return isNaN(this.current) || this.current < 0 ? 0 : this.current;
  }

  get isFinished() {
    return this.currentVal >= this.maxVal;
  }


}
