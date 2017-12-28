import { Injectable } from '@angular/core';
import { Observable } from 'rxjs/Observable'
import { of } from 'rxjs/observable/of'

import { Motor } from './motor';
import { Profile } from './motor';
import { MOTORS } from './fake-motors'
import { MessageService } from './message.service'

@Injectable()
export class MotorService {

  constructor(private messageService : MessageService) { }

  getMotors(): Observable<Motor[]> {
    this.messageService.add('MotorService : fetched motors 1');
    this.messageService.add('MotorService : fetched motors 2');
    this.messageService.add('MotorService : fetched motors 3 ');
    this.messageService.add('MotorService : fetched motors 4 ');
    this.messageService.add('MotorService : fetched motors 5 ');
    this.messageService.add('MotorService : fetched motors 6 ');
    this.messageService.add('MotorService : fetched motors 7 ');
    this.messageService.add('MotorService : fetched motors 8 ');
    this.messageService.add('MotorService : fetched motors 9 ');
    this.messageService.add('MotorService : fetched motors 10');
    this.messageService.add('MotorService : fetched motors 11');
    this.messageService.add('MotorService : fetched motors 12');
    return of(MOTORS);
  }

  /////////////////////////
  // Operational Commands
  /////////////////////////

  /// Stop Commands ///
  hardStop(motorIds : number[]) : void {

  }

  softStop(motorIds : number[]) : void {

  }

  hardHiZ(motorIds : number[]) : void {

  }

  softHiZ(motorIds : number[]) : void {

  }

  setProfile(motorId : number , profile : Profile)
  {

  }

  goToPosition(motorIds : number[] , positions : number[])
  {

  }

  goToPositionDir(motorIds : number[] , positions : number [] , dirCW : boolean[])
  {

  }

  runAtSpeed(motorIds : number[] , speed : number[] , dirCW : boolean[])
  {

  }

  goUntil(motorIds : number[] , speed : number[] , dirCW : boolean[] , actions : boolean[])
  {

  }
  
}
