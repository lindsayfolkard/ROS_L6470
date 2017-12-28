import { BrowserModule } from '@angular/platform-browser';
import { NgModule } from '@angular/core';

import { AppComponent } from './app.component';

import { FormsModule } from '@angular/forms'; // <-- here
import { RoundProgressModule } from 'angular-svg-round-progressbar';
import { MotorsComponent } from './motors/motors.component';
import { HeroDetailComponent } from './hero-detail/hero-detail.component';
import { MotorDetailComponent } from './motor-detail/motor-detail.component';
import { MessagesComponent } from './messages/messages.component'; // <-- here

import { MotorService } from './motor.service'
import { MessageService } from './message.service';
import { ProgressComponentComponent } from './progress-component/progress-component.component';
import { SystemOverviewComponent } from './system-overview/system-overview.component'

@NgModule({
  declarations: [
    AppComponent,
    MotorsComponent,
    HeroDetailComponent,
    MotorDetailComponent,
    MessagesComponent,
    ProgressComponentComponent,
    SystemOverviewComponent
  ],
  imports: [
    BrowserModule,
    FormsModule, // <-- here
    RoundProgressModule // <-- and here
  ],
  providers: [MotorService, MessageService],
  bootstrap: [AppComponent]
})
export class AppModule { }
