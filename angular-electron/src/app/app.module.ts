import { BrowserModule } from '@angular/platform-browser';
import { NgModule } from '@angular/core';

import { AppComponent } from './app.component';

import { FormsModule } from '@angular/forms'; // <-- here
import { RoundProgressModule } from 'angular-svg-round-progressbar';
import { MotorsComponent } from './motors/motors.component'; // <-- here

@NgModule({
  declarations: [
    AppComponent,
    MotorsComponent
  ],
  imports: [
    BrowserModule, 
    FormsModule, // <-- here
    RoundProgressModule // <-- and here
  ],
  providers: [],
  bootstrap: [AppComponent]
})
export class AppModule { }
