import { async, ComponentFixture, TestBed } from '@angular/core/testing';

import { MotorControlComponent } from './motor-control.component';

describe('MotorControlComponent', () => {
  let component: MotorControlComponent;
  let fixture: ComponentFixture<MotorControlComponent>;

  beforeEach(async(() => {
    TestBed.configureTestingModule({
      declarations: [ MotorControlComponent ]
    })
    .compileComponents();
  }));

  beforeEach(() => {
    fixture = TestBed.createComponent(MotorControlComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
