import { async, ComponentFixture, TestBed } from '@angular/core/testing';

import { MotorDetailComponent } from './motor-detail.component';

describe('MotorDetailComponent', () => {
  let component: MotorDetailComponent;
  let fixture: ComponentFixture<MotorDetailComponent>;

  beforeEach(async(() => {
    TestBed.configureTestingModule({
      declarations: [ MotorDetailComponent ]
    })
    .compileComponents();
  }));

  beforeEach(() => {
    fixture = TestBed.createComponent(MotorDetailComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
