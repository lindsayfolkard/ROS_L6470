import { async, ComponentFixture, TestBed } from '@angular/core/testing';

import { RosConfigComponent } from './ros-config.component';

describe('RosConfigComponent', () => {
  let component: RosConfigComponent;
  let fixture: ComponentFixture<RosConfigComponent>;

  beforeEach(async(() => {
    TestBed.configureTestingModule({
      declarations: [ RosConfigComponent ]
    })
    .compileComponents();
  }));

  beforeEach(() => {
    fixture = TestBed.createComponent(RosConfigComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
