import { Injectable } from '@angular/core';
import { Observable } from 'rxjs/Observable'
import { of } from 'rxjs/observable/of'

import { RosConfig } from './config'
import { FakeRosConfig } from './fake-config'

@Injectable()
export class ConfigService {

  constructor() { }

  getRosConfig(): Observable<RosConfig> {
    return of(FakeRosConfig);
  }

}
