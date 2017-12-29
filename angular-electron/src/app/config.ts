export class RosConfig {
  localCfgFile : string;
  interfaceType: string;
  bus : number;
  driverLogLevel: string;
  rosLogLevel: string;
};

export const DriverTypes : string[] = ["Powerstep01","L6470","L6472","L6474","L6480"];
export const DriverDebugLevel : string[] = ["Everything","Only Actions","Nothing"];
export const DriverInterfaces : string[] = ["SPI","USB"];// TODO
