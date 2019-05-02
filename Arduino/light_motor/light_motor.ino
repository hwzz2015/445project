
#include <Wire.h>
#include <AccelStepper.h>

#define Addr1 0x46
#define Addr2 0x45

int address[4] = {0x44,0x45,0x46,0x47};

#define HALFSTEP 8

// Motor pin definitions
#define motorPin1  5     // IN1 on the ULN2003 driver 1
#define motorPin2  6    // IN2 on the ULN2003 driver 1
#define motorPin3  7     // IN3 on the ULN2003 driver 1
#define motorPin4  8
// IN4 on the ULN2003 driver 1

AccelStepper stepper1(HALFSTEP, motorPin1, motorPin3, motorPin2, motorPin4);

void setup() 
{
  Serial.begin(9600);                            // Initialize serial communication at 9600
  Wire.begin();                                  // Initialize Arduino in I2C master.
  Wire.beginTransmission(Addr1);                  // I2C address of OPT3001 = 0x44
  Wire.write(0x01);                              // Config register address 0x01
  Wire.write(0xCE);
  Wire.write(0x10);                              // Write 0xCE10 to turn on sensor
  Wire.endTransmission();

  Wire.beginTransmission(Addr2);                  // I2C address of OPT3001 = 0x45
  Wire.write(0x01);                              // Config register address 0x01
  Wire.write(0xCE);
  Wire.write(0x10);                              // Write 0xCE10 to turn on sensor
  Wire.endTransmission();

  stepper1.setMaxSpeed(1000.0);
  stepper1.setAcceleration(400.0);
  stepper1.setSpeed(600);
  stepper1.moveTo(0);
          
  stepper1.runToNewPosition(0);
  

  Serial.println("start");
}

float pos =0;
float stepsize = 2000; //negative for clock-wise
int steps = 8;

void loop() 
{ 
  float flux1, flux2;

  flux1 = get_light(Addr1);
  Wire.beginTransmission(Addr1);
  Wire.write(0x00);                              // Send result register address
  Wire.endTransmission();
  delay(10);
  
  Wire.requestFrom(Addr1, 2);                     // Request 2 bytes data from OPT3001
  uint16_t iData;
  uint8_t  iBuff[2];
  while (Wire.available())
  {
    Wire.readBytes(iBuff, 2);
    iData = (iBuff[0] << 8) | iBuff[1];
    //Serial.print(iData,BIN);                     // Print the received data
    flux1 = SensorOpt3001_convert(iData);   // Calculate LUX from sensor data
  }
  
  Wire.beginTransmission(Addr2);
  Wire.write(0x00);                              // Send result register address
  Wire.endTransmission();
  delay(10);
  
  Wire.requestFrom(Addr2, 2);                     // Request 2 bytes data from OPT3001
  while (Wire.available())
  {
  
    Wire.readBytes(iBuff, 2);
    iData = (iBuff[0] << 8) | iBuff[1];
    //Serial.print(iData,BIN);                     // Print the received data
    flux2 = SensorOpt3001_convert(iData);   // Calculate LUX from sensor data
  }
  Serial.print(flux1);
  Serial.print(" and ");
  Serial.println(flux2);

  if(flux2 > 100.0){
      stepper1.runToNewPosition(1000);
      Serial.println("run to 1000");
  }else{
      stepper1.runToNewPosition(-1000);
      Serial.println("run to -1000");
  }

  
  delay(1000);
}

float get_light(int Addr){
  Wire.beginTransmission(Addr);
  Wire.write(0x00);                              // Send result register address
  Wire.endTransmission();
  delay(10);
  
  Wire.requestFrom(Addr, 2);                     // Request 2 bytes data from OPT3001
  uint16_t iData;
  uint8_t  iBuff[2];
  float flux;
  while (Wire.available())
  {
    Wire.readBytes(iBuff, 2);
    iData = (iBuff[0] << 8) | iBuff[1];
    //Serial.print(iData,BIN);                     // Print the received data
    flux = SensorOpt3001_convert(iData);   // Calculate LUX from sensor data
  }
  return flux;
}
float SensorOpt3001_convert(uint16_t iRawData)
{
  uint16_t iExponent, iMantissa;
  iMantissa = iRawData & 0x0FFF;                 // Extract Mantissa
  iExponent = (iRawData & 0xF000) >> 12;         // Extract Exponent 
  return iMantissa * (0.01 * pow(2, iExponent)); // Calculate final LUX
}
