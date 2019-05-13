
#include <Wire.h>
#include <Stepper.h>

// Motor pin definitions
#define motorPin1  5     // IN1 on the ULN2003 driver 1
#define motorPin2  7    // IN2 on the ULN2003 driver 1
#define motorPin3  6     // IN3 on the ULN2003 driver 1
#define motorPin4  8     // IN4 on the ULN2003 driver 1

int address[4] = {0x44,0x45,0x46,0x47};
const int step_per_rev = 2048;
Stepper stepper(step_per_rev, motorPin1, motorPin3, motorPin2, motorPin4);

void setup() 
{
  Serial.begin(9600);                            // Initialize serial communication at 9600
  Wire.begin();                                  // Initialize Arduino in I2C master.
  for(int i=0;i<4;i++){
  Wire.beginTransmission(address[i]);                  // I2C address of OPT3001 = 0x44
  Wire.write(0x01);                              // Config register address 0x01
  Wire.write(0xCE);
  Wire.write(0x10);                              // Write 0xCE10 to turn on sensor
  Wire.endTransmission();
  }
  stepper.setSpeed(10);
  
  Serial.println("start");
}

float pos =0;
float stepsize = 2000; //negative for clock-wise
int steps = 8;

void loop() 
{ 
  
  float flux[4];

  for(int i=0;i<4;i++){
  flux[i] = get_light(address[i]);
  Serial.print("sensor ");

  Serial.println(flux[i]);
  }
  

//  if(flux[2] > 100.0){
//      stepper.step(step_per_rev);
//      Serial.println("clockwise");
//  }else{
//      stepper.step(-step_per_rev);
//      Serial.println("counter-clockwise");
//  }
 
 int mm = getIndexOfMaximumValue(flux,4);
 Serial.println(mm);
 stepper.step(step_per_rev);
 Serial.println("clockwise");
 delay(1000);
}

float get_light(int addr){
  Wire.beginTransmission(addr);
  Wire.write(0x00);                              // Send result register address
  Wire.endTransmission();
  delay(10);
  
  Wire.requestFrom(addr, 2);                     // Request 2 bytes data from OPT3001
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

int getIndexOfMaximumValue(float* array, int size){
 int maxIndex = 0;
 int max = array[maxIndex];
 for (int i=1; i<size; i++){
   if (max<array[i]){
     max = array[i];
     maxIndex = i;
   }
 }
 return maxIndex;
}


float SensorOpt3001_convert(uint16_t iRawData)
{
  uint16_t iExponent, iMantissa;
  iMantissa = iRawData & 0x0FFF;                 // Extract Mantissa
  iExponent = (iRawData & 0xF000) >> 12;         // Extract Exponent 
  return iMantissa * (0.01 * pow(2, iExponent)); // Calculate final LUX
}
