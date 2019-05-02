// Based on the following code:
// OPT3001 with Arduino (Lux meter)
// by Varad Kulkarni <http://www.microcontrollershub.com>
// Created 28 March 2018

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
//#include <TimeLib.h>
#include <AccelStepper.h>
#define HALFSTEP 8
#include <Stepper.h>
// Motor pin definitions
#define motorPin1  9     // IN1 on the ULN2003 driver 1
#define motorPin2  10    // IN2 on the ULN2003 driver 1
#define motorPin3  11     // IN3 on the ULN2003 driver 1
#define motorPin4  12     // IN4 on the ULN2003 driver 1


#define step_size 0.0078125   //step angle of the motor
#define step_thred 1.0   //threshold for rotation
#define total_step 18377
// Initialize with pin sequence IN1-IN3-IN2-IN4 for using the AccelStepper with 28BYJ-48
//AccelStepper stepper1(16, motorPin1, motorPin3, motorPin2, motorPin4);
Stepper stepper1(2048, motorPin1, motorPin3, motorPin2, motorPin4);

File myFile;

float last_angle = 0;
float cur_angle = 0;

void setup() 
{ 
  Serial.begin(9600);                            // Initialize serial communication at 9600
  Wire.begin();                                  // Initialize Arduino in I2C master.
  Wire.beginTransmission(0x44);                  // I2C address of OPT3001 = 0x44
  Wire.write(0x01);                              // Config register address 0x01
  Wire.write(0xCE);
  Wire.write(0x10);                              // Write 0xCE10 to turn on sensor
  Wire.endTransmission();
  Serial.println("Device x44 Found!");
  
  Wire.beginTransmission(0x45);                  // I2C address of OPT3001 = 0x45
  Wire.write(0x01);                              // Config register address 0x01
  Wire.write(0xCE);
  Wire.write(0x10);                              // Write 0xCE10 to turn on sensor
  Wire.endTransmission();
  Serial.println("Device x45 Found!");

  Wire.beginTransmission(0x46);                  // I2C address of OPT3001 = 0x46
  Wire.write(0x01);                              // Config register address 0x01
  Wire.write(0xCE);
  Wire.write(0x10);                              // Write 0xCE10 to turn on sensor
  Wire.endTransmission();
  Serial.println("Device x46 Found!");
  
  //stepper1.setMaxSpeed(1000.0);
 // stepper1.setAcceleration(100.0);
  stepper1.setSpeed(15);
 // stepper1.step(2048);

  Wire.beginTransmission(0x47);                  // I2C address of OPT3001 = 0x47
  Wire.write(0x01);                              // Config register address 0x01
  Wire.write(0xCE);
  Wire.write(0x10);                              // Write 0xCE10 to turn on sensor
  Wire.endTransmission();

 
/*
  Serial.print("Initializing SD card...");

  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");
  setTime(10,54,0,0,0,0);
  pinMode(8, OUTPUT);
  Serial.println("start");
  */
}
void loop() 
{ 
  float flux[4];
  float angle[4] = {135,-135 ,-45,45};
  
  Wire.beginTransmission(0x44);
  Wire.write(0x00);                              // Send result register address
  Wire.endTransmission();
  delay(10);
  
  Wire.requestFrom(0x44, 2);                     // Request 2 bytes data from OPT3001
  uint16_t iData;
  uint8_t  iBuff[2];
  while (Wire.available())
  {
    Wire.readBytes(iBuff, 2);
    iData = (iBuff[0] << 8) | iBuff[1];
    //Serial.print(iData,BIN);                     // Print the received data
    flux[0] = SensorOpt3001_convert(iData);   // Calculate LUX from sensor data
  }
  //Serial.println("x44 Data Read!");
  
  Wire.beginTransmission(0x45);
  Wire.write(0x00);                              // Send result register address
  Wire.endTransmission();
  delay(10);
  
  Wire.requestFrom(0x45, 2);                     // Request 2 bytes data from OPT3001
  while (Wire.available())
  {
  
    Wire.readBytes(iBuff, 2);
    iData = (iBuff[0] << 8) | iBuff[1];
    //Serial.print(iData,BIN);                     // Print the received data
    flux[1] = SensorOpt3001_convert(iData);   // Calculate LUX from sensor data
  }
  //Serial.println("x45 Data Read!");

  

  Wire.beginTransmission(0x46);
  Wire.write(0x00);                              // Send result register address
  Wire.endTransmission();
  delay(10);
  
  Wire.requestFrom(0x46, 2);                     // Request 2 bytes data from OPT3001
  while (Wire.available())
  {
  
    Wire.readBytes(iBuff, 2);
    iData = (iBuff[0] << 8) | iBuff[1];
    //Serial.print(iData,BIN);                     // Print the received data
    flux[2] = SensorOpt3001_convert(iData);   // Calculate LUX from sensor data
  }
  //Serial.println("x46 Data Read!");

  Wire.beginTransmission(0x47);
  Wire.write(0x00);                              // Send result register address
  Wire.endTransmission();
  delay(10);
  
  Wire.requestFrom(0x47, 2);                     // Request 2 bytes data from OPT3001
  while (Wire.available())
  {
  
    Wire.readBytes(iBuff, 2);
    iData = (iBuff[0] << 8) | iBuff[1];
    //Serial.print(iData,BIN);                     // Print the received data
    flux[3] = SensorOpt3001_convert(iData);   // Calculate LUX from sensor data
  }
  //Serial.println("x45 Data Read!")
  
 
  Serial.print(flux[0]);
  Serial.print("    ");
  Serial.print(flux[1]);
  Serial.print("    ");
  Serial.print(flux[2]);
  Serial.print("    ");
  Serial.println(flux[3]);
  

  /*
  Serial.print(hour());
  printDigits(minute());
  printDigits(second());
  Serial.println("    ");
*/
  last_angle = cur_angle;
  cur_angle = getOutAngle(flux,angle);
  float rotation_angle = cur_angle-last_angle;
  Serial.println("Rotation Angle");
  Serial.println(rotation_angle);
  if(cur_angle == 0){
    //stepper1.moveTo(0);
    Serial.println("Reset to Zero Position");
  }
  else if(abs(rotation_angle) >= step_thred){
    
    Serial.println(rotation_angle*total_step/270.0); 
    
    int num_step = (int) (rotation_angle/270.0*total_step);
    //stepper1.runToNewPosition(num_step); 
    Serial.println("Rotate Steps: ");
    Serial.println(num_step);
    stepper1.step(num_step); 
    
  }
  Serial.println("Current Brim Angle");
  Serial.println(cur_angle);
  delay(3000);
  //Serial.println("Current Motor Position");
  //Serial.println(stepper1.currentPosition());
  /*
  myFile = SD.open("Record1.txt", FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile) {
    myFile.print(flux1);
    myFile.print(" ");
    myFile.print(flux2);
    myFile.print(" ");
    myFile.print(flux3);
    myFile.print(" ");
    myFile.print(hour());
    myFile.print(":");
    myFile.print(minute());
    myFile.print(":");
    myFile.print(second());
    myFile.println("    ");
   
    myFile.close();
    
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }


  
  for(int i = 0;i< 30;i++){
  digitalWrite(8, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(8, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
  }
  */
}


float SensorOpt3001_convert(uint16_t iRawData)
{
  uint16_t iExponent, iMantissa;
  iMantissa = iRawData & 0x0FFF;                 // Extract Mantissa
  iExponent = (iRawData & 0xF000) >> 12;         // Extract Exponent 
  return iMantissa * (0.01 * pow(2, iExponent)); // Calculate final LUX
}

void printDigits(int digits) {
 // utility function for digital clock display: prints preceding colon and leading 0
 Serial.print(":");
 if (digits < 10)
 Serial.print('0');
 Serial.print(digits);
}

float getOutAngle(float flux[4], float angle[4]){
  float threshold = 20;
  bool b = false;
  int i;
  for(i=0; i<4; i++){
    b = b || (flux[i]>threshold);
  }
  if(!b){
    return 0;
  }
  float max_flux = 0;
  float max2_flux = 0;
  float max_angle = 0;
  float max2_angle = 0;
  for(i=0;i<4;i++){
    if(flux[i]>max_flux){
      max2_flux = max_flux;
      max_flux = flux[i];
      max2_angle = max_angle;
      max_angle = angle[i];
    }
    else if(flux[i]>max2_flux){
      max2_flux = flux[i];
      max2_angle = angle[i];
    }
  }
  float ratio = max_flux/(max_flux+max2_flux);
  return max_angle*ratio+max2_angle*(1-ratio);
}
