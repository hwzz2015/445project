// OPT3001 with Arduino (Lux meter)
// by Varad Kulkarni <http://www.microcontrollershub.com>
// Created 28 March 2018

#include <Wire.h>
void setup() 
{
  Serial.begin(9600);                            // Initialize serial communication at 9600
  Wire.begin();                                  // Initialize Arduino in I2C master.
  Wire.beginTransmission(0x44);                  // I2C address of OPT3001 = 0x44
  Wire.write(0x01);                              // Config register address 0x01
  Wire.write(0xCE);
  Wire.write(0x10);                              // Write 0xCE10 to turn on sensor
  Wire.endTransmission();

  Wire.beginTransmission(0x45);                  // I2C address of OPT3001 = 0x45
  Wire.write(0x01);                              // Config register address 0x01
  Wire.write(0xCE);
  Wire.write(0x10);                              // Write 0xCE10 to turn on sensor
  Wire.endTransmission();

  
  Serial.println("start");
}
void loop() 
{ 
  float flux1, flux2;
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
    flux1 = SensorOpt3001_convert(iData);   // Calculate LUX from sensor data
  }
  
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
    flux2 = SensorOpt3001_convert(iData);   // Calculate LUX from sensor data
  }
  Serial.print(flux1);
  Serial.print(" ++ ");
  Serial.println(flux2);
  delay(100);
}
float SensorOpt3001_convert(uint16_t iRawData)
{
  uint16_t iExponent, iMantissa;
  iMantissa = iRawData & 0x0FFF;                 // Extract Mantissa
  iExponent = (iRawData & 0xF000) >> 12;         // Extract Exponent 
  return iMantissa * (0.01 * pow(2, iExponent)); // Calculate final LUX
}
