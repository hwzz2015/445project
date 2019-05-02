//ECE 445 senior design
//Self-adjust helmat
//April 15th, 2019

//physical digital pins: 
//accelerometer: 2
//speaker: 9
//leds:10
//motors: 5-8
//switch: 11,12

#include "SparkFunLIS3DH.h"
#include "Wire.h"
#include "SPI.h"
#include "Stepper.h"

#define NOTE_A4  440
#define NOTE_DS5 622

//fall detection pin definitions
const int FALL_DETECTION = 2;
const int SPEAKER = 9;
const int LED = 10;

// Motor pin definitions
#define motorPin1  5     // IN1 on the ULN2003 driver 1
#define motorPin2  6    // IN2 on the ULN2003 driver 1
#define motorPin3  7     // IN3 on the ULN2003 driver 1
#define motorPin4  8     // IN4 on the ULN2003 driver 1

#define switchPin1  11
#define switchPin2  12   

int alarm_flag = 0;

#define step_size 0.0078125   //step angle of the motor
#define step_thred 1.0   //threshold for rotation
#define total_step 18377

float last_angle = 0;
float cur_angle = 0;

const int sensors[4] = {0x44,0x45,0x46,0x47};

// Initialize with pin sequence IN1-IN3-IN2-IN4 for using the AccelStepper with 28BYJ-48
//AccelStepper stepper1(16, motorPin1, motorPin3, motorPin2, motorPin4);
Stepper stepper1(2048, motorPin1, motorPin3, motorPin2, motorPin4);

LIS3DH myIMU(I2C_MODE, 0x18); //Default constructor is I2C, addr 0x19.


void setup() {
  Serial.begin(9600);
  delay(300); //relax...
  Serial.println("Initializing");
  Wire.begin();                                  // Initialize Arduino in I2C master.

  //setup light sensor i2c communication
  for(int i=0;i<4;i++){
  Wire.beginTransmission(sensors[i]);                  // I2C address of OPT3001 = 0x44
  Wire.write(0x01);                              // Config register address 0x01
  Wire.write(0xCE);
  Wire.write(0x10);                              // Write 0xCE10 to turn on sensor
  Wire.endTransmission();
  Serial.print(" Device Found:");
  Serial.print(sensors[i]);
  }

  //setting up motor speed
  stepper1.setSpeed(15);
  
  //Accel sample rate and range effect interrupt time and threshold values!!!
  myIMU.settings.accelSampleRate = 50;  //Hz.  Can be: 0,1,10,25,50,100,200,400,1600,5000 Hz
  myIMU.settings.accelRange = 2;      //Max G force readable.  Can be: 2, 4, 8, 16

  myIMU.settings.adcEnabled = 0;
  myIMU.settings.tempEnabled = 0;
  myIMU.settings.xAccelEnabled = 1;
  myIMU.settings.yAccelEnabled = 1;
  myIMU.settings.zAccelEnabled = 1;
  
  //Call .begin() to configure the IMU
  myIMU.begin();
  
  configIntterupts();

  Serial.println("\nSetting the Interrupt Event");
  pinMode(FALL_DETECTION,INPUT);
  pinMode(SPEAKER,OUTPUT);
  pinMode(LED,OUTPUT);
  attachInterrupt(digitalPinToInterrupt(FALL_DETECTION),setAlarm,RISING);

  pinMode(switchPin1,INPUT);
  pinMode(switchPin1,INPUT);

  //motor initialize
  int buttonState2 = digitalRead(switchPin2);
 
  Serial.println(buttonState2);
  if(buttonState2 == HIGH){
    Serial.println(" clockwise");
    }
  while(buttonState2 == HIGH){
    //Serial.print(buttonState2);
    stepper1.step(-1*5);
    delay(5);
    buttonState2 = digitalRead(switchPin2);
  }
  
  Serial.println("limit, counterclockwise");
  stepper1.step(9000);
  Serial.println("------------------Initialize finished----------------------");
  delay(500);
}

void loop()
{
  lightmotor();
  falldetect();
  delay(500);
}


void lightmotor() 
{ 
  float flux[4];
  float angle[4] = {135,-135 ,-45,45};
  
  Serial.print("light sensor values: ");
  for(int i=0;i<4;i++){
    Wire.beginTransmission(sensors[i]);
    Wire.write(0x00);                              // Send result register address
    Wire.endTransmission();
    delay(20);
    
    Wire.requestFrom(sensors[i], 2);                     // Request 2 bytes data from OPT3001
    uint16_t iData;
    uint8_t  iBuff[2];
    while (Wire.available())
    {
      Wire.readBytes(iBuff, 2);
      iData = (iBuff[0] << 8) | iBuff[1];
      //Serial.print(iData,BIN);                     // Print the received data
      flux[i] = SensorOpt3001_convert(iData);   // Calculate LUX from sensor data
    }
    delay(20);
    Serial.print(flux[i]);
    Serial.print("    ");
  }
 
  last_angle = cur_angle;
  cur_angle = getOutAngle(flux,angle);
  float rotation_angle = cur_angle-last_angle;
  Serial.print("\nRotation Angle ");
  Serial.println(rotation_angle);
  if(cur_angle == 0){
    Serial.println("Reset to Zero Position");
  }
  else if(abs(rotation_angle) >= step_thred){
    //Serial.println(rotation_angle*total_step/270.0);  //debug
    int num_step = (int) (rotation_angle/270.0*total_step); 
    Serial.print("Rotate Steps: ");
    Serial.println(num_step);
    stepper1.step(num_step); 
  }
  Serial.print("Current Brim Angle ");
  Serial.println(cur_angle);
  delay(100);
}

void falldetect()
{
//  Get all parameters
  Serial.print("\nAccelerometer:\n");
  Serial.print(" X = ");
  Serial.println(myIMU.readFloatAccelX(), 4);
  Serial.print(" Y = ");
  Serial.println(myIMU.readFloatAccelY(), 4);
  Serial.print(" Z = ");
  Serial.println(myIMU.readFloatAccelZ(), 4);

  uint8_t dataRead;

  Serial.print("LIS3DH_INT1_SRC: 0x");
  myIMU.readRegister(&dataRead, LIS3DH_INT1_SRC);//cleared by reading
  Serial.println(dataRead, HEX);
  Serial.println("Decoded events:");
  if(dataRead & 0x40) Serial.println("Interrupt Active");
  if(dataRead & 0x20) Serial.println("Z high");
  if(dataRead & 0x10) Serial.println("Z low");
  if(dataRead & 0x08) Serial.println("Y high");
  if(dataRead & 0x04) Serial.println("Y low");
  if(dataRead & 0x02) Serial.println("X high");
  if(dataRead & 0x01) Serial.println("X low");
  Serial.println();

  if(alarm_flag == 1){
    alarm();
    alarm_flag = 0;
  }

  delay(100);
}

void configIntterupts()
{
  uint8_t dataToWrite = 0;

  //LIS3DH_INT1_CFG   
  //dataToWrite |= 0x80;//AOI, 0 = OR 1 = AND
  //dataToWrite |= 0x40;//6D, 0 = interrupt source, 1 = 6 direction source
  //Set these to enable individual axes of generation source (or direction)
  // -- high and low are used generically
  //dataToWrite |= 0x20;//Z high
  dataToWrite |= 0x10;//Z low
  //dataToWrite |= 0x08;//Y high
  //dataToWrite |= 0x04;//Y low
  //dataToWrite |= 0x02;//X high
  //dataToWrite |= 0x01;//X low
  myIMU.writeRegister(LIS3DH_INT1_CFG, dataToWrite);
  
  //LIS3DH_INT1_THS   
  dataToWrite = 0;
  //Provide 7 bit value, 0x7F always equals max range by accelRange setting
  dataToWrite |= 0x20; // 1/8 range
  myIMU.writeRegister(LIS3DH_INT1_THS, dataToWrite);
  
  //LIS3DH_INT1_DURATION  
  dataToWrite = 0;
  //minimum duration of the interrupt
  //LSB equals 1/(sample rate)
  dataToWrite |= 0x09; // 1/50 s = 20ms
  myIMU.writeRegister(LIS3DH_INT1_DURATION, dataToWrite);
  
  //LIS3DH_CTRL_REG3
  //Choose source for pin 1
  dataToWrite = 0;
  //dataToWrite |= 0x80; //Click detect on pin 1
  dataToWrite |= 0x40; //AOI1 event (Generator 1 interrupt on pin 1)
  //dataToWrite |= 0x20; //AOI2 event ()
  //dataToWrite |= 0x10; //Data ready
  //dataToWrite |= 0x04; //FIFO watermark
  //dataToWrite |= 0x02; //FIFO overrun
  myIMU.writeRegister(LIS3DH_CTRL_REG3, dataToWrite);
 
  //LIS3DH_CTRL_REG6
  //Choose source for pin 2 and both pin output inversion state
  dataToWrite = 0;
  //dataToWrite |= 0x80; //Click int on pin 2
  //dataToWrite |= 0x40; //Generator 1 interrupt on pin 2
  //dataToWrite |= 0x10; //boot status on pin 2
  //dataToWrite |= 0x02; //invert both outputs
  myIMU.writeRegister(LIS3DH_CTRL_REG6, dataToWrite);
}

//sensor raw value conversion
float SensorOpt3001_convert(uint16_t iRawData)
{
  uint16_t iExponent, iMantissa;
  iMantissa = iRawData & 0x0FFF;                 // Extract Mantissa
  iExponent = (iRawData & 0xF000) >> 12;         // Extract Exponent 
  return iMantissa * (0.01 * pow(2, iExponent)); // Calculate final LUX
}

//motor angle decision
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

//fall alarm flag
void setAlarm(){
  alarm_flag = 1;
  return;
}

//sound and light for fall detect
void alarm(){
  Serial.println("Alarm!!!");
  int i=0;
  for(i=0; i<2; i++){
    digitalWrite(LED,HIGH);
    delay(1000); 
    tone(SPEAKER,NOTE_A4,1000);
    digitalWrite(LED,LOW);
    delay(1000); 
    tone(SPEAKER,NOTE_A4,1000); 
  }
  return;  
}
