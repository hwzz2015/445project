#include <AccelStepper.h>
#define HALFSTEP 8

// Motor pin definitions
#define motorPin1  9     // IN1 on the ULN2003 driver 1
#define motorPin2  10    // IN2 on the ULN2003 driver 1
#define motorPin3  11     // IN3 on the ULN2003 driver 1
#define motorPin4  12     // IN4 on the ULN2003 driver 1

// Initialize with pin sequence IN1-IN3-IN2-IN4 for using the AccelStepper with 28BYJ-48
AccelStepper stepper1(HALFSTEP, motorPin1, motorPin3, motorPin2, motorPin4);

void setup() {
  
  stepper1.setMaxSpeed(1200.0);
  stepper1.setAcceleration(600.0);
  stepper1.setSpeed(300);
  stepper1.moveTo(0);

  Serial.begin(9600);
  while (!Serial);             // Leonardo: wait for serial monitor
  Serial.println("\nstart");
  stepper1.runToNewPosition(0);

}//--(end setup )---

float pos =0;
float stepsize = 2000; //negative for clock-wise
int steps = 20;

void loop() {

  //Change direction when the stepper reaches the target position
  /*if (stepper1.distanceToGo() == 0) {
    stepper1.moveTo(-stepper1.currentPosition());
  }
  */
  //stepper1.runToNewPosition(-20000);
  
  
  if(pos>stepsize*steps or pos < -stepsize*steps)stepsize *= -1;
  pos += stepsize;
  stepper1.runToNewPosition(pos);
  Serial.println(stepper1.currentPosition());
}
