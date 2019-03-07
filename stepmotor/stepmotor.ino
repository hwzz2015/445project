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
  stepper1.setMaxSpeed(1000.0);
  stepper1.setAcceleration(500.0);
  stepper1.setSpeed(800);
  stepper1.moveTo(0);

}//--(end setup )---

void loop() {

  //Change direction when the stepper reaches the target position
  /*if (stepper1.distanceToGo() == 0) {
    stepper1.moveTo(-stepper1.currentPosition());
  }
  */
  
  stepper1.runToNewPosition(0);
  stepper1.runToNewPosition(4000);
}
