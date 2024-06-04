#include "Servo.h"
#include <Servo.h>

#define SERVO1 4
Servo myservo;
int pos = 150;
int speed = 1000;

void setup() {
  // put your setup code here, to run once:
  myservo.attach(SERVO1);
}

void loop() {
  // put your main code here, to run repeatedly:
  // code from nav that we are in position
  //
  //
  
  myservo.write(pos);

  for (pos = 120; pos >= 75; pos -= 2)  // goes from 0 degrees to 180 degrees    // in steps of 1 degree
  {
     myservo.write(pos);              // tell servo to go to position in variable 'pos'
     delay(50);                       // waits 50ms for the servo to reach the position
  }
  delay(1000);

  // wait for Nav to give ok to open gripper
  //
  //

  for (pos = 75; pos <= 150; pos += 2)  // goes from close degrees to open degrees    // in steps of 1 degree
  {
     myservo.write(pos);              // tell servo to go to position in variable 'pos'
     delay(50);                       // waits 50ms for the servo to reach the position
  }
    //break;
} //end of loop()
