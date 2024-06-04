#include <Wire.h>
#include "Gyro.h"
#include "Drivetrain.h"
#include "IRSensor.h"

#define GYRO_ADDR 0x68

Gyro gyro(0x68); // object that interacts with the gyroscope chip
Drivetrain motors(6, 9, 10, 11);  // object that interacts with the motors
IRSensor IR_left(14), IR_right(15);

void setup()
{
  Wire.begin(); // the I2C object
 
  Serial.begin(115200);
  Serial.println("\nProgram Start");

  Wire.beginTransmission(GYRO_ADDR);               // check if gyroscope chip is connected
  if(Wire.endTransmission())                       //
  {                                                //
    Serial.println("I2C error");                   //
    while(1){}                                     //
  }                                                //
  Serial.println("I2C connected successfully");    //

  gyro.begin(2);    // initialize the gyroscope and motor objects
  motors.begin();   // 
  IR_left.begin();
  IR_right.begin();
}

float desired_angle = 0;
long timer = 0;

float P = 2 ;
float I = 0.0;
float D = 0.05;
float error_accumulator = 0;
float prev_error = 0;
float accum_cap = 250;
float error_threshhold = 1;

int print_timer = 0;
void loop()
{
//  Serial.println(IR_left.readRaw());
//  delay(100);
  gyro.update();  // get gyroscope raw data and integrate to get position
  
  if(Serial.available())  // handle commands sent over serial
  {
    switch(Serial.read())
    {
      case 'a':
      {
        desired_angle = gyro.wrap360(Serial.parseFloat());
        break;
      }
      default:
        break;
    }
  }


  ///// closed loop PID control to maintain robot angle /////
  
  float error = desired_angle - gyro.angle;
  if(error > 180)
    error = error - 360;
  else if(error < -180)
    error = error + 360;

  if(abs(error) < error_threshhold)
    error = 0;
    
  error_accumulator += error * 0.0025;
  error_accumulator = motors.bound(error_accumulator, accum_cap);

  float error_rate = (error - prev_error) / 0.0025;
  prev_error = error;

  motors.setTurnRate((P * error + I * error_accumulator) / 100);

  

  print_timer++;
  if(print_timer > 10)
  {
    Serial.print(gyro.rate);
    Serial.print("\t - ");
    Serial.println(gyro.angle);
    print_timer = 0;
  }
  while(micros() - timer < 2500){} // delay so that the total time between loops is 2500 microseconds
  timer = micros();                //
}
