#include <Wire.h>
#include "ROBOT.h"
#include "defines.h"

float LastErrorF = 0, SumErrorF = 0;

void Robot::ReadEncoders()
{
  E1 = 0;
  E2 = 0;
  Wire.beginTransmission(Adress);                      // Send byte to get a reading from encoder 1
  Wire.write(Encoder1);
  Wire.endTransmission();
  
  Wire.requestFrom(Adress, 4);                         // Request 4 bytes from MD25
  while(Wire.available() < 4);                              // Wait for 4 bytes to arrive
  E1 = Wire.read();                                 // First byte for encoder 1, HH.
  E1 <<= 8;
  E1 += Wire.read();                                     // Second byte for encoder 1, HL
  E1 <<= 8;
  E1 += Wire.read();                                     // Third byte for encoder 1, LH
  E1 <<= 8;
  E1  +=Wire.read();                                     // Fourth byte for encoder 1, LL

  Wire.beginTransmission(Adress);                      // Send byte to get a reading from encoder 2
  Wire.write(Encoder2);
  Wire.endTransmission();
  
  Wire.requestFrom(Adress, 4);                         // Request 4 bytes from MD25
  while(Wire.available() < 4);                              // Wait for 4 bytes to arrive
  E2 = Wire.read();                                 // First byte for encoder 2, HH.
  E2 <<= 8;
  E2 += Wire.read();                                     // Second byte for encoder 2, HL
  E2 <<= 8;
  E2 += Wire.read();                                     // Third byte for encoder 2, LH
  E2 <<= 8;
  E2  +=Wire.read();                                     // Fourth byte for encoder 2, LL

}
void Robot::Forward(int Distance){       // function that should make the robot drive forward in a straight line for a specified distance
  Wire.beginTransmission(Adress);
  Wire.write(Mode);
  Wire.write(0x01);            // sets the MD25 to mode 1
  Wire.endTransmission();
  
  Wire.beginTransmission(Adress);
  Wire.write(Command);
  Wire.write(0x20);            // sends byte to restore the encoders to 0
  Wire.endTransmission();
  
  
  
  
  int LastError = 0, SumError = 0, Error;
  int Speed;
  int CurrentDistance, n=0;
  Goal = 0;
  while(!Goal)                           // while it wasnt reached the target distance, it repeats this loop
  {
    ReadEncoders();                      // read the values of the encoders and stores them in E1 and E2
    CurrentDistance = (E1+E2) / 2;       // computes the distance travelled as the average of the 2 encoders
    Error = Distance - CurrentDistance;  // computes the error in order to use the PID controller
    
    
    if(Error > THRESHOLD) Speed = 127;   // is the error is quite big, it travells at maximum speed
    else
    {
      Speed = Kp*Error + Kd*(Error - LastError) + Ki * SumError;    // standard PID controller, though Ki might not be needed as the Integral of the distance does not have any significance
      LastError = Error;                                            // in order to compute the derivative term
      SumError = SumError + Error;                                  // in order to compute the integral term
    }
    
    Drive(Speed);
    if(Error == 0) n++;         // every time the error is 0, the counter goes up by 1
    if(Error > 50) n=0;         // random value, if the value of the error increases (overshoots), the counter resets
    if(n == 10) Goal = 1;       // random value, as soon as the counter reaches 10, it breaks out of the loop (it reaches the goal)
  }
  
  Wire.beginTransmission(Adress);
  Wire.write(Speed1);
  Wire.write(0);
  Wire.endTransmission();       // set speed of motor 1 (left) to 0

  Wire.beginTransmission(Adress);
  Wire.write(Speed2);
  Wire.write(0);
  Wire.endTransmission();      // set speed of motor 2 (right) to 0, stops both motors
}


void Robot::Drive(int Speed){
  int MotorCorrection, S1, S2, Error;
  

  
  Error = E2 - E1;
  MotorCorrection = DKp * Error + DKd * (Error - LastErrorF) + DKi * SumErrorF;    // standard PID controller, needs the Ki term, calculates a motor correction factor
  LastErrorF = Error;              // in order to compute the derivative term
  SumErrorF = SumErrorF + Error;   // in order to compute the integral term

  S1 = Speed + MotorCorrection;
  S2 = Speed - MotorCorrection;  // adds the motor correction factor to the left motor, while it substracts it from the right one

  if(S1 > 127) S1 = 127;
  if(S2 > 127) S2 = 127;
  if(S1 < -128) S1 = -128;     // capping the values for the motor speeds, as a value bigger than 127 or smaller than -128 might fuck up the controller
  if(S2 < -128) S2 = -128;

    
  Wire.beginTransmission(Adress);
  Wire.write(Speed1);
  Wire.write(S1);
  Wire.endTransmission();       // set speed of motor 1 (left)

  Wire.beginTransmission(Adress);
  Wire.write(Speed2);
  Wire.write(S2);
  Wire.endTransmission();      // set speed of motor 2 (right)
}

void Robot::Turn(int Radius, int Degrees){
  Wire.beginTransmission(Adress);
  Wire.write(Mode);
  Wire.write(0x01);            // sets the MD25 to mode 1
  Wire.endTransmission();
  
  Wire.beginTransmission(Adress);
  Wire.write(Command);
  Wire.write(0x20);            // sends byte to restore the encoders to 0
  Wire.endTransmission();

  int LastError = 0, SumError = 0, Error;
  int Speed;
  int CurrentAngle, n=0;
  float Angle1, Angle2;
  Goal = 0;
  while(!Goal){
    ReadEncoders();
    Angle1 = E1 * 180/(Radius - WidthRobot/2);
    Angle2 = E2 * 180/(Radius + WidthRobot/2);
    
  }
}

