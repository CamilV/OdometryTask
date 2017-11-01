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
void Robot::Forward(float Distance){
  Wire.beginTransmission(Adress);
  Wire.write(Mode);
  Wire.write(0x01);            // sets the MD25 to mode 1
  Wire.endTransmission();
  
  Wire.beginTransmission(Adress);
  Wire.write(Command);
  Wire.write(0x20);            // reset encoders to zero
  Wire.endTransmission();
  
  
  
  
  int LastError = 0, SumError = 0, Error;
  int Speed;
  int CurrentDistance, n=0;
  while(!Goal)
  {
    ReadEncoders();
    CurrentDistance = (E1+E2) / 2;
    Error = Distance - CurrentDistance;
    
    
    if(Error > THRESHOLD) Speed = 127;
    else
    {
      Speed = Kp*Error + Kd*(Error - LastError) + Ki * SumError;
      LastError = Error;
      SumError = SumError + Error;
    }
    
    Drive(Speed);
    if(Error == 0) n++;
    if(Error > 50) n=0;         // random value
    if(n == 10) Goal = 1;       // random value
  }
  
  Wire.beginTransmission(Adress);
  Wire.write(Speed1);
  Wire.write(0);
  Wire.endTransmission();       // set speed of motor 1 (left) to 0

  Wire.beginTransmission(Adress);
  Wire.write(Speed2);
  Wire.write(0);
  Wire.endTransmission();      // set speed of motor 2 (right) to 0
}


void Robot::Drive(int Speed){
  int MotorCorrection, S1, S2, Error;
  

  
  Error = E2 - E1;
  MotorCorrection = DKp * Error + DKd * (Error - LastErrorF) + DKi * SumErrorF;
  LastErrorF = Error;
  SumErrorF = SumErrorF + Error;   //PID Controller

  S1 = Speed + MotorCorrection;
  S2 = Speed - MotorCorrection;  // calculating the speed

  if(S1 > 127) S1 = 127;
  if(S2 > 127) S2 = 127;
  if(S1 < -128) S1 = -128;
  if(S2 < -128) S2 = -128;     // capping the values for the motor speeds

    
  Wire.beginTransmission(Adress);
  Wire.write(Speed1);
  Wire.write(S1);
  Wire.endTransmission();       // set speed of motor 1 (left)

  Wire.beginTransmission(Adress);
  Wire.write(Speed2);
  Wire.write(S2);
  Wire.endTransmission();      // set speed of motor 2 (right)
}

