#include <Wire.h>
#include "ROBOT.h"
#include "defines.h"

float LastErrorF = 0, SumErrorF = 0;

void Robot::Initialize()
{
  Serial.begin(9600);
  Wire.begin();
  pinMode(LED,OUTPUT);
  pinMode(Buzzer,OUTPUT);
  pinMode(Servo,OUTPUT);
}

void Robot::ReadEncoders()
{
  E1 = 0;
  E2 = 0;
  Wire.beginTransmission(Adress);                      // Send byte to get a reading from encoder 1
  Wire.write(Encoder1);
  Wire.endTransmission();
  
  Wire.requestFrom(Adress, 4);                         // Request 4 bytes from MD25
  while(Wire.available() < 4);                             // Wait for 4 bytes to arrive
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
  Serial.print(E1);
  Serial.print("  ");
  Serial.println(E2);

}
void Robot::Forward(float Distance){       // function that should make the robot drive forward in a straight line for a specified distance
  Serial.print("Forward");
  Wire.beginTransmission(Adress);
  Wire.write(Mode);
  Wire.write(0x01);            // sets the MD25 to mode 1
  Wire.endTransmission();
  
  Wire.beginTransmission(Adress);
  Wire.write(Command);
  Wire.write(0x20);            // sends byte to restore the encoders to 0
  //Wire.write(0x30);         
  Wire.endTransmission();
  Distance = int(360 * Distance / (CircumferenceWheel1/2 + CircumferenceWheel2/2) + 0.5);
  
  
  float LastError = 0, SumError = 0, Error;
  float Speed;
  float CurrentDistance;
  int n=0;
  Goal = 0;
  SumErrorF = 0;
  while(!Goal)                           // while it hasnt reached the target distance, it repeats this loop
  {
    
    ReadEncoders();                      // read the values of the encoders and stores them in E1 and E2
    CurrentDistance = (E1+E2) / 2;       // computes the distance travelled as the average of the 2 encoders
    Error = Distance - CurrentDistance;  // computes the error in order to use the PID controller
    if(CurrentDistance < 180) Speed = 25 + CurrentDistance*0.6;
    
    else if(Error > THRESHOLD) Speed = 127;   // is the error is quite big, it travells at maximum speed
    else
    {
      Speed = Kp*Error + Kd*(Error - LastError) + Ki * SumError;    // standard PID controller, though Ki might not be needed as the Integral of the distance does not have any significance
      LastError = Error;                                            // in order to compute the derivative term
      SumError = SumError + Error;                                  // in order to compute the integral term
    }
    
    Drive(Speed);
    if((Error < 5) && (Error >-5)) n++;         // every time the error is 0, the counter goes up by 1
    if((Error > 20)&&(Error < -20)) n=0;         // random value, if the value of the error increases (overshoots), the counter resets
    if(n == 5) Goal = 1;       // random value, as soon as the counter reaches 10, it breaks out of the loop (it reaches the goal)
    Serial.print("   ");
    Serial.print(n);
    Serial.print("   ");
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

// this function ensures that the robot dirves straight
void Robot::Drive(float Speed){
  float MotorCorrection, S1, S2; 
  float Error;
  if(Speed > 127) Speed = 127;
  if(Speed < -128) Speed = -128;      // caps the speed 
  Wire.beginTransmission(Adress);
  Wire.write(Mode);
  Wire.write(1);            // sets the MD25 to mode 1
  Wire.endTransmission();

  
  Error = E2 * CircumferenceWheel2/1000 - E1 * CircumferenceWheel1/1000; // uses the circumfernce of the wheels as a parameter because the wheels are slighty different in size
  Serial.print("   ");
  Serial.print(Error);
  Serial.print("   ");
  MotorCorrection = DKp * Error + DKd * (Error - LastErrorF) + DKi * SumErrorF;    // standard PID controller, needs the Ki term, calculates a motor correction factor
  // MotorCorrection = 0;
  LastErrorF = Error;              // in order to compute the derivative term
  SumErrorF = SumErrorF + Error;   // in order to compute the integral term

  S1 = Speed + MotorCorrection;
  S2 = Speed - MotorCorrection;  // adds the motor correction factor to the left motor, while it substracts it from the right one

  if(S1 > 127) S1 = 127;
  if(S2 > 127) S2 = 127;
  if(S1 < -128) S1 = -128;     // capping the values for the motor speeds, as a value bigger than 127 or smaller than -128 might fuck up the controller
  if(S2 < -128) S2 = -128;
  Serial.print("   ");
  Serial.print(Speed);
  Serial.print("   ");
  Serial.print("   ");
  Serial.print(S1);
  Serial.print("   ");
  Serial.print("   ");
  Serial.print(S2);
  Serial.print("   ");        // debbuging purpose

  Serial.print("PID");
  Wire.beginTransmission(Adress);
  Wire.write(Speed1);
  Wire.write(int(S1+0.5));
  Wire.endTransmission();       // set speed of motor 1 (left)

  Wire.beginTransmission(Adress);
  Wire.write(Speed2);
  Wire.write(int(S2+0.5));
  Wire.endTransmission();      // set speed of motor 2 (right)
  //delay(2);
}

void Robot::Turn(float Radius, int Degrees, bool Direction){  // left = 0 right = 1
  Wire.beginTransmission(Adress);
  Wire.write(Mode);
  Wire.write(0x01);            // sets the MD25 to mode 1
  Wire.endTransmission();
  
  Wire.beginTransmission(Adress);
  Wire.write(Command);
  Wire.write(0x20);            // sends byte to restore the encoders to 0
  Wire.endTransmission();

  int LastError = 0,  Error;
  SumError = 0;
  float Speed;
  int CurrentAngle, n=0;
  float Angle1, Angle2;
  Goal = 0;
  while(!Goal){
    ReadEncoders();
    if(Direction == 0){         // case we want to turn left
      Angle1 = E1 * 180/((Radius - WidthRobot/2)*3.14);      // calculates the angle of the wheel 1
      Angle2 = E2 * 180/((Radius + WidthRobot/2)*3.14);      // calculates the angle of the wheel 2
      
    }
    else{                         // case we want to turn right
      Angle1 = E1 * 180/((Radius + WidthRobot/2)*3.14);
      Angle2 = E2 * 180/((Radius - WidthRobot/2)*3.14);
    }
    CurrentAngle = (Angle2+Angle1)/2;

    Error = Degrees - CurrentAngle;
    if(Error > 20) Speed = 127;
    else {Speed = TKp * Error + TKd * (Error - LastError);
          LastError = Error;
    }
    StraightTurn(Radius, Direction, Speed);
    if(Error == 0) n++;         // every time the error is 0, the counter goes up by 1
    if(Error > 50) n=0;         // random value, if the value of the error increases (overshoots), the counter resets
    if(n == 10) Goal = 1;
  }
}

// ensures the robot stays paralel to the radius of the curvature
void Robot::StraightTurn(float Radius, bool Direction,float Speed){
  float Angle1, Angle2, Error,LastError, MotorCorrection;
  float S1, S2;
  if(Direction == 1){   // 2 cases for both direction
      Angle1 = E1 * 180/((Radius + WidthRobot/2)*3.14);
      Angle2 = E2 * 180/((Radius - WidthRobot/2)*3.14);
      Error = Angle1 - Angle2;
      MotorCorrection = DeKp * Error + DeKd * (Error - LastError) + DeKi * SumError; 
      LastError = Error;
      SumError = SumError + Error;
      S2 = Speed + MotorCorrection;
      S1 = Speed - MotorCorrection;
    }
    else{
      Angle1 = E1 * 180/((Radius - WidthRobot/2)*3.14);
      Angle2 = E2 * 180/((Radius + WidthRobot/2)*3.14);
      Error = Angle2 - Angle1;
      MotorCorrection = DeKp * Error + DeKd * (Error - LastError) + DeKi * SumError; 
      LastError = Error;
      SumError = SumError + Error;
      S2 = Speed - MotorCorrection;
      S1 = Speed + MotorCorrection;
    }
    if(S1 > 127) S1 = 127;
    if(S2 > 127) S2 = 127;
    if(S1 < -128) S1 = -128;     // capping the values for the motor speeds, as a value bigger than 127 or smaller than -128 might fuck up the controller
    if(S2 < -128) S2 = -128;
    
    Wire.beginTransmission(Adress);
    Wire.write(Speed1);
    Wire.write(int(S1+0.5));
    Wire.endTransmission();       // set speed of motor 1 (left)
  
    Wire.beginTransmission(Adress);
    Wire.write(Speed2);
    Wire.write(int(S2+0.5));
    Wire.endTransmission();      // set speed of motor 2 (right)

    
}

