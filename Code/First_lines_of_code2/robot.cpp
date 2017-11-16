#include <Wire.h>
#include "ROBOT.h"
#include "defines.h"
#include <Servo.h>

float LastErrorF = 0, SumErrorF = 0;

Servo myServo;
/*
 * Function used to initialize all starting parameters of the robot
 */
void Robot::Initialize()
{
  Serial.begin(9600);
  Wire.begin();
  pinMode(LED,OUTPUT);
  pinMode(Buzzer,OUTPUT);
  myServo.attach(ServoMotor);
  myServo.write(servoPos);
  delay(1000);
}
/*
 * Function used to read both encoders and store their value into the global variables E1 and E2
 * code is taken from the Odometry Briefing document and adapted to work for both encoders
 */
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
//  Serial.print(E1);
//  Serial.print("  ");
//  Serial.println(E2);

}
/*
 * Function that takes as an input a distance in mm and makes the robot drive in a straight line that distance
 * Uses a PID controller to compute the required speed which is send to the motors through th Drive() function
 * Implements an acceleration as the function is called in order to prevent sudden acceleration
 * Uses a counter to check wheter or not it has reached its required distance and it hasnt overshot
 */
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
    if(CurrentDistance < 180 && Distance > 401) Speed = 25 + CurrentDistance*0.6;
    
    else if(Error > THRESHOLD) Speed = 100;   // is the error is quite big, it travells at maximum speed
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

/*
 * Private function only called by the Forward() function
 * Implements a PID controller to ensure the robot drives straight at all times
 */
void Robot::Drive(float Speed){
  float MotorCorrection, S1, S2; 
  float Error;
  if(Speed > 100) Speed = 100;
  if(Speed < -100) Speed = -100;      // caps the speed 
  Wire.beginTransmission(Adress);
  Wire.write(Mode);
  Wire.write(1);            // sets the MD25 to mode 1
  Wire.endTransmission();

  
  Error = E2 * CircumferenceWheel2/1000 - E1 * CircumferenceWheel1/1000; // uses the circumfernce of the wheels as a parameter because the wheels are slighty different in size
  Serial.print("   ");
  Serial.print(Error);
  Serial.print("   ");
  MotorCorrection = DKp * Error + DKd * (Error - LastErrorF) + DKi * SumErrorF;    // standard PID controller, needs the Ki term, calculates a motor correction factor
//  MotorCorrection = 0;
  LastErrorF = Error;              // in order to compute the derivative term
  SumErrorF = SumErrorF + Error;   // in order to compute the integral term

  S1 = Speed + MotorCorrection;
  S2 = Speed - MotorCorrection;  // adds the motor correction factor to the left motor, while it substracts it from the right one

  if(S1 > 127) S1 = 127;
  if(S2 > 127) S2 = 127;
  if(S1 < -128) S1 = -128;     // capping the values for the motor speeds, as a value bigger than 127 or smaller than -128 might fuck up the controller
  if(S2 < -128) S2 = -128;
//  Serial.print("   ");
//  Serial.print(Speed);
//  Serial.print("   ");
//  Serial.print("   ");
//  Serial.print(S1);
//  Serial.print("   ");
//  Serial.print("   ");
//  Serial.print(S2);
//  Serial.print("   ");        // debbuging purpose

//  Serial.print("PID");
  if(nor%2 == 0){                     // implements a randomizer to power one motor before the other, by powering only one motor before the other, over time the error becomes significant
    Wire.beginTransmission(Adress);
    Wire.write(Speed2);
    Wire.write(int(S2+0.5));
    Wire.endTransmission();       // set speed of motor 1 (left)

    Wire.beginTransmission(Adress);
    Wire.write(Speed1);
    Wire.write(int(S1+0.5));
    Wire.endTransmission();      // set speed of motor 2 (right)
    //delay(2);
    nor++;
  }
  else{
    Wire.beginTransmission(Adress);
    Wire.write(Speed1);
    Wire.write(int(S1+0.5));
    Wire.endTransmission();       // set speed of motor 1 (left)

    Wire.beginTransmission(Adress);
    Wire.write(Speed2);
    Wire.write(int(S2+0.5));
    Wire.endTransmission();      // set speed of motor 2 (right)
    //delay(2);
    nor++;
  }
}
/*
 * Function used to drive around the first arc of the course.
 * Similarly to Forward(), implements a PID controller to deccelerate smoothly to the target distance
 * Because of the lower speed, it doesn't require an initial acceleration
 */
void Robot::Turn1(){  
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
  int  n=0;
  float GD1, GD2;
  Goal = 0;
  GD1 = 1.53*3.1415*(180 - WidthRobot/2);         // computes the distance that each wheel has to travel
  GD2 = 1.53*3.1415*(180 + WidthRobot/2);
  GD1 = int(360*GD1/CircumferenceWheel1 + 0.5);   // converts the distance into encoder ticks
  GD2 = int(360*GD2/CircumferenceWheel2 + 0.5);
  while(!Goal){
    ReadEncoders();
    Error = GD1/2 + GD2/2 - E1/2 - E2/2;
    
    if(Error > THRESHOLD) Speed = 100;
    else {Speed = Kp * Error + Kd * (Error - LastError);
          LastError = Error;
    }
    StraightTurn1(Speed);
    if(abs(Error) < 5) n++;         // every time the error is less than a small value, the counter goes up by 1
    if(abs(Error) > 20) n=0;         // if the value of the error increases (overshoots), the counter resets
    if(n == 5) Goal = 1;
  }
  Serial.print("Finished");
  Wire.beginTransmission(Adress);
  Wire.write(Speed1);
  Wire.write(0);
  Wire.endTransmission();       // set speed of motor 1 (left) to 0

  Wire.beginTransmission(Adress);
  Wire.write(Speed2);
  Wire.write(0);
  Wire.endTransmission();
}
/*
 * Similar to Turn1(), this function is used to get around the second arc of the course
 */
void Robot::Turn2(){  
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
  int  n=0;
  float GD1, GD2;
  Goal = 0;
  GD1 = 0.515*3.1415*(260 - WidthRobot/2);
  GD2 = 0.515*3.1415*(260 + WidthRobot/2);
  GD1 = int(360*GD1/CircumferenceWheel1 + 0.5);
  GD2 = int(360*GD2/CircumferenceWheel2 + 0.5);
  while(!Goal){
    ReadEncoders();
    Error = GD1/2 + GD2/2 - E1/2 - E2/2;
    
    if(Error > THRESHOLD) Speed = 100;
    else {Speed = Kp * Error + Kd * (Error - LastError);
          LastError = Error;
    }
    StraightTurn2(Speed);
    if(abs(Error) < 5) n++;         // every time the error is 0, the counter goes up by 1
    if(abs(Error) > 20) n=0;         // random value, if the value of the error increases (overshoots), the counter resets
    if(n == 5) Goal = 1;
  }
  Serial.print("Finished");
  Wire.beginTransmission(Adress);
  Wire.write(Speed1);
  Wire.write(0);
  Wire.endTransmission();       // set speed of motor 1 (left) to 0

  Wire.beginTransmission(Adress);
  Wire.write(Speed2);
  Wire.write(0);
  Wire.endTransmission();
}
/*
 * Private function called by Turn1() every time the PID controller computes a new velocity
 * This function computes the required speed value for both motors and sends that data to the MD25
 */
void Robot::StraightTurn1(float Speed){
    float MotorCorrection=0, Error, LastError;
    float S1,S2;
    if(Speed > 65) Speed = 65;
    if(Speed < -128) Speed = -128;
    float GD1 = 1.5*3.1415*(180 - WidthRobot/2);
    float GD2 = 1.5*3.1415*(180 + WidthRobot/2);
    GD1 = int(360*GD1/CircumferenceWheel1 + 0.5);
    GD2 = int(360*GD2/CircumferenceWheel2 + 0.5);
    Error = E1 / GD1 * 100 - E2/GD2 * 100;
    //MotorCorrection = TKp * Error + TKd*(Error-LastError);
    LastError = Error;
    S1 = GD1/GD2*Speed;
    S2 = Speed + MotorCorrection;
 
    if(S1 > 127) S1 = 127;
    if(S2 > 127) S2 = 127;
    if(S1 < -128) S1 = -128;     // capping the values for the motor speeds, as a value bigger than 127 or smaller than -128 might fuck up the controller
    if(S2 < -128) S2 = -128;
    Serial.print(MotorCorrection);
    Serial.print("    ");
    Serial.print(Error);
    Serial.print("    ");
    Serial.print(S1);
    Serial.print("    ");
    Serial.println(S2);
    Wire.beginTransmission(Adress);
    Wire.write(Speed1);
    Wire.write(int(S1+0.5));
    Wire.endTransmission();       // set speed of motor 1 (left)
  
    Wire.beginTransmission(Adress);
    Wire.write(Speed2);
    Wire.write(int(S2+0.5));
    Wire.endTransmission();      // set speed of motor 2 (right)
}
/*
 * Similar to StraightTurn1()
 */
void Robot::StraightTurn2(float Speed){
    float MotorCorrection=0, Error, LastError;
    float S1,S2;
    if(Speed > 65) Speed = 65;
    if(Speed < -128) Speed = -128;
    float GD1 = 0.5*3.1415*(260 - WidthRobot/2);
    float GD2 = 0.5*3.1415*(260 + WidthRobot/2);
    GD1 = int(360*GD1/CircumferenceWheel1 + 0.5);
    GD2 = int(360*GD2/CircumferenceWheel2 + 0.5);
    Error = E1 / GD1 * 100 - E2/GD2 * 100;
    //MotorCorrection = TKp * Error + TKd*(Error-LastError);
    LastError = Error;
    S1 = GD1/GD2*Speed;
    S2 = Speed + MotorCorrection;
 
    if(S1 > 127) S1 = 127;
    if(S2 > 127) S2 = 127;
    if(S1 < -128) S1 = -128;     // capping the values for the motor speeds, as a value bigger than 127 or smaller than -128 might fuck up the controller
    if(S2 < -128) S2 = -128;
    Serial.print(MotorCorrection);
    Serial.print("    ");
    Serial.print(Error);
    Serial.print("    ");
    Serial.print(S1);
    Serial.print("    ");
    Serial.println(S2);
    Wire.beginTransmission(Adress);
    Wire.write(Speed1);
    Wire.write(int(S1+0.5));
    Wire.endTransmission();       // set speed of motor 1 (left)
  
    Wire.beginTransmission(Adress);
    Wire.write(Speed2);
    Wire.write(int(S2+0.5));
    Wire.endTransmission();      // set speed of motor 2 (right)
}
/*
 * Function used to signal each waypoint alomg the course by 3 short LED blinks and a short buzzer sound
 */
void Robot::LEDBlink(){
  tone(Buzzer, 600);
  digitalWrite(LED,HIGH);
  delay(150);
  digitalWrite(LED,LOW);
  delay(150);
  digitalWrite(LED,HIGH);
  delay(150);
  digitalWrite(LED,LOW);
  delay(150);
  digitalWrite(LED,HIGH);
  delay(150);
  digitalWrite(LED,LOW);
  noTone(Buzzer);
}
/*
 * This function is used to rotate the robot around its axis by a given number of degrees
 * Implements a PID  controller
 */
void Robot::SpinLeft(float Degrees){
  Wire.beginTransmission(Adress);
  Wire.write(Mode);
  Wire.write(0x01);            // sets the MD25 to mode 1
  Wire.endTransmission();
  
  Wire.beginTransmission(Adress);
  Wire.write(Command);
  Wire.write(0x20);            // sends byte to restore the encoders to 0
  Wire.endTransmission();
  float Error, LastError;
  float Speed;
  int n=0;
  Degrees = 3.1428 *WidthRobot * Degrees / ( CircumferenceWheel2);   // converts degrees to encoder ticks for the right wheel, because it's the one spinning forward
  Goal = 0;
  while(!Goal){
    ReadEncoders();
    Error = Degrees - E2;
    Speed = TKp * Error + TKd*(Error - LastError)+0.5;
    //Speed = 80;
    LastError = Error;
    if(Speed > 100) Speed = 100;
    if(Speed < -100) Speed = -100;
    Serial.print(Speed);
    Serial.print("   ");
    Serial.print(Error);
    Serial.println("   ");
    Wire.beginTransmission(Adress);
    Wire.write(Speed2);
    Wire.write(int(Speed+0.5));
    Wire.endTransmission();

    Wire.beginTransmission(Adress);
    Wire.write(Speed1);
    Wire.write(int(-Speed-0.5));
    Wire.endTransmission();
    if((Error < 5) && (Error >-5)) n++;         // every time the error is 0, the counter goes up by 1
    if((Error > 20)||(Error < -20)) n=0;         // random value, if the value of the error increases (overshoots), the counter resets
    if(n == 5) Goal = 1;       // random value, as soon as the counter reaches 10, it breaks out of the loop (it reaches the goal)
    Serial.println(Goal);    
  }
  Wire.beginTransmission(Adress);
  Wire.write(Speed1);
  Wire.write(0);
  Wire.endTransmission();       // set speed of motor 1 (left) to 0

  Wire.beginTransmission(Adress);
  Wire.write(Speed2);
  Wire.write(0);
  Wire.endTransmission();
}
/*
 * Very similar to SpinLeft(), only this function reverses the spin direction
 */
void Robot::SpinRight(float Degrees){
  Wire.beginTransmission(Adress);
  Wire.write(Mode);
  Wire.write(0x01);            // sets the MD25 to mode 1
  Wire.endTransmission();
  
  Wire.beginTransmission(Adress);
  Wire.write(Command);
  Wire.write(0x20);            // sends byte to restore the encoders to 0
  Wire.endTransmission();
  float Error, LastError;
  float Speed;
  int n=0;
  Degrees = 3.1428 *WidthRobot * Degrees / (CircumferenceWheel1);
  Goal = 0;
  while(!Goal){
    ReadEncoders();
    Error = Degrees - E1;
    //Serial.println(Error);
    Speed = TKp * Error + TKd*(Error - LastError)+0.5;
    //Speed = 80;
    LastError = Error;
    if(Speed > 100) Speed = 100;
    if(Speed < -100) Speed = -100;
    Serial.print(Speed);
    Serial.print("   ");
    Serial.print(Error);
    Serial.print("   ");
    Serial.println(n);
    Wire.beginTransmission(Adress);
    Wire.write(Speed2);
    Wire.write(int(-Speed-0.5));
    Wire.endTransmission();

    Wire.beginTransmission(Adress);
    Wire.write(Speed1);
    Wire.write(int(Speed+0.5));
    Wire.endTransmission();
    if((Error < 5) && (Error >-5)) n++;         // every time the error is 0, the counter goes up by 1
    if((Error > 20)&&(Error < -20)) n=0;         // random value, if the value of the error increases (overshoots), the counter resets
    if(n == 5) Goal = 1;       // random value, as soon as the counter reaches 10, it breaks out of the loop (it reaches the goal)
  }
  //Serial.print("FINISH");
   Wire.beginTransmission(Adress);
  Wire.write(Speed1);
  Wire.write(0);
  Wire.endTransmission();       // set speed of motor 1 (left) to 0

  Wire.beginTransmission(Adress);
  Wire.write(Speed2);
  Wire.write(0);
  Wire.endTransmission();
}
/*
 * Used to operate the servo that dispenses M&M
 */
void Robot::Dispensemm(int ServoAngle) {
  myServo.attach(ServoMotor);
  myServo.write(ServoAngle);
  Serial.print(ServoAngle);
  delay(200);
}


