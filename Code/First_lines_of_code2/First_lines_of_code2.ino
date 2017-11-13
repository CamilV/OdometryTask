#include "defines.h"
#include "ROBOT.h"
#include <Wire.h>

Robot R;

void setup() {
  R.Initialize(); 
  // R.Forward(1000);
  //tone(Buzzer,50,1000);
  R.Forward(428); // 13 to 12
  R.LEDBlink();
  
  R.Forward(360); // 12 to 11
  R.LEDBlink();
  R.SpinRight(130.4);
  
  R.Turn1(); // 11 to 10
  R.Forward(30);
  R.Dispensemm(42);
  R.LEDBlink();
  
  R.SpinLeft(84);   // 10 to 9
  R.Forward(180); 
  R.LEDBlink();
  R.SpinRight(130);
  R.Forward(620); // 9 to 8
  R.Dispensemm(72);
  R.LEDBlink();
  
  R.SpinRight(36);
  R.Forward(385); // 8 to 7
  R.LEDBlink();
  R.SpinRight(84);
  R.Forward(384); // 7 to 6
  R.Dispensemm(100);
  R.LEDBlink();
  
  R.SpinRight(80);
  R.Forward(400); // 6 to 5
  R.LEDBlink();
  R.SpinRight(84);
  R.Forward(636); // 5 to 4
  R.Dispensemm(129);
  R.LEDBlink();
    
  R.SpinLeft(90);
  R.Turn2(); // 4 to 3
  R.SpinRight(83);
  R.LEDBlink();
  R.Forward(480); // 3 to 2
  R.Dispensemm(158);
  R.LEDBlink();
  
  R.SpinRight(83);
  R.Forward(260); // 2 to 1
  R.LEDBlink();
  R.SpinLeft(90);
  R.Forward(340); // 1 to 0
  R.LEDBlink();
};

void loop() {
  
//  Wire.beginTransmission(Adress);
//  Wire.write(Speed1);
//  Wire.write(255);
//  Wire.endTransmission(); 
//  delay(1000);
//  R.Forward(100);
  
//  R.Forward(856); // 13 to 11
//  R.Turn(0, 143, 1); // Spin on 11
//  R.Turn(180, 270, 0); // 11 to 10
//  R.Turn(0, 90, 0); // Spin 10
//  R.Forward(180); // 10 to 9
//  R.Turn(0, 140, 1); // Spin on 9
//  R.Forward(622); // 9 to 8
//  R.Turn(0, 50, 1); // Spin on 8
//  R.Forward(400); // 8 to 7
//  R.Turn(0, 90, 1); //Spin on 7
//  R.Forward(400); // 7 to 6
//  R.Turn(0, 90, 1); // Spin on 6
//  R.Forward(400); // 6 to 5
//  R.Turn(0, 90, 1); // Spin on 5
//  R.Forward(660); // 5 to 4
//  R.Turn(0, 90, 0); // Spin on 4
//  R.Turn(260, 90 , 0); // 4 to 3
//  R.Turn(0, 90, 1); // Spin on 3
//  R.Forward(500); // 3 to 2
//  R.Turn(0, 90, 1); // Spin on 2
//  R.Forward(260) ; // 2 to 1
//  R.Turn(0, 90, 0); // Spin on 1
//  R.Forward(340); // 1 to 0
}

