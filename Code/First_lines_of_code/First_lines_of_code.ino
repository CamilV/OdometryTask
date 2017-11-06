#include "robot.cpp"
#include "ROBOT.h"

void setup() {
  
}

void loop() {
  Forward(856); // 13 to 11
  Turn(0, 143, 1); // Spin on 11
  Turn(180, 270, 0); // 11 to 10
  Turn(0, 90, 0); // Spin 10
  Forward(180); // 10 to 9
  Turn(0, 140, 1); // Spin on 9
  Forwad(622); // 9 to 8
  Turn(0, 50, 1); // Spin on 8
  Forward(400); // 8 to 7
  Turn(0, 90, 1); //Spin on 7
  Forward(400); // 7 to 6
  Turn(0, 90, 1); // Spin on 6
  Forward(400); // 6 to 5
  Turn(0, 90, 1); // Spin on 5
  Forward(660); // 5 to 4
  Turn(0, 90, 0); // Spin on 4
  Turn(260, 90 , 0); // 4 to 3
  Turn(0, 90, 1); // Spin on 3
  Forward(500); // 3 to 2
  Turn(0, 90, 1); // Spin on 2
  Forawrd(260) ; // 2 to 1
  Turn(0, 90, 0); // Spin on 1
  Forward(340); // 1 to 0
}
