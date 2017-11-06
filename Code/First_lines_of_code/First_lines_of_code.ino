#include "defines.h"
#include "ROBOT.h"

Robot R;

void setup() {
  
};

void loop() {
  R.Forward(856); // 13 to 11
  R.Turn(0, 143, 1); // Spin on 11
  R.Turn(180, 270, 0); // 11 to 10
  R.Turn(0, 90, 0); // Spin 10
  R.Forward(180); // 10 to 9
  R.Turn(0, 140, 1); // Spin on 9
  R.Forward(622); // 9 to 8
  R.Turn(0, 50, 1); // Spin on 8
  R.Forward(400); // 8 to 7
  R.Turn(0, 90, 1); //Spin on 7
  R.Forward(400); // 7 to 6
  R.Turn(0, 90, 1); // Spin on 6
  R.Forward(400); // 6 to 5
  R.Turn(0, 90, 1); // Spin on 5
  R.Forward(660); // 5 to 4
  R.Turn(0, 90, 0); // Spin on 4
  R.Turn(260, 90 , 0); // 4 to 3
  R.Turn(0, 90, 1); // Spin on 3
  R.Forward(500); // 3 to 2
  R.Turn(0, 90, 1); // Spin on 2
  R.Forward(260) ; // 2 to 1
  R.Turn(0, 90, 0); // Spin on 1
  R.Forward(340); // 1 to 0
}
