#ifndef ROBOT_H_INCLUDED
#define ROBOT_H_INCLUDED
#include "Arduino.h"
#include "defines.h"

class Robot
{
  public:
    // Driving Functions
    void Forward(float distance);
    void SpinLeft(float angle);
    void SpinRight(float angle);
    void Turn(int ForwardSpeed, int TurnSpeed);
    void Drive(int Speed);

    // Encoder Reading
    void ReadEncoders();

    // Initialize
    void Initialize();

    // Global Variables
    long E1, E2;
    bool Goal;
}
;
#endif
