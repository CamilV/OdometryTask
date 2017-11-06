#ifndef ROBOT_H_INCLUDED
#define ROBOT_H_INCLUDED
#include "Arduino.h"
#include "defines.h"

class Robot
{
  public:
    // Driving Functions
    void Forward(int distance);
    void SpinLeft(float angle);
    void SpinRight(float angle);
    //void Turn(int ForwardSpeed, int TurnSpeed);
    void Drive(int Speed);
    void Turn(int Radius,int Degrees);
    // Encoder Reading
    void ReadEncoders();

    // Initialize
    void Initialize();
    bool Goal;
  private:
    long E1, E2;
    
}
;
#endif
