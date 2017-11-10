#ifndef ROBOT_H_INCLUDED
#define ROBOT_H_INCLUDED
#include "Arduino.h"
#include "defines.h"

class Robot
{
  public:
    // Driving Functions
    void Forward(float distance);
    void Turn(float Radius,int Degrees, bool Direction);
    void Initialize();
    void SpinLeft(float Degrees);
    void SpinRight(float Degrees);
    // Encoder Reading
    void ReadEncoders();

    // Initialize
    
 
  private:
    
    bool Goal;
    long E1, E2;
    float SumError;
    void Drive(float Speed);
    void StraightTurn(float Radius, bool Direction, float Speed);
    
}
;
#endif
