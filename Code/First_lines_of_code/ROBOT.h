#ifndef ROBOT_H_INCLUDED
#define ROBOT_H_INCLUDED
#include "Arduino.h"
#include "defines.h"

class Robot
{
  public:
    // Driving Functions
    void Forward(int distance);
    void Turn(float Radius,int Degrees, bool Direction);
    
    // Encoder Reading
    void ReadEncoders();

    // Initialize
    void Initialize();
    bool Goal;
 
  private:
    long E1, E2;
    float SumError;
    void Drive(int Speed);
    void StraightTurn(float Radius, bool Direction, float Speed);
    
}
;
#endif
