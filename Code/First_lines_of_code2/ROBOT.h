#ifndef ROBOT_H_INCLUDED
#define ROBOT_H_INCLUDED
#include "Arduino.h"
#include "defines.h"

class Robot
{
  public:
    // Driving Functions
    void Forward(float distance);
    void Turn1();
    void Turn2();
    void Initialize();
    void SpinLeft(float Degrees);
    void SpinRight(float Degrees);
    // Encoder Reading
    void ReadEncoders();
    void LEDBlink();

    // Initialize
    
 
  private:
    
    bool Goal;
    int nor=0;
    long E1, E2;
    float SumError;
    void Drive(float Speed);
    void StraightTurn1(float Speed);
    void StraightTurn2(float Speed);
}
;
#endif

