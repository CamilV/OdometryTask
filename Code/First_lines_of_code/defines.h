#ifndef DEFINES_H_INCLUDED
#define DEFINES_H_INCLUDED

// pins
#define LED           4
#define Buzzer        5
#define ServoMotor    6

// Memory Adress
#define Speed1     0x00   // register to acces the speed of motor 1 (left)
#define Speed2     0x01   // register to acces the speed of motor 2 (right)

#define Mode       0xF   // register to acces the control mode (0 1 2 3)
#define Encoder1   0x02   // register to acces the encoder of motor 1
#define Encoder2   0x06   // register to acces the encoder of motor 2
#define Command    0x10   // register to acces the command register of the MD25
#define Adress     0x58   // adress of the MD25

#define DSR        0x30   // byte to send to disable automatic speed regulation to command
#define ESR        0x31   // byte to send to enable automatic speed regulation to command

#define THRESHOLD   700   // random value, threshold below which the PID controller for the distance function driving activates
// physical dimensions
#define CircumferenceWheel1 317.0   // mm   size of left wheel
#define CircumferenceWheel2 320.0   // mm   size of right wheel
#define WidthRobot          287.0   // mm random value

// PID Forward Driving(used in the Drive function, to make the robot drive straight)
#define DKp           0.6   // best value until now
#define DKd           0.2   // best value until now
#define DKi           0   // zero because the integral part doesnt work well with the PID Controller

// PID Turn Radius Degrees
#define TKp           0     // yet to be determined
#define TKd           0     // yet to be determined

// PID Turn Straight Degrees
#define DeKp          0     // yet to be determined
#define DeKd          0     // yet to be determined
#define DeKi          0     // yet to be determined

// PID Distance(used to compute the accelerations when reaching the desired distance)
#define Kp            0.47  // very good value, dont change without saving them somewhere
#define Kd            0.8   // very good value, dont change without saving them somewhere
#define Ki            0   // zero because we dont need the integral of the distance

#endif
