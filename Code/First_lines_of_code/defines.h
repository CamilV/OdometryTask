#ifndef DEFINES_H_INCLUDED
#define DEFINES_H_INCLUDED

// Memory Adress
#define Speed1     0x00   // register to acces the speed of motor 1 (left)
#define Speed2     0x01   // register to acces the speed of motor 2 (right)

#define Mode       0x0D   // register to acces the control mode (0 1 2 3)
#define Encoder1   0x02   // register to acces the encoder of motor 1
#define Encoder2   0x06   // register to acces the encoder of motor 2
#define Command    0x0E   // register to acces the command register of the MD25
#define Adress     0x58   // adress of the MD25

#define DSR        0x30   // byte to send to disable automatic speed regulation to command
#define ESR        0x31   // byte to send to enable automatic speed regulation to command

#define THRESHOLD   360   // random value, threshold below which the PID controller for the distance function driving activates
// physical dimensions
#define RadiusWheel 100  // mm random value
#define WidthRobot  250   // mm random value

// PID Forward Driving(used in the Drive function, to make the robot drive straight)
#define DKp           0   // random value
#define DKd           0   // random value
#define DKi           0   // random value

// PID Distance(used to compute the accelerations when reaching the desired distance)
#define Kp            0   // random value
#define Kd            0   // random value
#define Ki            0   // random value

#endif
