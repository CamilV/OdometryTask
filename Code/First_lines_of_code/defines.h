#ifndef DEFINES_H_INCLUDED
#define DEFINES_H_INCLUDED

// Memory Adress
#define Speed1     0x00
#define Speed2     0x01

#define Mode       0x0D
#define Encoder1   0x02
#define Encoder2   0x06
#define Command    0x0E
#define Adress     0x58

#define DSR        0x30
#define ESR        0x31

#define THRESHOLD  1000   // random value

// PID Forward Driving
#define DKp           0   // random value
#define DKd           0   // random value
#define DKi           0   // random value

// PID Distance
#define Kp            0   // random value
#define Kd            0   // random value
#define Ki            0   // random value

#endif
