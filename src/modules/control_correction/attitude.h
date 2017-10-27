#ifndef ATTITUDE_H
#define ATTITUDE_H

#define BUFLEN 512  //Max length of buffer
#define PORT 8888   //The port on which to send data

typedef struct{
  float roll;
  float pitch;
  float yaw;
  float thrust;
} correctionValues;

#endif
